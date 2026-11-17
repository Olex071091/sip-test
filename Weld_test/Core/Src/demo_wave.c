/*
 * demo_wave.c - runs the pulse FSM from weld_core on a NUCLEO-F072RB and
 * shows the current REFERENCE on DAC_OUT2 (PA5, see DEMO_DAC_CHANNEL) for an oscilloscope.
 *
 * The F072 is a Cortex-M0 WITHOUT an FPU: every float operation is software
 * emulated.  That is why the tick here is 10 kHz (100 us) and the waveform
 * is slowed down compared to a real welder (rise 0.5 ms, fall 1.5 ms,
 * 100 Hz).  Measure the real load on PB4 (see demo_tick) and lower TICK_HZ
 * if the pulse is wider than ~70% of the tick period.
 *
 * Keep TICK_HZ in sync with the TIM6 settings in CubeMX:
 *   TIM6 clock 48 MHz, Prescaler 47 (-> 1 MHz), Period 99  =>  10 kHz.
 */
#include "main.h"          /* CubeMX header: brings CMSIS device registers (HAL or LL project) */
#include "demo_wave.h"

#define TICK_HZ          10000.0f
#define TICK_DT_S        (1.0f / TICK_HZ)

#define I_FULL_SCALE_A   500.0f       /* DAC 4095 = 3.3 V = 500 A   */
#define DAC_MAX          4095.0f

/* DAC channel: 2 = DAC_OUT2 on PA5 (default, leaves PA4 free for the
 * 74HC595 display wiring), 1 = DAC_OUT1 on PA4. */
#ifndef DEMO_DAC_CHANNEL
#define DEMO_DAC_CHANNEL 2
#endif
#if DEMO_DAC_CHANNEL == 2
  #define DEMO_DAC_EN     DAC_CR_EN2
  #define DEMO_DAC_DHR    DAC->DHR12R2
#else
  #define DEMO_DAC_EN     DAC_CR_EN1
  #define DEMO_DAC_DHR    DAC->DHR12R1
#endif

#define PIN_SYNC         (1u << 3)       /* PB3, high during RISE            */
#define PIN_BUSY         (1u << 4)       /* PB4, high while ISR is running   */
#define PIN_SC           (1u << 5)       /* PB5, high in short-circuit states*/
#define PIN_BTN          (1u << 13)      /* PC13, button B1, LOW when pressed */

/* GPIOx->BSRR sets pins, GPIOx->BRR clears pins (plain CMSIS, no HAL/LL needed) */
#define PB_SET(m)        (GPIOB->BSRR = (m))
#define PB_CLR(m)        (GPIOB->BRR  = (m))

/* ---- toy arc model (same idea as test_host.c, slowed down) ---- */
#define PLANT_TAU_S      300e-6f
#define K_PLANT          (TICK_DT_S / PLANT_TAU_S)
#define SC_HOLD_TICKS    20           /* 2 ms short circuit            */
#define SC_RAMP_TICKS    4            /* then U climbs: neck ruptures  */

demo_cfg_t demo_cfg = { 368.0f, 56.0f, 2.0e-3f, 100.0f };

static pulse_fsm_t    fsm;
static arc_det_t      det;
static pulse_params_t applied;
static float          i_sim;
static uint8_t        btn_prev;
static uint8_t        sc_armed;
static int            sc_tick = -1;   /* -1 = no short-circuit event running */

static float clamp_dac(float x)
{
    return x < 0.0f ? 0.0f : (x > DAC_MAX ? DAC_MAX : x);
}

void demo_init(void)
{
    pulse_init(&fsm, TICK_DT_S);
    /* slowed-down timing, see header comment */
    fsm.t_rise_s        = 500e-6f;
    fsm.t_fall_s        = 1.5e-3f;
    fsm.dudt_detach_v_s = 5000.0f;
    fsm.sc_debounce_s   = 200e-6f;
    fsm.t_sc_low_s      = 1.0e-3f;
    fsm.pinch_slope_a_s = 100000.0f;   /* 100 A/ms */
    fsm.dudt_break_v_s  = 15000.0f;
    fsm.t_sc_drop_max_s = 6e-3f;

    arc_det_init(&det, TICK_DT_S, 2000.0f, 1500.0f);

    applied.ip_a = demo_cfg.ip_a;
    applied.ib_a = demo_cfg.ib_a;
    applied.tp_s = demo_cfg.tp_s;
    applied.freq_hz = demo_cfg.freq_hz;
    pulse_set_params(&fsm, &applied);
    pulse_start(&fsm);

    DAC->CR    |= DEMO_DAC_EN;          /* switch the DAC channel on          */
    TIM6->SR    = ~TIM_SR_UIF;          /* clear a stale update flag          */
    TIM6->DIER |= TIM_DIER_UIE;         /* update interrupt on                */
    TIM6->CR1  |= TIM_CR1_CEN;          /* start the 10 kHz timer             */
}

/* main loop: pick up edits of demo_cfg made in the debugger */
void demo_poll(void)
{
    float ip = demo_cfg.ip_a, ib = demo_cfg.ib_a;
    float tp = demo_cfg.tp_s, f  = demo_cfg.freq_hz;

    if (ip != applied.ip_a || ib != applied.ib_a ||
        tp != applied.tp_s || f  != applied.freq_hz) {
        applied.ip_a = ip;  applied.ib_a = ib;
        applied.tp_s = tp;  applied.freq_hz = f;
        pulse_set_params(&fsm, &applied);       /* used from the next pulse */
    }
}

/* TIM6 update interrupt, 10 kHz */
void demo_tick(void)
{
    if (!(TIM6->SR & TIM_SR_UIF)) return;
    TIM6->SR = ~TIM_SR_UIF;             /* flag is cleared here, HAL handler sees nothing */

    PB_SET(PIN_BUSY);

    /* button B1 (PC13) is LOW when pressed: arm one short circuit */
    uint8_t btn = (GPIOC->IDR & PIN_BTN) ? 0u : 1u;
    if (btn && !btn_prev) sc_armed = 1;
    btn_prev = btn;
    if (sc_armed && sc_tick < 0 && fsm.state == PS_BASE) {
        sc_armed = 0;
        sc_tick = 0;
    }

    /* toy arc voltage */
    float u = 15.0f + 0.033f * i_sim;
    if (fsm.state == PS_PEAK && fsm.t_state > 0.8f * fsm.p.tp_s)
        u += 3.0f;                              /* fake droplet detachment */
    if (sc_tick >= 0) {
        u = (sc_tick < SC_HOLD_TICKS)
                ? 3.0f
                : 3.0f + 7.0f * (float)(sc_tick - SC_HOLD_TICKS + 1);
        if (++sc_tick >= SC_HOLD_TICKS + SC_RAMP_TICKS) sc_tick = -1;
    }

    arc_det_update(&det, u);
    float i_ref = pulse_tick(&fsm, &det);
    i_sim += (i_ref - i_sim) * K_PLANT;

    /* Вывод уставки тока i_ref */
    /*DEMO_DAC_DHR = (uint32_t)clamp_dac(i_ref * (DAC_MAX / I_FULL_SCALE_A)); */

    /* Масштабируем напряжение: DAC 4095 = 33.0 В */
    #define U_FULL_SCALE_V 33.0f
    DEMO_DAC_DHR = (uint32_t)clamp_dac(det.u_lp * (DAC_MAX / U_FULL_SCALE_V));

    if (fsm.state == PS_RISE)     PB_SET(PIN_SYNC);
    else                          PB_CLR(PIN_SYNC);
    if (fsm.state >= PS_SC_LOW)   PB_SET(PIN_SC);
    else                          PB_CLR(PIN_SC);

    PB_CLR(PIN_BUSY);
}
