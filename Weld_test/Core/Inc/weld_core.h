/*
 * weld_core.h - hardware-independent welding algorithms (plain C99, float).
 *
 * Nothing here touches registers: the core only receives measured U/I and
 * returns a current reference.  All numeric defaults are starting points
 * and MUST be tuned on a rig with logged I(t)/U(t) and high-speed video.
 */
#ifndef WELD_CORE_H
#define WELD_CORE_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Synergy: wire feed speed -> pulse parameters (one table per wire/gas) */
/* ------------------------------------------------------------------ */
typedef struct {
    float wfs_m_min;   /* wire feed speed, m/min            */
    float ip_a;        /* peak current, A                   */
    float ib_a;        /* base current, A                   */
    float tp_s;        /* nominal peak time, s              */
    float freq_hz;     /* nominal pulse frequency, Hz       */
    float u_arc_v;     /* target mean arc voltage, V        */
} synergy_point_t;

typedef struct {
    const synergy_point_t *pts;   /* sorted by wfs_m_min ascending */
    uint8_t n;
} synergy_table_t;

void synergy_lookup(const synergy_table_t *t, float wfs, synergy_point_t *out);

/* ------------------------------------------------------------------ */
/* Arc signal conditioning: filtered U and dU/dt, runs in the fast tick */
/* ------------------------------------------------------------------ */
typedef struct {
    float dt;          /* tick period, s              */
    float a_u;         /* LP coefficient for U        */
    float a_d;         /* LP coefficient for dU/dt    */
    float u_lp;        /* filtered arc voltage, V     */
    float u_prev;
    float dudt;        /* filtered dU/dt, V/s         */
    bool  primed;
} arc_det_t;

void arc_det_init(arc_det_t *d, float dt, float u_cut_hz, float dudt_cut_hz);
void arc_det_update(arc_det_t *d, float u_meas);

/* ------------------------------------------------------------------ */
/* Pulse MIG waveform state machine (fast tick, e.g. 25-50 kHz)        */
/* ------------------------------------------------------------------ */
typedef enum {
    PS_OFF = 0,
    PS_RISE,        /* base -> peak ramp                              */
    PS_PEAK,        /* droplet forms and detaches                     */
    PS_FALL,        /* soft ramp down                                 */
    PS_BASE,        /* keeps arc alive, sets heat input               */
    PS_SC_LOW,      /* short circuit: drop current                    */
    PS_SC_PINCH,    /* short circuit: controlled pinch ramp           */
    PS_SC_DROP      /* neck about to break: cut to base               */
} pulse_state_t;

typedef struct {
    float ip_a, ib_a, tp_s, freq_hz;
} pulse_params_t;

typedef struct {
    /* ---- configuration ---- */
    float dt;
    float t_rise_s, t_fall_s;
    float tp_min_frac;        /* detach may end the peak after this share of tp */
    float dudt_detach_v_s;    /* dU/dt (+) that marks droplet detachment        */
    float u_sc_v;             /* U below this = short circuit                   */
    float u_arc_ok_v;         /* U above this = arc is back                     */
    float sc_debounce_s;
    float i_sc_low_a, t_sc_low_s;
    float pinch_slope_a_s, i_pinch_max_a;
    float dudt_break_v_s;     /* dU/dt (+) precursor of neck rupture            */
    float t_sc_drop_max_s;

    /* ---- runtime ---- */
    pulse_state_t state;
    pulse_params_t p;         /* active parameters                              */
    pulse_params_t pending;   /* written by slow task                           */
    volatile bool pending_valid;
    float t_state, t_cycle, t_sc;
    float i_ref;
    uint32_t pulses, short_circuits;
} pulse_fsm_t;

void  pulse_init(pulse_fsm_t *f, float dt);
void  pulse_set_params(pulse_fsm_t *f, const pulse_params_t *p); /* slow task */
void  pulse_start(pulse_fsm_t *f);
void  pulse_stop(pulse_fsm_t *f);
float pulse_tick(pulse_fsm_t *f, const arc_det_t *d);            /* fast tick */

/* ------------------------------------------------------------------ */
/* Arc length regulator (slow, ~1 kHz): trims pulse frequency          */
/* ------------------------------------------------------------------ */
typedef struct {
    float dt;
    float kp;              /* Hz per V                 */
    float ki;              /* Hz per (V*s)             */
    float f_nom, f_min, f_max;
    float u_target;
    float integ;           /* Hz */
} arc_reg_t;

void  arc_reg_init(arc_reg_t *r, float dt, float kp, float ki);
void  arc_reg_setpoint(arc_reg_t *r, float f_nom, float u_target, float f_min, float f_max);
float arc_reg_step(arc_reg_t *r, float u_avg);

/* ------------------------------------------------------------------ */
/* MMA: drooping characteristic + arc force + hot start + anti-stick   */
/* ------------------------------------------------------------------ */
typedef struct {
    float dt;
    float i_set_a, i_min_a, i_max_a;
    float u_af_v, k_af_a_v;       /* arc-force threshold and gain          */
    float u_stick_v, t_stick_s;   /* anti-stick                            */
    float hot_boost_a, t_hot_s;   /* hot start                             */
    float t_run, t_low;
} mma_t;

void  mma_init(mma_t *m, float dt);
void  mma_reset(mma_t *m);                 /* call when the arc is struck */
float mma_tick(mma_t *m, float u_arc);

#endif /* WELD_CORE_H */
