/*
 * demo_wave.h - pulse-MIG waveform generator demo for NUCLEO-F072RB.
 *
 * Analog output: DAC_OUT2 = PA5 (Arduino D13, on-board LED LD2), 3.3 V = 500 A.
 * (PA4 / DAC_OUT1 is selectable with DEMO_DAC_CHANNEL in demo_wave.c.)
 * Digital markers: PB3 (D3) = RISE (scope trigger), PB5 (D4) = short circuit
 * states, PB4 (D5) = ISR busy (shows CPU load).
 * Button B1 (PC13): injects a simulated short circuit at the next BASE phase.
 */
#ifndef DEMO_WAVE_H
#define DEMO_WAVE_H

#include "weld_core.h"

/* Edit these in the debugger (Live Expressions) to change the waveform. */
typedef struct {
    volatile float ip_a;      /* peak current, A      */
    volatile float ib_a;      /* base current, A      */
    volatile float tp_s;      /* peak duration, s     */
    volatile float freq_hz;   /* pulse frequency, Hz  */
} demo_cfg_t;

extern demo_cfg_t demo_cfg;

void demo_init(void);   /* once, after all MX_*_Init() calls           */
void demo_poll(void);   /* in while(1): applies changes of demo_cfg    */
void demo_tick(void);   /* from TIM6_DAC_IRQHandler (10 kHz)           */

#endif /* DEMO_WAVE_H */
