#include "weld_core.h"

#define TWO_PI 6.2831853f

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float lerpf(float a, float b, float k)
{
    return a + (b - a) * k;
}

/* ---------------------------- synergy ---------------------------- */
void synergy_lookup(const synergy_table_t *t, float wfs, synergy_point_t *out)
{
    const synergy_point_t *p = t->pts;
    uint8_t n = t->n;

    if (wfs <= p[0].wfs_m_min)     { *out = p[0];     return; }
    if (wfs >= p[n - 1].wfs_m_min) { *out = p[n - 1]; return; }

    for (uint8_t i = 1; i < n; i++) {
        if (wfs <= p[i].wfs_m_min) {
            const synergy_point_t *a = &p[i - 1], *b = &p[i];
            float k = (wfs - a->wfs_m_min) / (b->wfs_m_min - a->wfs_m_min);
            out->wfs_m_min = wfs;
            out->ip_a      = lerpf(a->ip_a,    b->ip_a,    k);
            out->ib_a      = lerpf(a->ib_a,    b->ib_a,    k);
            out->tp_s      = lerpf(a->tp_s,    b->tp_s,    k);
            out->freq_hz   = lerpf(a->freq_hz, b->freq_hz, k);
            out->u_arc_v   = lerpf(a->u_arc_v, b->u_arc_v, k);
            return;
        }
    }
}

/* -------------------------- arc detector ------------------------- */
static float lp_alpha(float dt, float fc)
{
    float w = TWO_PI * fc * dt;
    return w / (1.0f + w);
}

void arc_det_init(arc_det_t *d, float dt, float u_cut_hz, float dudt_cut_hz)
{
    d->dt = dt;
    d->a_u = lp_alpha(dt, u_cut_hz);
    d->a_d = lp_alpha(dt, dudt_cut_hz);
    d->u_lp = d->u_prev = d->dudt = 0.0f;
    d->primed = false;
}

void arc_det_update(arc_det_t *d, float u_meas)
{
    if (!d->primed) {
        d->u_lp = d->u_prev = u_meas;
        d->dudt = 0.0f;
        d->primed = true;
        return;
    }
    d->u_lp += d->a_u * (u_meas - d->u_lp);
    float raw = (d->u_lp - d->u_prev) / d->dt;
    d->u_prev = d->u_lp;
    d->dudt += d->a_d * (raw - d->dudt);
}

/* ---------------------------- pulse FSM -------------------------- */
static void enter(pulse_fsm_t *f, pulse_state_t s)
{
    f->state = s;
    f->t_state = 0.0f;
}

static void start_cycle(pulse_fsm_t *f)
{
    if (f->pending_valid) {
        f->p = f->pending;
        f->pending_valid = false;
    }
    f->t_cycle = 0.0f;
    f->pulses++;
    enter(f, PS_RISE);
}

void pulse_init(pulse_fsm_t *f, float dt)
{
    *f = (pulse_fsm_t){0};
    f->dt = dt;
    f->t_rise_s = 150e-6f;
    f->t_fall_s = 600e-6f;
    f->tp_min_frac = 0.6f;
    f->dudt_detach_v_s = 20000.0f;
    f->u_sc_v = 8.0f;
    f->u_arc_ok_v = 14.0f;
    f->sc_debounce_s = 100e-6f;
    f->i_sc_low_a = 40.0f;
    f->t_sc_low_s = 500e-6f;
    f->pinch_slope_a_s = 400000.0f;     /* 400 A/ms */
    f->i_pinch_max_a = 300.0f;
    f->dudt_break_v_s = 30000.0f;
    f->t_sc_drop_max_s = 3e-3f;
    f->state = PS_OFF;
}

void pulse_set_params(pulse_fsm_t *f, const pulse_params_t *p)
{
    f->pending_valid = false;
    f->pending = *p;
    f->pending_valid = true;
}

void pulse_start(pulse_fsm_t *f)
{
    if (f->state != PS_OFF) return;
    if (f->pending_valid) { f->p = f->pending; f->pending_valid = false; }
    f->t_sc = 0.0f;
    start_cycle(f);
}

void pulse_stop(pulse_fsm_t *f)
{
    f->state = PS_OFF;
    f->i_ref = 0.0f;
}

static bool short_circuit(pulse_fsm_t *f, const arc_det_t *d)
{
    if (d->u_lp < f->u_sc_v) f->t_sc += f->dt; else f->t_sc = 0.0f;
    return f->t_sc >= f->sc_debounce_s;
}

float pulse_tick(pulse_fsm_t *f, const arc_det_t *d)
{
    const pulse_params_t *p = &f->p;

    f->t_state += f->dt;
    f->t_cycle += f->dt;

    switch (f->state) {
    case PS_OFF:
        f->i_ref = 0.0f;
        break;

    case PS_RISE:
        f->i_ref = lerpf(p->ib_a, p->ip_a, clampf(f->t_state / f->t_rise_s, 0.0f, 1.0f));
        if (f->t_state >= f->t_rise_s) enter(f, PS_PEAK);
        break;

    case PS_PEAK: {
        f->i_ref = p->ip_a;
        bool detached = (f->t_state >= f->tp_min_frac * p->tp_s) &&
                        (d->dudt > f->dudt_detach_v_s);
        if (detached || f->t_state >= p->tp_s) enter(f, PS_FALL);
        break;
    }

    case PS_FALL:
        f->i_ref = lerpf(p->ip_a, p->ib_a, clampf(f->t_state / f->t_fall_s, 0.0f, 1.0f));
        if (short_circuit(f, d)) { f->short_circuits++; enter(f, PS_SC_LOW); break; }
        if (f->t_state >= f->t_fall_s) enter(f, PS_BASE);
        break;

    case PS_BASE:
        f->i_ref = p->ib_a;
        if (short_circuit(f, d)) { f->short_circuits++; enter(f, PS_SC_LOW); break; }
        if (f->t_cycle >= 1.0f / p->freq_hz) start_cycle(f);
        break;

    case PS_SC_LOW:
        f->i_ref = f->i_sc_low_a;
        if (d->u_lp >= f->u_arc_ok_v) { f->t_cycle = 0.0f; enter(f, PS_BASE); break; }
        if (f->t_state >= f->t_sc_low_s) enter(f, PS_SC_PINCH);
        break;

    case PS_SC_PINCH:
        f->i_ref = clampf(f->i_sc_low_a + f->pinch_slope_a_s * f->t_state,
                          0.0f, f->i_pinch_max_a);
        if (d->dudt >= f->dudt_break_v_s) { enter(f, PS_SC_DROP); break; }
        if (d->u_lp >= f->u_arc_ok_v)     { f->t_cycle = 0.0f; enter(f, PS_BASE); }
        break;

    case PS_SC_DROP:
        f->i_ref = p->ib_a;
        if (d->u_lp >= f->u_arc_ok_v)        { f->t_cycle = 0.0f; enter(f, PS_BASE); break; }
        if (f->t_state >= f->t_sc_drop_max_s) enter(f, PS_SC_LOW);
        break;
    }
    return f->i_ref;
}

/* ------------------------ arc length regulator ------------------- */
void arc_reg_init(arc_reg_t *r, float dt, float kp, float ki)
{
    *r = (arc_reg_t){0};
    r->dt = dt; r->kp = kp; r->ki = ki;
}

void arc_reg_setpoint(arc_reg_t *r, float f_nom, float u_target, float f_min, float f_max)
{
    r->f_nom = f_nom; r->u_target = u_target;
    r->f_min = f_min; r->f_max = f_max;
}

/* Sign convention: arc too long (u_avg > target) -> LOWER frequency.
 * Higher mean current melts wire faster and lengthens the arc, so this
 * sign must be confirmed on the rig before closing the loop on a real arc. */
float arc_reg_step(arc_reg_t *r, float u_avg)
{
    float e = r->u_target - u_avg;
    r->integ += r->ki * e * r->dt;
    r->integ = clampf(r->integ, r->f_min - r->f_nom, r->f_max - r->f_nom);
    return clampf(r->f_nom + r->kp * e + r->integ, r->f_min, r->f_max);
}

/* -------------------------------- MMA ---------------------------- */
void mma_init(mma_t *m, float dt)
{
    *m = (mma_t){0};
    m->dt = dt;
    m->i_set_a = 150.0f; m->i_min_a = 15.0f; m->i_max_a = 500.0f;
    m->u_af_v = 15.0f;   m->k_af_a_v = 12.0f;
    m->u_stick_v = 5.0f; m->t_stick_s = 0.8f;
    m->hot_boost_a = 40.0f; m->t_hot_s = 0.15f;
}

void mma_reset(mma_t *m)
{
    m->t_run = 0.0f;
    m->t_low = 0.0f;
}

float mma_tick(mma_t *m, float u_arc)
{
    m->t_run += m->dt;

    if (u_arc < m->u_stick_v) m->t_low += m->dt; else m->t_low = 0.0f;
    if (m->t_low >= m->t_stick_s) return m->i_min_a;        /* anti-stick */

    float i = m->i_set_a;
    if (m->t_run < m->t_hot_s) i += m->hot_boost_a;          /* hot start  */
    if (u_arc < m->u_af_v)     i += m->k_af_a_v * (m->u_af_v - u_arc); /* arc force */
    return clampf(i, m->i_min_a, m->i_max_a);
}
