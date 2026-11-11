/* backflip_player.c - see backflip_player.h */
#include "backflip_player.h"

#include <string.h>

static uint32_t span_ms(const bf_table_t *t, int f)
{
    return (uint32_t)t->move_ms[f] * t->blend_pct[f] / 100u + t->hold_ms[f];
}

uint32_t bf_duration_ms(const bf_table_t *t)
{
    uint32_t d = 0;
    for (int f = 0; f < t->n_frames; f++) d += span_ms(t, f);
    return d;
}

static float target(const bf_player_t *p, int f, int i)
{
    return (float)p->stand[i] + (float)p->table->frames[f][i];
}

static void arm(bf_player_t *p, int f, uint32_t now)
{
    p->frame = f;
    p->t_arm = now;
    p->t_next = now + span_ms(p->table, f);
    memcpy(p->from, p->goal, sizeof p->from);
}

bf_status_t bf_start(bf_player_t *p, const bf_table_t *t, const uint16_t stand[BF_SERVOS], uint32_t now_ms)
{
    memset(p, 0, sizeof *p);
    if (!t || t->n_frames < 1) return BF_ERR_EMPTY;
    p->table = t;
    p->hold_end_ms = 600;
    /* refuse a table that would drive a joint past its end stop from this stand: clipping it would change the
     * flip, and the robot would land wrong */
    for (int f = 0; f < t->n_frames; f++)
        for (int i = 0; i < BF_SERVOS; i++) {
            int v = (int)stand[i] + t->frames[f][i];
            int over = v < BF_TICK_MIN ? BF_TICK_MIN - v : (v > BF_TICK_MAX ? v - BF_TICK_MAX : 0);
            if (over > p->rail_over) { p->rail_over = over; p->rail_servo = i + 1; p->rail_frame = f; }
        }
    if (p->rail_over) return BF_ERR_RAIL;
    for (int i = 0; i < BF_SERVOS; i++) { p->stand[i] = stand[i]; p->goal[i] = (float)stand[i]; }
    arm(p, 0, now_ms);
    p->playing = 1;
    return BF_OK;
}

int bf_tick(bf_player_t *p, uint32_t now, uint16_t out[BF_SERVOS])
{
    if (!p->playing) return 0;
    const bf_table_t *t = p->table;
    /* signed compares survive the millis() wrap-around */
    if (!p->t_end && (int32_t)(now - p->t_next) >= 0) {
        if (p->frame + 1 >= t->n_frames) p->t_end = now ? now : 1;
        else arm(p, p->frame + 1, now);
    }
    uint32_t dt = now - p->t_arm, ms = t->move_ms[p->frame];
    for (int i = 0; i < BF_SERVOS; i++) {
        float to = target(p, p->frame, i), v = to;
        if (ms && dt < ms) v = p->from[i] + (to - p->from[i]) * (float)dt / (float)ms;
        if (v < BF_TICK_MIN) v = BF_TICK_MIN;
        if (v > BF_TICK_MAX) v = BF_TICK_MAX;
        p->goal[i] = (float)(int)(v + 0.5f);
        out[i] = (uint16_t)p->goal[i];
    }
    if (p->t_end && (int32_t)(now - (p->t_end + p->hold_end_ms)) >= 0) p->playing = 0;
    return 1;
}

void bf_stop(bf_player_t *p) { p->playing = 0; }
