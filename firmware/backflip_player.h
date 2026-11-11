/* backflip_player.h - a keyframe player for a 12-servo quadruped backflip.
 *
 * Hardware-agnostic and allocation-free: you call bf_tick() from your servo loop with the time in ms, it
 * writes the 12 servo targets (ticks, 0..1023) to send now.  Port it by wiring bf_tick() to your servo bus.
 *
 *   bf_player_t p;
 *   uint16_t stand[12];   read_servo_positions(stand);         // where the robot stands right now
 *   if (bf_start(&p, &BF_K7_405, stand, millis()) != BF_OK) { refuse: a joint would hit its end stop }
 *   every 5 ms:  if (bf_tick(&p, millis(), targets)) send_targets(targets); else done
 *
 * What a table is (see presets/<name>.h, generated from ../presets/<name>.json):
 *   frames[f][12]  servo targets in ticks RELATIVE to the stand (frame 0 is normally all zeros)
 *   move_ms[f]     the servos ramp in a straight line from where they were told to be to frame f in this time
 *   hold_ms[f]     then wait this long; frame f+1 starts move_ms[f] * blend_pct[f] / 100 + hold_ms[f] after f
 * The playback is anchored to the pose the robot stands in when it starts (bf_start), so it does not depend on
 * the calibration of the robot the table was made on.  Same arithmetic as backflip/keyframes.py Preset.play()
 * (tests/test_player.py runs both and compares every tick).
 */
#ifndef BACKFLIP_PLAYER_H
#define BACKFLIP_PLAYER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BF_SERVOS     12
#define BF_TICK_MIN   0
#define BF_TICK_MAX   1023

typedef struct {
    const char     *name;
    int             n_frames;
    const int16_t (*frames)[BF_SERVOS];
    const uint16_t *move_ms;
    const uint16_t *hold_ms;
    const uint8_t  *blend_pct;
} bf_table_t;

typedef enum {
    BF_OK = 0,
    BF_ERR_EMPTY = -1,        /* no frames                                                   */
    BF_ERR_RAIL = -2,         /* from this stand a frame would pass a servo's end (see bf_rail_*) */
} bf_status_t;

typedef struct {
    const bf_table_t *table;
    uint16_t stand[BF_SERVOS];      /* anchor: the pose it started from                          */
    float    goal[BF_SERVOS];       /* what the servos are told now (ticks)                      */
    float    from[BF_SERVOS];       /* where the current ramp started                            */
    int      frame;                 /* the frame being played                                    */
    uint32_t t_arm;                 /* ms: when it started                                       */
    uint32_t t_next;                /* ms: when the next frame starts                            */
    uint32_t t_end;                 /* ms: when the last frame's hold ended (0 = not yet)        */
    uint32_t hold_end_ms;           /* keep sending the last pose this long after the end        */
    int      playing;
    /* the worst end-stop overrun found by bf_start (for an error message) */
    int      rail_servo, rail_frame, rail_over;
} bf_player_t;

/* Check the table against `stand` and start playing at `now_ms`. Returns BF_OK, or an error and plays nothing. */
bf_status_t bf_start(bf_player_t *p, const bf_table_t *t, const uint16_t stand[BF_SERVOS], uint32_t now_ms);

/* Advance to now_ms and write the 12 servo targets (ticks).  Returns 1 while playing, 0 when done. */
int bf_tick(bf_player_t *p, uint32_t now_ms, uint16_t out[BF_SERVOS]);

/* Stop now; the servos keep the last targets they were sent. */
void bf_stop(bf_player_t *p);

/* Total length of a table, ms (without hold_end_ms). */
uint32_t bf_duration_ms(const bf_table_t *t);

#ifdef __cplusplus
}
#endif
#endif
