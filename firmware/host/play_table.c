/* play_table.c - run the player on a PC and print every 5 ms tick row as CSV (used by tests/test_player.py).
 *   make -C firmware/host && ./firmware/host/play_table k7_405 > ticks.csv */
#include <stdio.h>
#include <string.h>

#include "backflip_player.h"
#include "presets/all_presets.h"

int main(int argc, char **argv)
{
    const char *want = argc > 1 ? argv[1] : "k7_405";
    const bf_table_t *t = NULL;
    const uint16_t *stand = NULL;
    for (int k = 0; k < BF_N_PRESETS; k++)
        if (!strcmp(want, bf_presets[k].key)) { t = bf_presets[k].table; stand = bf_presets[k].stand; }
    if (!t) { fprintf(stderr, "no preset %s\n", want); return 2; }

    bf_player_t p;
    uint32_t now = 1000;                       /* any start time works, the player only uses differences */
    bf_status_t st = bf_start(&p, t, stand, now);
    if (st != BF_OK) {
        fprintf(stderr, "refused: servo %d frame %d is %d ticks past its end\n", p.rail_servo, p.rail_frame, p.rail_over);
        return 1;
    }
    uint16_t out[BF_SERVOS];
    for (;;) {
        now += 5;                              /* the servo loop: every 5 ms */
        if (!bf_tick(&p, now, out)) break;
        for (int i = 0; i < BF_SERVOS; i++) printf(i ? ",%u" : "%u", out[i]);
        printf("\n");
    }
    return 0;
}
