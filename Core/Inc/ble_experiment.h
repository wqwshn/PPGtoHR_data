#ifndef BLE_EXPERIMENT_H
#define BLE_EXPERIMENT_H
#include <stdint.h>

enum { RF_PREPARE, RF_WAIT_LINK, RF_NORMAL, RF_RESET_HELD, RF_DONE };
enum { RF_SNAPSHOT, RF_BOOT, RF_LINK_HIGH, RF_LINK_LOW, RF_NORMAL_START,
       RF_NORMAL_ABORT, RF_RESET_ASSERT, RF_RESET_RELEASE, RF_COMPLETE,
       RF_WAIT_WARNING, RF_RESET_WARNING };
#define RF_EVENT(code) (1u << (code))
typedef struct {
    uint32_t phase_start, link_since;
    uint8_t phase, cycle, completed, link, saw_low, need_low, warned;
} RFExperiment;

/* Pure controller: no delays, no UART, all elapsed-time tests tolerate tick wrap. */
static inline void RF_Init(RFExperiment *s, uint32_t now) {
    *s = (RFExperiment){ .phase_start=now, .link_since=now,
                        .phase=RF_PREPARE, .cycle=1 };
}
static inline uint32_t RF_Step(RFExperiment *s, uint32_t now, uint8_t link) {
    uint32_t events=0;
    if (link != s->link) {
        s->link=link; s->link_since=now;
        events |= RF_EVENT(link ? RF_LINK_HIGH : RF_LINK_LOW);
    }
    if (!link) s->saw_low=1;
    switch (s->phase) {
    case RF_PREPARE:
        if ((uint32_t)(now-s->phase_start)>=60000u) {
            s->phase=RF_WAIT_LINK; s->phase_start=now; s->warned=0;
        }
        break;
    case RF_WAIT_LINK:
        if (link && (!s->need_low || s->saw_low) &&
            (uint32_t)(now-s->link_since)>=300u) {
            s->phase_start=now;
            if (s->completed==3) {
                s->phase=RF_DONE; events |= RF_EVENT(RF_COMPLETE);
            } else {
                s->cycle=s->completed+1; s->phase=RF_NORMAL;
                events |= RF_EVENT(RF_NORMAL_START);
            }
        } else if (!s->warned && (uint32_t)(now-s->phase_start)>=120000u) {
            s->warned=1; events |= RF_EVENT(RF_WAIT_WARNING);
        }
        break;
    case RF_NORMAL:
        if (!link) {
            s->phase=RF_WAIT_LINK; s->phase_start=now; s->warned=0;
            events |= RF_EVENT(RF_NORMAL_ABORT);
        } else if ((uint32_t)(now-s->phase_start)>=30000u) {
            s->phase=RF_RESET_HELD; s->phase_start=now;
            s->need_low=1; s->saw_low=0; s->warned=0;
            events |= RF_EVENT(RF_RESET_ASSERT);
        }
        break;
    case RF_RESET_HELD:
        if (!s->saw_low && !s->warned && (uint32_t)(now-s->phase_start)>=1000u) {
            s->warned=1; events |= RF_EVENT(RF_RESET_WARNING);
        }
        if ((uint32_t)(now-s->phase_start)>=30000u) {
            s->completed++; s->phase=RF_WAIT_LINK;
            s->phase_start=now; s->warned=0;
            /* Require HIGH stability after release, not time spent in reset. */
            s->link_since=now;
            events |= RF_EVENT(RF_RESET_RELEASE);
        }
        break;
    default: break;
    }
    return events;
}
#endif
