/* Pure controller: only TX power changes; ADC and connection settings stay fixed. */
#ifndef BLE_POWER_EXPERIMENT_H
#define BLE_POWER_EXPERIMENT_H
#include <stdint.h>
#define BP_EVENT(n) (1UL << (n))
enum { BP_PREPARE=5, BP_SET, BP_QUERY, BP_SETTLE, BP_MEASURE,
       BP_DONE, BP_ERROR, BP_WAIT_LINK, BP_READ_DONE, BP_READ_ERROR,
       BP_READ_PREPARE, BP_READ_QUERY, BP_WRITE_WAIT, BP_WRITE_SETTLE,
       BP_WRITE_MEASURE, BP_WRITE_DONE, BP_WRITE_ERROR };
enum { BP_BOOT=11, BP_SET_SENT, BP_QUERY_SENT, BP_VERIFIED, BP_START,
       BP_END, BP_FAILED, BP_COMPLETE, BP_RECOVERED, BP_LINK_HIGH,
       BP_LINK_LOW, BP_WAIT_WARNING, BP_DIAGNOSTIC, BP_READ_OK,
       BP_WRITE_SENT_DONE, BP_WRITE_COMPLETE };
static const int16_t bp_powers[7]={25,0,25,-50,25,-100,25};
static const char *const bp_commands[7]={
    "<ST_TX_POWER=+2.5>", "<ST_TX_POWER=0>", "<ST_TX_POWER=+2.5>",
    "<ST_TX_POWER=-5>", "<ST_TX_POWER=+2.5>", "<ST_TX_POWER=-10>",
    "<ST_TX_POWER=+2.5>"};
typedef struct {
    uint32_t phase_start, link_since;
    uint8_t phase, stage, link, failed, warned;
} BPExperiment;
static void BP_Init(BPExperiment *s, uint32_t now) {
    s->phase=BP_PREPARE; s->stage=1; s->phase_start=now;
    s->link_since=now; s->link=0; s->failed=0; s->warned=0;
}
static uint32_t BP_Enter(BPExperiment *s, uint32_t now, uint8_t phase, uint8_t event) {
    s->phase=phase; s->phase_start=now; return BP_EVENT(event);
}
/* Read-only preflight never emits BP_SET_SENT, even after failure. */
static uint32_t BP_ReadOnlyStep(BPExperiment *s,uint32_t now,uint8_t link,int reply) {
    uint32_t events=0;
    if (link!=s->link) {
        s->link=link; events|=BP_EVENT(link?BP_LINK_HIGH:BP_LINK_LOW);
    }
    if (s->phase==BP_READ_PREPARE && (uint32_t)(now-s->phase_start)>=10000u)
        events|=BP_Enter(s,now,BP_READ_QUERY,BP_QUERY_SENT);
    else if (s->phase==BP_READ_QUERY) {
        if (reply==1) events|=BP_Enter(s,now,BP_READ_DONE,BP_READ_OK);
        else if (reply<0 || (uint32_t)(now-s->phase_start)>=2000u)
            events|=BP_Enter(s,now,BP_READ_ERROR,BP_FAILED);
    }
    return events;
}
/* reply: 0=no response, 1=exact requested readback, -1=invalid/error. */
static uint32_t BP_Step(BPExperiment *s, uint32_t now, uint8_t link, int reply) {
    uint32_t events=0, elapsed=(uint32_t)(now-s->phase_start);
    if (link!=s->link) {
        s->link=link; s->link_since=now;
        events|=BP_EVENT(link?BP_LINK_HIGH:BP_LINK_LOW);
    }
    switch (s->phase) {
    case BP_PREPARE:
        if (elapsed>=60000u) { s->phase=BP_WAIT_LINK; s->phase_start=now; }
        break;
    case BP_WAIT_LINK:
        if (link && (uint32_t)(now-s->link_since)>=300u)
            events|=BP_Enter(s,now,BP_SET,BP_SET_SENT);
        else if (elapsed>=120000u && !s->warned) {
            s->warned=1; events|=BP_EVENT(BP_WAIT_WARNING);
        }
        break;
    case BP_SET:
        if (elapsed>=300u) events|=BP_Enter(s,now,BP_QUERY,BP_QUERY_SENT);
        break;
    case BP_QUERY:
        if (reply==1) {
            events|=BP_EVENT(BP_VERIFIED);
            if (s->failed) events|=BP_Enter(s,now,BP_ERROR,BP_RECOVERED);
            else { s->phase=BP_SETTLE; s->phase_start=now; }
        } else if (reply<0 || elapsed>=2000u) {
            events|=BP_EVENT(BP_FAILED);
            if (s->failed) { s->phase=BP_ERROR; s->phase_start=now; }
            else {
                s->failed=1; s->stage=7;
                events|=BP_Enter(s,now,BP_SET,BP_SET_SENT);
            }
        }
        break;
    case BP_SETTLE:
        if (elapsed>=10000u) events|=BP_Enter(s,now,BP_MEASURE,BP_START);
        break;
    case BP_MEASURE:
        /* Deliberately do not restart after link loss: outages are an outcome. */
        if (elapsed>=60000u) {
            events|=BP_EVENT(BP_END);
            if (s->stage==7) events|=BP_Enter(s,now,BP_DONE,BP_COMPLETE);
            else { ++s->stage; events|=BP_Enter(s,now,BP_SET,BP_SET_SENT); }
        }
        break;
    default: break;
    }
    return events;
}
/* Write-only experiment: advance only after UART completion, never assert readback. */
static uint32_t BP_WriteStep(BPExperiment *s,uint32_t now,uint8_t link,int tx_done) {
    if (s->phase==BP_PREPARE || s->phase==BP_WAIT_LINK) {
        uint32_t events=BP_Step(s,now,link,0);
        if (s->phase==BP_SET) s->phase=BP_WRITE_WAIT;
        return events;
    }
    uint32_t events=0, elapsed=(uint32_t)(now-s->phase_start);
    if (link!=s->link) {
        s->link=link; events|=BP_EVENT(link?BP_LINK_HIGH:BP_LINK_LOW);
    }
    switch(s->phase) {
    case BP_WRITE_WAIT:
        if (tx_done) events|=BP_Enter(s,now,BP_WRITE_SETTLE,BP_WRITE_SENT_DONE);
        else if (elapsed>=2000u) events|=BP_Enter(s,now,BP_WRITE_ERROR,BP_FAILED);
        break;
    case BP_WRITE_SETTLE:
        if (elapsed>=10000u) events|=BP_Enter(s,now,BP_WRITE_MEASURE,BP_START);
        break;
    case BP_WRITE_MEASURE:
        if (elapsed>=60000u) {
            events|=BP_EVENT(BP_END);
            if (s->stage==7) events|=BP_Enter(s,now,BP_WRITE_DONE,BP_WRITE_COMPLETE);
            else { ++s->stage; events|=BP_Enter(s,now,BP_WRITE_WAIT,BP_SET_SENT); }
        }
        break;
    default: break;
    }
    return events;
}
/* Strict decimal readback, tolerates an omitted '+' and trailing .0 only. */
static int BP_Readback(const char *p, int16_t expected) {
    const char *prefix="<rd_tx_power=";
    while (*prefix) { if (*p++!=*prefix++) return 0; }
    int sign=1, n=0, digits=0;
    if (*p=='-' || *p=='+') { if (*p=='-') sign=-1; ++p; }
    while (*p>='0' && *p<='9') {
        if (++digits>3) return -1;
        n=n*10+(*p++-'0');
    }
    if (!digits) return -1;
    n*=10;
    if (*p=='.') { ++p; if (*p<'0'||*p>'9') return -1; n+=*p++-'0'; }
    if (*p++!='>' || *p!='\0') return -1;
    if (expected==32767) {
        static const int16_t supported[]={-195,-135,-100,-70,-50,-35,-20,-10,0,10,15,25};
        for (unsigned i=0;i<sizeof(supported)/sizeof(supported[0]);i++)
            if (sign*n==supported[i]) return 1;
        return -1;
    }
    return sign*n==expected ? 1 : -1;
}
#endif
