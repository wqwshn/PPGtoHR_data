#include "ble_power_experiment.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int rf_controller_test(void) {
    BPExperiment s;
    BP_Init(&s,0);
    BP_Step(&s,59999,1,0); CHECK(s.phase==BP_PREPARE);
    BP_Step(&s,60000,1,0); CHECK(s.phase==BP_WAIT_LINK);
    BP_Step(&s,60300,1,0); CHECK(s.phase==BP_SET && s.stage==1);
    uint32_t now=60300;
    for (int stage=1;stage<=7;stage++) {
        CHECK(s.stage==stage);
        BP_Step(&s,now+299,1,0); CHECK(s.phase==BP_SET);
        CHECK(BP_Step(&s,now+300,1,0)&BP_EVENT(BP_QUERY_SENT));
        CHECK(s.phase==BP_QUERY);
        CHECK(BP_Step(&s,now+320,1,1)&BP_EVENT(BP_VERIFIED));
        CHECK(s.phase==BP_SETTLE);
        BP_Step(&s,now+10319,1,0); CHECK(s.phase==BP_SETTLE);
        CHECK(BP_Step(&s,now+10320,1,0)&BP_EVENT(BP_START));
        CHECK(s.phase==BP_MEASURE);
        /* A wireless outage must remain inside the measurement interval. */
        CHECK(BP_Step(&s,now+12000,0,0)&BP_EVENT(BP_LINK_LOW));
        CHECK(s.phase_start==now+10320);
        BP_Step(&s,now+70319,0,0); CHECK(s.phase==BP_MEASURE);
        uint32_t events=BP_Step(&s,now+70320,0,0);
        CHECK(events&BP_EVENT(BP_END));
        CHECK(s.phase==(stage==7?BP_DONE:BP_SET));
        now+=70320;
    }
    CHECK(s.stage==7 && !s.failed);
    CHECK(bp_powers[0]==25 && bp_powers[1]==0 && bp_powers[3]==-50 && bp_powers[5]==-100);
    BP_Step(&s,now+100000,1,0); CHECK(s.phase==BP_DONE);
    /* No connection at startup -> no power changes. */
    BP_Init(&s,0); BP_Step(&s,60000,0,0);
    CHECK(BP_Step(&s,180000,0,0)&BP_EVENT(BP_WAIT_WARNING));
    CHECK(s.phase==BP_WAIT_LINK);
    BP_Step(&s,180001,1,0); BP_Step(&s,180300,1,0); CHECK(s.phase==BP_WAIT_LINK);
    BP_Step(&s,180301,1,0); CHECK(s.phase==BP_SET);
    /* Wrong readback -> abort the sweep, attempt baseline exactly once. */
    s.phase=BP_QUERY; s.stage=4; s.phase_start=0;
    uint32_t events=BP_Step(&s,1,1,-1);
    CHECK(events&BP_EVENT(BP_FAILED)); CHECK(s.stage==7 && s.phase==BP_SET && s.failed);
    BP_Step(&s,301,1,0); CHECK(s.phase==BP_QUERY);
    CHECK(BP_Step(&s,302,1,1)&BP_EVENT(BP_RECOVERED));
    CHECK(s.phase==BP_ERROR); /* Recovery does not make the experiment successful. */
    /* Lost query reply, including failed restoration, terminates safely. */
    BP_Init(&s,0); s.phase=BP_QUERY; s.stage=2;
    BP_Step(&s,1999,1,0); CHECK(s.phase==BP_QUERY);
    BP_Step(&s,2000,1,0); CHECK(s.failed && s.stage==7);
    BP_Step(&s,2300,1,0); BP_Step(&s,4300,1,0); CHECK(s.phase==BP_ERROR);
    BP_Step(&s,999999,1,1); CHECK(s.phase==BP_ERROR);
    /* uint32 tick wrap. */
    BP_Init(&s,0xfffffff0u); s.phase=BP_MEASURE;
    BP_Step(&s,(uint32_t)(0xfffffff0u+59999u),1,0); CHECK(s.stage==1);
    BP_Step(&s,(uint32_t)(0xfffffff0u+60000u),1,0); CHECK(s.stage==2);
    CHECK(BP_Readback("<rd_tx_power=+2.5>",25)==1);
    CHECK(BP_Readback("<rd_tx_power=2.5>",25)==1);
    CHECK(BP_Readback("<rd_tx_power=-10.0>",-100)==1);
    CHECK(BP_Readback("<rd_tx_power=0>",0)==1);
    CHECK(BP_Readback("<rd_tx_power=-5>",25)==-1);
    CHECK(BP_Readback("<st_tx_power=ok>",25)==0);
    CHECK(BP_Readback("<rd_tx_power=25x>",25)==-1);
    CHECK(BP_Readback("<rd_tx_power=>",25)==-1);
    CHECK(BP_Readback("<rd_tx_power=2.50>",25)==-1);
    CHECK(BP_Readback("<rd_tx_power=-13.5>",32767)==1);
    CHECK(BP_Readback("<rd_tx_power=+4>",32767)==-1);
    /* Preflight queries without waiting for BLE connection and never writes. */
    BP_Init(&s,0); s.phase=BP_READ_PREPARE;
    BP_ReadOnlyStep(&s,9999,0,0); CHECK(s.phase==BP_READ_PREPARE);
    events=BP_ReadOnlyStep(&s,10000,0,0);
    CHECK(events&BP_EVENT(BP_QUERY_SENT)); CHECK(!(events&BP_EVENT(BP_SET_SENT)));
    BP_ReadOnlyStep(&s,11999,0,0); CHECK(s.phase==BP_READ_QUERY);
    events=BP_ReadOnlyStep(&s,12000,0,0);
    CHECK(events&BP_EVENT(BP_FAILED)); CHECK(s.phase==BP_READ_ERROR);
    CHECK(!(events&BP_EVENT(BP_SET_SENT)));
    BP_ReadOnlyStep(&s,999999,1,1); CHECK(s.phase==BP_READ_ERROR);
    BP_Init(&s,0); s.phase=BP_READ_PREPARE;
    BP_ReadOnlyStep(&s,10000,1,0);
    CHECK(BP_ReadOnlyStep(&s,10001,1,1)&BP_EVENT(BP_READ_OK));
    CHECK(s.phase==BP_READ_DONE);
    BP_Init(&s,0);
    BP_WriteStep(&s,60000,1,0); BP_WriteStep(&s,60300,1,0);
    CHECK(s.phase==BP_WRITE_WAIT);
    now=60300;
    for (int stage=1;stage<=7;stage++) {
        CHECK(s.stage==stage);
        BP_WriteStep(&s,now+20,1,0); CHECK(s.phase==BP_WRITE_WAIT);
        events=BP_WriteStep(&s,now+30,1,1);
        CHECK(events&BP_EVENT(BP_WRITE_SENT_DONE)); CHECK(s.phase==BP_WRITE_SETTLE);
        CHECK(!(events&(BP_EVENT(BP_VERIFIED)|BP_EVENT(BP_QUERY_SENT))));
        BP_WriteStep(&s,now+10029,1,1); CHECK(s.phase==BP_WRITE_SETTLE);
        BP_WriteStep(&s,now+10030,1,1); CHECK(s.phase==BP_WRITE_MEASURE);
        BP_WriteStep(&s,now+11000,0,1); CHECK(s.phase_start==now+10030);
        BP_WriteStep(&s,now+70029,0,1); CHECK(s.phase==BP_WRITE_MEASURE);
        events=BP_WriteStep(&s,now+70030,0,1);
        CHECK(events&BP_EVENT(BP_END));
        CHECK(!(events&(BP_EVENT(BP_VERIFIED)|BP_EVENT(BP_QUERY_SENT))));
        CHECK(s.phase==(stage==7?BP_WRITE_DONE:BP_WRITE_WAIT));
        now+=70030;
    }
    CHECK(events&BP_EVENT(BP_WRITE_COMPLETE));
    BP_Init(&s,0); s.phase=BP_WRITE_WAIT;
    BP_WriteStep(&s,1999,1,0); CHECK(s.phase==BP_WRITE_WAIT);
    CHECK(BP_WriteStep(&s,2000,1,0)&BP_EVENT(BP_FAILED)); CHECK(s.phase==BP_WRITE_ERROR);
    BP_WriteStep(&s,2001,1,1); CHECK(s.phase==BP_WRITE_ERROR);
    return 0;
}
