#include "ble_experiment.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

/* Runs the actual firmware controller compiled for Cortex-M4 in an emulator. */
int rf_controller_test(void) {
    RFExperiment s;
    RF_Init(&s,0);
    RF_Step(&s,59999,1); CHECK(s.phase==RF_PREPARE);
    RF_Step(&s,60000,1); CHECK(s.phase==RF_WAIT_LINK);
    RF_Step(&s,60300,1); CHECK(s.phase==RF_NORMAL);
    uint32_t now=60300;
    for (int round=1;round<=3;round++) {
        CHECK(s.cycle==round);
        RF_Step(&s,now+29999,1); CHECK(s.phase==RF_NORMAL);
        CHECK(RF_Step(&s,now+30000,1)&RF_EVENT(RF_RESET_ASSERT));
        CHECK(s.phase==RF_RESET_HELD);
        RF_Step(&s,now+30001,0);
        RF_Step(&s,now+59999,0); CHECK(s.phase==RF_RESET_HELD);
        CHECK(RF_Step(&s,now+60000,0)&RF_EVENT(RF_RESET_RELEASE));
        CHECK(s.phase==RF_WAIT_LINK);
        RF_Step(&s,now+90000,0); CHECK(s.phase==RF_WAIT_LINK);
        CHECK(RF_Step(&s,now+100000,1)&RF_EVENT(RF_LINK_HIGH));
        RF_Step(&s,now+100299,1); CHECK(s.phase==RF_WAIT_LINK);
        uint32_t e=RF_Step(&s,now+100300,1);
        CHECK(e&RF_EVENT(round==3?RF_COMPLETE:RF_NORMAL_START));
        now+=100300;
    }
    CHECK(s.phase==RF_DONE && s.completed==3);
    RF_Step(&s,now+1000000,1); CHECK(s.phase==RF_DONE);

    /* Interrupted normal segment restarts for a full 30s after reconnect. */
    RF_Init(&s,0); RF_Step(&s,1,1); RF_Step(&s,60000,1); RF_Step(&s,60001,1);
    CHECK(RF_Step(&s,70000,0)&RF_EVENT(RF_NORMAL_ABORT));
    CHECK(s.phase==RF_WAIT_LINK && s.completed==0);
    RF_Step(&s,71000,1); RF_Step(&s,71300,1); CHECK(s.phase==RF_NORMAL);
    RF_Step(&s,101299,1); CHECK(s.phase==RF_NORMAL);
    RF_Step(&s,101300,1); CHECK(s.phase==RF_RESET_HELD);
    /* Stuck high cannot masquerade as reconnection after reset. */
    CHECK(RF_Step(&s,102300,1)&RF_EVENT(RF_RESET_WARNING));
    RF_Step(&s,131300,1); RF_Step(&s,132000,1); CHECK(s.phase==RF_WAIT_LINK);
    CHECK(RF_Step(&s,251300,1)&RF_EVENT(RF_WAIT_WARNING));
    CHECK(s.phase==RF_WAIT_LINK);

    /* HAL tick rollover. */
    uint32_t base=0xfffff000u;
    RF_Init(&s,base); RF_Step(&s,base+1,1);
    RF_Step(&s,base+60000u,1); RF_Step(&s,base+60001u,1);
    CHECK(s.phase==RF_NORMAL);
    CHECK(RF_Step(&s,base+90001u,1)&RF_EVENT(RF_RESET_ASSERT));
    return 0;
}
