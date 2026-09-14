/* Included in main.c after diagnostic globals. Single main-loop producer,
 * UART IRQ consumer; no additional UART transmission between raw and status. */
#if BLE_RF_EXPERIMENT
#include "ble_experiment.h"
#define RF_PACKET_LEN 27
#define RF_QUEUE_SIZE 32
static RFExperiment rf_state;
static uint8_t rf_queue[RF_QUEUE_SIZE][RF_PACKET_LEN], rf_tx[RF_PACKET_LEN];
static volatile uint8_t rf_head, rf_tail;
static uint32_t rf_event_id, rf_heartbeat;
static uint16_t rf_dropped;
static void RF_U32(uint8_t *p, uint32_t n) {
    p[0]=n>>24; p[1]=n>>16; p[2]=n>>8; p[3]=n;
}
static void RF_Enqueue(uint8_t code, uint32_t now) {
    uint8_t next=(rf_head+1)%RF_QUEUE_SIZE;
    ++rf_event_id;
    if (next==rf_tail) { if (rf_dropped!=65535) ++rf_dropped; return; }
    uint8_t *p=rf_queue[rf_head];
    p[0]=0xAA; p[1]=0xEE; p[2]=code; p[3]=rf_state.cycle;
    p[4]=rf_state.phase; p[5]=rf_state.link;
    p[6]=(rf_state.phase==RF_RESET_HELD);
    RF_U32(p+7,now); RF_U32(p+11,raw_diag_sample_counter);
    RF_U32(p+15,rf_event_id); RF_U32(p+19,rf_state.phase_start);
    p[23]=rf_dropped>>8; p[24]=rf_dropped;
    p[25]=0; for (unsigned i=2;i<25;i++) p[25]^=p[i]; p[26]=0x55;
    __DMB(); rf_head=next;
}
static void RF_Start(void) {
    uint32_t now=HAL_GetTick(); RF_Init(&rf_state,now); rf_heartbeat=now;
    RF_Enqueue(RF_BOOT,now);
}
static void RF_Poll(void) {
    uint32_t now=HAL_GetTick();
    uint32_t events=RF_Step(&rf_state,now,
        HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port,BLE_STATE_Pin)==GPIO_PIN_SET);
    if (events & RF_EVENT(RF_RESET_ASSERT))
        HAL_GPIO_WritePin(BLE_RST_GPIO_Port,BLE_RST_Pin,GPIO_PIN_SET);
    if (events & RF_EVENT(RF_RESET_RELEASE))
        HAL_GPIO_WritePin(BLE_RST_GPIO_Port,BLE_RST_Pin,GPIO_PIN_RESET);
    for (uint8_t code=1;code<=RF_RESET_WARNING;code++)
        if (events & RF_EVENT(code)) RF_Enqueue(code,now);
    if ((uint32_t)(now-rf_heartbeat)>=1000u) {
        rf_heartbeat=now; RF_Enqueue(RF_SNAPSHOT,now);
    }
}
static void RF_TryTransmit(void) {
    if (rf_tail==rf_head) return;
    __DMB(); memcpy(rf_tx,rf_queue[rf_tail],RF_PACKET_LEN);
    if (HAL_UART_Transmit_DMA(&huart2,rf_tx,RF_PACKET_LEN)==HAL_OK) {
        raw_diag_dma_tx_kind=3;
        rf_tail=(rf_tail+1)%RF_QUEUE_SIZE;
    }
}
#endif
