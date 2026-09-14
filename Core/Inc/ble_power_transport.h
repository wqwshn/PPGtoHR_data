/* Included only in main.c. Commands/events use a spare post-Raw DMA slot. */
#if BLE_POWER_EXPERIMENT
#include "ble_power_experiment.h"
#define BP_QUEUE_SIZE 32
static BPExperiment bp_state;
static uint8_t bp_queue[BP_QUEUE_SIZE][53], bp_lengths[BP_QUEUE_SIZE], bp_tx[53];
static volatile uint8_t bp_head, bp_tail;
static uint32_t bp_event_id, bp_heartbeat;
static uint16_t bp_dropped;
static volatile uint16_t bp_rx_head, bp_rx_tail;
static volatile uint8_t bp_rx_error;
static uint8_t bp_rx[128];
static char bp_response[48];
static uint8_t bp_response_len;
static volatile uint32_t bp_rx_count;
static volatile uint16_t bp_rx_errors, bp_command_done;
static volatile uint8_t bp_tx_is_command;
static uint8_t bp_rx_recent[22], bp_rx_recent_len, bp_rx_recent_head;
static uint8_t bp_diag_reason;
static uint16_t bp_command_baseline;
static void BP_U32(uint8_t *p,uint32_t n) {
    p[0]=n>>24; p[1]=n>>16; p[2]=n>>8; p[3]=n;
}
static void BP_Queue(const uint8_t *p,uint8_t len) {
    uint8_t next=(bp_head+1)%BP_QUEUE_SIZE;
    if (next==bp_tail) { if (bp_dropped!=65535) ++bp_dropped; return; }
    memcpy(bp_queue[bp_head],p,len); bp_lengths[bp_head]=len;
    __DMB(); bp_head=next;
}
static void BP_Emit(uint8_t code,uint32_t now,const BPExperiment *s) {
    uint8_t p[27]={0xAA,0xEE,code,s->stage,s->phase,s->link,0};
    BP_U32(p+7,now); BP_U32(p+11,raw_diag_sample_counter);
    BP_U32(p+15,++bp_event_id); BP_U32(p+19,s->phase_start);
    p[23]=bp_dropped>>8; p[24]=bp_dropped;
    for (unsigned i=2;i<25;i++) { p[25]^=p[i]; }
    p[26]=0x55;
    BP_Queue(p,sizeof(p));
}
/* AA EF v1, 53 bytes: lossless hex of the final 22 RX bytes plus counters. */
static void BP_Diagnostic(uint32_t now,const BPExperiment *s,uint8_t reason) {
    uint8_t p[53]={0xAA,0xEF,1,s->stage,s->phase,s->link,reason};
    BP_U32(p+7,now); BP_U32(p+11,raw_diag_sample_counter);
    BP_U32(p+15,++bp_event_id); BP_U32(p+19,bp_rx_count);
    p[23]=bp_rx_errors>>8; p[24]=bp_rx_errors;
    p[25]=bp_command_done>>8; p[26]=bp_command_done;
    p[27]=bp_rx_recent_len;
    for (uint8_t i=0;i<bp_rx_recent_len;i++)
        p[28+i]=bp_rx_recent[(bp_rx_recent_head+22-bp_rx_recent_len+i)%22];
    p[50]=BLE_POWER_READ_ONLY;
    for (unsigned i=2;i<51;i++) { p[51]^=p[i]; }
    p[52]=0x55; BP_Queue(p,sizeof(p));
}
static void BP_TxDone(void) {
    if (bp_tx_is_command) {
        if (bp_command_done!=65535) ++bp_command_done;
        bp_tx_is_command=0;
    }
}
/* Called before HAL's USART handler; does not stop or abort TX DMA. */
void BP_RxIRQ(void) {
    uint32_t flags=huart2.Instance->ISR;
    if (flags & (USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE|USART_ISR_PE)) {
        bp_rx_error=1;
        if (bp_rx_errors!=65535) ++bp_rx_errors;
        huart2.Instance->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NCF|USART_ICR_PECF;
    }
    if (flags & USART_ISR_RXNE) {
        uint8_t b=(uint8_t)huart2.Instance->RDR;
        ++bp_rx_count;
        uint16_t next=(bp_rx_head+1)%128;
        if (next==bp_rx_tail) bp_rx_error=1;
        else { bp_rx[bp_rx_head]=b; __DMB(); bp_rx_head=next; }
    }
}
static void RF_Start(void) {
    uint32_t now=HAL_GetTick(); BP_Init(&bp_state,now); bp_heartbeat=now;
#if BLE_POWER_READ_ONLY
    bp_state.phase=BP_READ_PREPARE;
#endif
    __HAL_UART_CLEAR_OREFLAG(&huart2); __HAL_UART_FLUSH_DRREGISTER(&huart2);
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
    BP_Emit(BP_BOOT,now,&bp_state);
}
static void RF_Poll(void) {
    uint32_t now=HAL_GetTick(); int reply=0;
    while (bp_rx_tail!=bp_rx_head) {
        __DMB(); char b=(char)bp_rx[bp_rx_tail]; bp_rx_tail=(bp_rx_tail+1)%128;
        bp_rx_recent[bp_rx_recent_head]=(uint8_t)b; bp_rx_recent_head=(bp_rx_recent_head+1)%22;
        if (bp_rx_recent_len<22) ++bp_rx_recent_len;
        if (b=='<') bp_response_len=0;
        if (bp_response_len || b=='<') {
            if (bp_response_len>=sizeof(bp_response)-1) { bp_response_len=0; continue; }
            bp_response[bp_response_len++]=b;
            if (b=='>') {
                bp_response[bp_response_len]='\0';
                int r=BP_Readback(bp_response,BLE_POWER_READ_ONLY?32767:bp_powers[bp_state.stage-1]);
                if ((bp_state.phase==BP_QUERY || bp_state.phase==BP_READ_QUERY) && r) reply=r;
                bp_response_len=0;
            }
        }
    }
    if (bp_rx_error && (bp_state.phase==BP_QUERY || bp_state.phase==BP_READ_QUERY)) reply=-1;
    BPExperiment before=bp_state;
#if BLE_POWER_WRITE_ONLY
    uint32_t events=BP_WriteStep(&bp_state,now,
        HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port,BLE_STATE_Pin)==GPIO_PIN_SET,
        bp_command_done!=bp_command_baseline);
#elif BLE_POWER_READ_ONLY
    uint32_t events=BP_ReadOnlyStep(&bp_state,now,
#else
    uint32_t events=BP_Step(&bp_state,now,
#endif
#if !BLE_POWER_WRITE_ONLY
        HAL_GPIO_ReadPin(BLE_STATE_GPIO_Port,BLE_STATE_Pin)==GPIO_PIN_SET,reply);
#endif
    /* END and FAILED belong to the previous stage, not the new target. */
    if (events & BP_EVENT(BP_END)) BP_Emit(BP_END,now,&before);
    if (events & BP_EVENT(BP_FAILED)) BP_Emit(BP_FAILED,now,&before);
    if (!BLE_POWER_WRITE_ONLY && (events & (BP_EVENT(BP_FAILED)|BP_EVENT(BP_VERIFIED)|BP_EVENT(BP_READ_OK)))) {
        bp_diag_reason=(events & BP_EVENT(BP_FAILED))?(bp_rx_error?3:reply<0?2:1):0;
        BP_Diagnostic(now,&before,bp_diag_reason);
    }
    for (uint8_t code=BP_BOOT;code<=BP_WRITE_COMPLETE;code++) {
        if (!(events & BP_EVENT(code))) continue;
        if (code==BP_END || code==BP_FAILED) continue;
        if (code==BP_SET_SENT || code==BP_QUERY_SENT) {
            bp_command_baseline=bp_command_done;
            bp_rx_error=0; bp_response_len=0;
            const char *cmd=code==BP_SET_SENT?bp_commands[bp_state.stage-1]:"<RD_TX_POWER>";
            BP_Queue((const uint8_t *)cmd,(uint8_t)strlen(cmd));
        }
        BP_Emit(code,now,&bp_state);
    }
    if ((uint32_t)(now-bp_heartbeat)>=1000u) {
        bp_heartbeat=now; BP_Emit(0,now,&bp_state);
        if (bp_state.phase==BP_ERROR || bp_state.phase==BP_READ_ERROR || bp_state.phase==BP_READ_DONE)
            BP_Diagnostic(now,&bp_state,bp_diag_reason);
    }
}
static void RF_TryTransmit(void) {
    if (bp_tail==bp_head) return;
    __DMB(); uint8_t len=bp_lengths[bp_tail]; memcpy(bp_tx,bp_queue[bp_tail],len);
    if (HAL_UART_Transmit_DMA(&huart2,bp_tx,len)==HAL_OK) {
        bp_tx_is_command=(bp_tx[0]=='<');
        raw_diag_dma_tx_kind=3; bp_tail=(bp_tail+1)%BP_QUEUE_SIZE;
    }
}
#endif
