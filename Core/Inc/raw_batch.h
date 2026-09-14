#ifndef RAW_BATCH_H
#define RAW_BATCH_H
#include <stdint.h>
/* Main-loop owned staging and immutable in-flight DMA storage. */
typedef struct { uint8_t pending[175], tx[175], count; } RawBatch;
/* Returns 0 while collecting, 1 when ready, -1 if this batch must be dropped.
 * Never touch tx while UART DMA is busy. Caller serializes UART state/start. */
static inline int RawBatch_Push(RawBatch *b, const uint8_t *frame, int idle)
{
    for (unsigned i=0; i<35; ++i) b->pending[b->count*35+i]=frame[i];
    if (++b->count < 5) return 0;
    b->count=0;
    if (!idle) return -1;
    for (unsigned i=0; i<175; ++i) b->tx[i]=b->pending[i];
    return 1;
}
#endif
