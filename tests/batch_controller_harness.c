#include "raw_batch.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
static RawBatch b;
int rf_controller_test(void) {
 uint8_t f[35];
 for (unsigned n=0;n<20;n++) {
   for(unsigned j=0;j<35;j++) f[j]=(uint8_t)(n*35+j);
   int result=RawBatch_Push(&b,f,n<10 || n>=15);
   CHECK(result == (n%5!=4 ? 0 : n==14 ? -1 : 1));
   if(n>=4 && n<9) for(unsigned j=0;j<175;j++) CHECK(b.tx[j]==(uint8_t)j);
   if(n>=9 && n<19) for(unsigned j=0;j<175;j++) CHECK(b.tx[j]==(uint8_t)(175+j));
 }
 for(unsigned j=0;j<175;j++) CHECK(b.tx[j]==(uint8_t)(525+j));
 return 0;
}
