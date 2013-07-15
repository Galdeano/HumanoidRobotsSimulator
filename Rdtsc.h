#ifndef H_Rdtsc
#define H_Rdtsc



#ifdef __i386
extern __inline__ uint64_t rdtsc() {
  uint64_t x;
  __asm__ volatile ("rdtsc" : "=A" (x));
  return x;
}
#elif defined __amd64
extern __inline__ uint64_t rdtsc() {
  uint64_t a, d;
  __asm__ volatile ("rdtsc" : "=a" (a), "=d" (d));
  return (d<<32) | a;
}
#endif


//        static uint64_t time,time2;
//        time=rdtsc();
//        printf("fps: %d %f \n",(double)(3200000000/(double)(time-time2)));
//        time2=time;

#endif
