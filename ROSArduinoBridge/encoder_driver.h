/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef PR_H1_01
  #define M1_ENCDRA 12
  #define M1_ENCDRB 13
  #define M2_ENCDRA 6
  #define M2_ENCDRB 21
  #define M3_ENCDRA 1
  #define M3_ENCDRB 0
#endif

void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

