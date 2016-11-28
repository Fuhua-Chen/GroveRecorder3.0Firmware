#line 1 "..\\..\\..\\..\\Library\\Framework\\src\\AudioMixer.c"
#line 1 "..\\..\\..\\..\\Library\\Framework\\inc\\AudioMixer.h"



#line 1 "..\\ConfigApp.h"
 
 
 
 
 


									


#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\Platform.h"




 




#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
 









 




















 











 








 




 



 

typedef enum IRQn
    {
         
        NonMaskableInt_IRQn       = -14,     
        HardFault_IRQn		      = -13,     
        SVCall_IRQn               = -5,      
        PendSV_IRQn               = -2,      
        SysTick_IRQn              = -1,      
         
        BOD_IRQn                  = 0,       
        WDT_IRQn                  = 1,       
        EINT0_IRQn                = 2,       
        EINT1_IRQn                = 3,       
        GPAB_IRQn                 = 4,       
        ALC_IRQn                  = 5,       
        PWM0_IRQn                 = 6,       
        IRQ7n                     = 7,
        TMR0_IRQn                 = 8,       
        TMR1_IRQn                 = 9,       
        IRQ10n                    = 10,
        IRQ11n                    = 11,
        UART0_IRQn                = 12,      
        IRQ13n                    = 13,
        SPI0_IRQn                 = 14,      
        IRQ15n                    = 15,
        IRQ16n                    = 16,
        IRQ17n                    = 17,
        I2C0_IRQn                 = 18,      
        IRQ19n                    = 19,
        IRQ20n                    = 20,
        TALARM_IRQn               = 21,      
        IRQ22n                    = 22,
        IRQ23n                    = 23,
        IRQ24n                    = 24,
        ACMP_IRQn                 = 25,      
        PDMA_IRQn                 = 26,      
        I2S0_IRQn                  = 27,     
        CAPS_IRQn                 = 28,      
        ADC_IRQn                  = 29,      
        IRQ30n                    = 30,
        RTC_IRQn                  = 31       

         
    } IRQn_Type;






 

 



   

#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"


 







#line 114 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 



#line 268 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"



#line 619 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

   

#line 117 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 260 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 296 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 615 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

   

#line 118 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"








 
#line 143 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 159 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 123 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\system_ISD9100.h"
 





















 












 






extern uint32_t SystemCoreClock;      
extern uint32_t CyclesPerUs;          
extern uint32_t gau32HiRCSrcTbl[];    






 
extern void SystemInit (void);






 
extern void SystemCoreClockUpdate (void);





#line 124 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"


  #pragma anon_unions
#line 147 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"


 
 
 





 


 



 
 
typedef struct
{


    















 
    volatile uint32_t CTL0;                  

    















 
    volatile uint32_t CTL1;                  

    




















 
    volatile uint32_t STATUS;                

    








 
    volatile uint32_t POSSEL;                

} ACMP_T;




 


































   
   


 



 
 
typedef struct
{


    










 
    volatile const  uint32_t DAT;                   

    









 
    volatile uint32_t CHEN;                  

    












 
    volatile uint32_t CLKDIV;                

    
















 
    volatile uint32_t DCICTL;                

    











 
    volatile uint32_t INTCTL;                

    








 
    volatile uint32_t PDMACTL;               

    


























 
    volatile uint32_t CMP0;                  

    


























 
    volatile uint32_t CMP1;                  

} ADC_T;




 





























































   
   


 



 
 
typedef struct
{


    











































































 
    volatile uint32_t CTL;                   

    














 
    volatile const  uint32_t STS;                   

    









 
    volatile uint32_t INTSTS;                

    









 
    volatile uint32_t INTCTL;                

} ALC_T;




 





























































   
   


 



 
 
typedef struct
{


    

















 
    volatile uint32_t VMID;                  
         uint32_t RESERVE0[1];


    
















 
    volatile uint32_t CURCTL0;               
         uint32_t RESERVE1[5];


    












 
    volatile uint32_t LDOSEL;                

    













 
    volatile uint32_t LDOPD;                 

    





















 
    volatile uint32_t MICBSEL;               

    









 
    volatile uint32_t MICBEN;                
         uint32_t RESERVE2[8];


    



















 
    volatile uint32_t MUXCTL;                
         uint32_t RESERVE3[3];


    






















 
    volatile uint32_t PGACTL;                

    

































 
    volatile uint32_t SIGCTL;                

    











 
    volatile uint32_t PGAGAIN;               
         uint32_t RESERVE4[6];


    














 
    volatile uint32_t TRIM;                  
         uint32_t RESERVE5[1];


    




























 
    volatile uint32_t CAPSCTL;               

    







 
    volatile const  uint32_t CAPSCNT;               

    



















 
    volatile uint32_t FQMMCTL;               

    










 
    volatile const  uint32_t FQMMCNT;               

} ANA_T;




 








































































































































   
   


 



 
 
typedef struct
{


    








 
    volatile uint32_t COEFF0;                

    








 
    volatile uint32_t COEFF1;                

    








 
    volatile uint32_t COEFF2;                

    








 
    volatile uint32_t COEFF3;                

    








 
    volatile uint32_t COEFF4;                

    








 
    volatile uint32_t COEFF5;                

    








 
    volatile uint32_t COEFF6;                

    








 
    volatile uint32_t COEFF7;                

    








 
    volatile uint32_t COEFF8;                

    








 
    volatile uint32_t COEFF9;                

    








 
    volatile uint32_t COEFF10;               

    








 
    volatile uint32_t COEFF11;               

    








 
    volatile uint32_t COEFF12;               

    








 
    volatile uint32_t COEFF13;               

    








 
    volatile uint32_t COEFF14;               
         uint32_t RESERVE0[1];


    






























 
    volatile uint32_t CTL;                   

} BIQ_T;




 
































































   
   


 



 
 
typedef struct
{


    


















 
    volatile uint32_t BODSEL;                

    


















 
    volatile uint32_t BODCTL;                

    












 
    volatile uint32_t TALMSEL;               

    
















 
    volatile uint32_t TALMCTL;               

    










 
    volatile uint32_t BODDTMR;               

} BODTALM_T;




 








































   
   


 



 
 
typedef struct
{


    



















































 
    volatile uint32_t PWRCTL;                

    














 
    volatile uint32_t AHBCLK;                

    






















































 
    volatile uint32_t APBCLK0;               

    










 
    volatile uint32_t DPDSTATE;              

    


























 
    volatile uint32_t CLKSEL0;               

    
































 
    volatile uint32_t CLKSEL1;               

    












 
    volatile uint32_t CLKDIV0;               

    











 
    volatile uint32_t CLKSEL2;               

    
































































 
    volatile uint32_t SLEEPCTL;              

    












 
    volatile uint32_t PWRSTSF;               

    













 
    volatile uint32_t DBGPD;                 

} CLK_T;




 






















































































































































































































   
   


 



 
 
typedef struct
{


    


















 
    volatile uint32_t CTL;                   

    














 
    volatile uint32_t DAT;                   

    









 
    volatile const  uint32_t CHECKSUM;              

} CRC_T;




 













   
   


 



 
 
typedef struct
{


    




























 
    volatile uint32_t CTL;                   

    












 
    volatile const  uint32_t STS;                   

    









 
    volatile uint32_t DMACTL;                

    









 
    volatile  uint32_t DATA;                  

    











 
    volatile uint32_t ZOHDIV;                

} DPWM_T;




 




























   
   


 



 
 
typedef struct
{


    































 
    volatile uint32_t ISPCTL;                

    









 
    volatile uint32_t ISPADDR;               

    









 
    volatile uint32_t ISPDAT;                

    














 
    volatile uint32_t ISPCMD;                

    













 
    volatile uint32_t ISPTRG;                

    









 
    volatile const  uint32_t DFBA;                  

} FMC_T;




 


































   
   


 



 
 
typedef struct
{


    






































































































 
    volatile uint32_t MODE;               

    






















































 
    volatile uint32_t DINOFF;             

    






































































 
    volatile uint32_t DOUT;               

    






















































































 
    volatile uint32_t DATMSK;             

    






































 
    volatile const  uint32_t PIN;                

    






















































































































 
    volatile uint32_t DBEN;               

    






































































































































 
    volatile uint32_t INTTYPE;            

    






































































































































































































































































 
    volatile uint32_t INTEN;              

    






















































































































 
    volatile uint32_t INTSRC;                     

} GPIO_T;


typedef struct { 
    

















 
    volatile uint32_t DBCTL;  
} GPIO_DB_T; 




 










































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   
   


 



 
 
typedef struct
{


    





























 
    volatile uint32_t CTL;                   

    













 
    volatile uint32_t ADDR0;                 

    









 
    volatile uint32_t DAT;                   

    


















 
    volatile const  uint32_t STATUS;                

    








 
    volatile uint32_t CLKDIV;                

    


















 
    volatile uint32_t TOCTL;                 

    













 
    volatile uint32_t ADDR1;                 

    













 
    volatile uint32_t ADDR2;                 

    













 
    volatile uint32_t ADDR3;                 

    
















































 
    volatile uint32_t ADDRMSK0;              

    
















































 
    volatile uint32_t ADDRMSK1;              

    
















































 
    volatile uint32_t ADDRMSK2;              

    
















































 
    volatile uint32_t ADDRMSK3;              

} I2C_T;




 

















































































































































   
   


 



 
 
typedef struct
{


    













































































 
    volatile uint32_t CTL;                   

    
























 
    volatile uint32_t CLKDIV;                

    





































 
    volatile uint32_t IEN;                   

    





















































































 
    volatile const  uint32_t STATUS;                

    










 
    volatile  uint32_t TX;                    

    










 
    volatile const  uint32_t RX;                    

} I2S_T;




 

















































































































































   
   


 



 
 
typedef struct
{
    



























































 
    volatile uint32_t DSCT_CTL;             

    









 
    volatile uint32_t DSCT_ENDSA;           

    









 
    volatile uint32_t DSCT_ENDDA;           

    









 
    volatile uint32_t TXBCCH;               

    










 
    volatile const  uint32_t INLBPCH0;              

    









 
    volatile const  uint32_t CURSACH;              

    









 
    volatile const  uint32_t CURDACH;              

    









 
    volatile const  uint32_t CURBCCH;              

    





















 
    volatile uint32_t INTENCH;              

    



























 
    volatile uint32_t CHIF;                 
 } PDMA_T;

typedef struct
{
    
















 
    volatile uint32_t GLOCTL;                

    






















 
    volatile uint32_t SVCSEL;                
         uint32_t RESERVE4[1];


    








 
    volatile const  uint32_t GLOBALIF;              

} PDMA_GCR_T;




 




































































































   
   


 



 
 
typedef struct
{


    













 
    volatile uint32_t CLKPSC;                

    















 
    volatile uint32_t CLKDIV;                

    






























 
    volatile uint32_t CTL;                   

    
















 
    volatile uint32_t PERIOD0;               

    















 
    volatile uint32_t CMPDAT0;               

    








 
    volatile const  uint32_t CNT0;                  

    
















 
    volatile uint32_t PERIOD1;               

    















 
    volatile uint32_t CMPDAT1;               

    








 
    volatile const  uint32_t CNT1;                  
         uint32_t RESERVE0[7];


    












 
    volatile uint32_t INTEN;                 

    










 
    volatile uint32_t INTSTS;                
         uint32_t RESERVE1[2];


    




















































 
    volatile uint32_t CAPCTL01;              
         uint32_t RESERVE2[1];


    








 
    volatile const  uint32_t RCAPDAT0;              

    








 
    volatile const  uint32_t FCAPDAT0;              

    








 
    volatile const  uint32_t RCAPDAT1;              

    








 
    volatile const  uint32_t FCAPDAT1;              
         uint32_t RESERVE3[4];


    













 
    volatile uint32_t CAPINEN;               

    














 
    volatile uint32_t POEN;                  

} PWM_T;




 































































































































   
   


 



 
 
typedef struct
{


    












 
    volatile  uint32_t INIT;                  

    




























 
    volatile  uint32_t RWEN;                  

    














 
    volatile uint32_t FREQADJ;               

    












 
    volatile uint32_t TIME;                  

    












 
    volatile uint32_t CAL;                   

    
















 
    volatile uint32_t CLKFMT;                

    









 
    volatile uint32_t WEEKDAY;               

    












 
    volatile uint32_t TALM;                  

    












 
    volatile uint32_t CALM;                  

    









 
    volatile const  uint32_t LEAPYEAR;              

    












 
    volatile uint32_t INTEN;                 

    
















 
    volatile uint32_t INTSTS;                

    














 
    volatile uint32_t TICK;                  

} RTC_T;




 






















































































































   
   


 



 
 
typedef struct
{


    

























































































 
    volatile uint32_t CTL;                   

    

















 
    volatile uint32_t CLKDIV;                

    




























 
    volatile uint32_t SSCTL;                 
         uint32_t RESERVE0[1];


    











 
    volatile const  uint32_t RX0;                   

    











 
    volatile const  uint32_t RX1;                   
         uint32_t RESERVE1[2];


    











 
    volatile  uint32_t TX0;                   

    











 
    volatile  uint32_t TX1;                   
         uint32_t RESERVE2[3];


    













 
    volatile uint32_t VARCLK;                

    














 
    volatile uint32_t PDMACTL;               

} SPI_T;




 










































































































   
   


 



 
 
typedef struct
{

    







 
         uint32_t PDID;


    
































 
    volatile uint32_t RSTSTS;                

    






















 
    volatile uint32_t IPRST0;                

    













































 
    volatile uint32_t IPRST1;                
         uint32_t RESERVE1[8];


    






































































 
    volatile uint32_t PASMTEN;               

    






































 
    volatile uint32_t PBSMTEN;               

    






































































 
    volatile uint32_t GPA_MFP;               

    













































 
    volatile uint32_t GPB_MFP;               
         uint32_t RESERVE2[5];


    


















 
    volatile uint32_t WKCTL;                 
         uint32_t RESERVE3[42];


    









 
    volatile uint32_t REGLCTL;               
         uint32_t RESERVE4[3];


    


















 
    volatile uint32_t IRCTCTL;               

} SYS_T;




 











































































































































































































































   
   


 



 
 
typedef struct
{


    





































 
    volatile uint32_t CTL;                   

    













 
    volatile uint32_t CMP;                   

    










 
    volatile uint32_t INTSTS;                

    









 
    volatile uint32_t CNT;                   

} TMR_T;




 































   
   


 



 
 
typedef struct
{


    









 
    volatile uint32_t DAT;                   

    










































 
    volatile uint32_t INTEN;                 

    




























 
    volatile uint32_t FIFO;                  

    


























 
    volatile uint32_t LINE;                  

    



















 
    volatile uint32_t MODEM;                 

    
















 
    volatile uint32_t MODEMSTS;              

    

















































 
    volatile uint32_t FIFOSTS;               

    












































































 
    volatile  uint32_t INTSTS;                

    











 
    volatile uint32_t TOUT;                  

    





















 
    volatile uint32_t BAUD;                  

    

















 
    volatile uint32_t IRDA;                  

    
















 
    volatile uint32_t ALTCTL;                

    













 
    volatile uint32_t FUNCSEL;               

} UART_T;




 

















































































































































































































































   
   


 



 
 
typedef struct
{


    









































 
    volatile uint32_t CTL;                   

} WDT_T;




 






















   
   

   
typedef struct
{
    volatile uint32_t	D[64];
} SBRAM_T;
   


   

 
 
 



 
 





 

#line 8857 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"

#line 8865 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"












   
 
 
 


 














































   











 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 


   







 













 
#line 9124 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"

 










   

   




 
 
 
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"
 









 










 



 



   

 
 
 







#line 50 "..\\..\\..\\..\\Library\\StdDriver\\inc\\acmp.h"

   



 












 







 







 







 







 







 







 







 
















 


void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32PosPin);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);


   

   

   







 

#line 9147 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"
 









 










 



 



   

 
 
 





 
 
 





 
 
 





 
 
 





 
 
 
#line 75 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

 
 
 










#line 98 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

#line 115 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

 
 
  






 
 
 


	






 
 
 












#line 160 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

   




 








 


                         










 











 


                         
                        






 










 









 


                                  




 














 
#line 264 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"






 














 
#line 293 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"






 







 







 













 








 















 








 









 









 













 













 
















 









 









 







 






 






 

  







 










 









 


















 








 







 






 






 



void ADC_Open(void);
void ADC_Close(void);
uint32_t ADC_GetSampleRate(void);
void ADC_EnableInt(uint32_t u32Mask);
void ADC_DisableInt(uint32_t u32Mask);
uint32_t ADC_GetIntFlag(uint32_t u32Mask);
void ADC_ClearIntFlag(uint32_t u32Mask);
void ADC_SetAMUX(uint32_t u32AMUXSel, uint32_t u32MUXPSel, uint32_t u32MUXNSel);
void ADC_SetGPIOChannel(uint32_t u32Mode);
void ADC_EnableMICBias(uint32_t u32BiasSel);
void ADC_DisableMICBias(void);
int32_t ADC_SetPGAGaindB(int32_t i32PGAGainIndB);
int32_t ADC_SetALCMaxGaindB(int32_t i32MaxGaindB);
int32_t ADC_SetALCMinGaindB(int32_t i32MinGaindB);
int32_t ADC_SetALCTargetLevel(int32_t i32TargetLevel);
int32_t ADC_GetPGAGaindB(void);

   

   

   







 
#line 9148 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\biq.h"
 









 










 



 



   

 
 
 




   



 









 












 








 






 


void BIQ_SetCoeff(uint32_t u32BiqCoeff[15]);
void BIQ_Reset(void);

   

   

   







     
#line 9149 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\bod.h"
 








 












 



 




 

 
 
 



#line 48 "..\\..\\..\\..\\Library\\StdDriver\\inc\\bod.h"


 
 
 





   




 





 
static __inline void BOD_EnableInt(BODTALM_T *BOD)
{
     BOD->BODCTL |= (0x1ul << (2));    
}





 
static __inline void BOD_DisableInt(BODTALM_T *BOD)
{
     BOD->BODCTL &= (~(0x1ul << (2)));    
}





 
static __inline void BOD_ClearIntFlag(BODTALM_T *BOD)
{
    BOD->BODCTL |= (0x1ul << (3));
}








 
static __inline uint32_t BOD_GetIntFlag(BODTALM_T *BOD)
{
    return ((BOD->BODCTL&(0x1ul << (3))) ? 1 : 0 );
}





 
static __inline void BOD_EnableHyst(BODTALM_T *BOD)
{
     BOD->BODSEL |= (0x1ul << (3));    
}





 
static __inline void BOD_DisableHyst(BODTALM_T *BOD)
{
     BOD->BODSEL &= (~(0x1ul << (3)));    
}







 
static __inline uint32_t BOD_GetOutput(BODTALM_T *BOD)
{
     return ((BOD->BODCTL&(0x1ul << (4))) ? 1 : 0);    
}

void BOD_Open(uint8_t u8Mode, uint8_t u8BODLevel);
void BOD_Close(void);
void BOD_SetDetectionTime(uint8_t u8OnDUR, uint16_t u16OffDUR);



   

   

   







 



#line 9150 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\capsense.h"
 








 
 










 



 



 
	
 
 
 





 
 
 
#line 51 "..\\..\\..\\..\\Library\\StdDriver\\inc\\capsense.h"

 
 
 





   



 







void     CapSense_SelectCurrentSourceValue( uint16_t u16SrcSel );
void     CapSense_SetCycleCounts( uint32_t u32Count, uint16_t u16LowTime );

void     CapSense_ResetCounter(void);
uint32_t CapSense_GetCounter(void);

   

   

   







 
#line 9151 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
 








 











 



 



 

 
 
 














 
 
 













 
 
 

























 
 
 





 
 
 




 
 
 




 
 
 





 
 
 
#line 135 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
 
 
 
#line 156 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

   




 









 










 


uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetHIRCFreq(void);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32HIRCType, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_SysTickDelay(uint32_t us);
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel);
void CLK_StandbyPowerDown(void);
void CLK_DeepSleep(void);
void CLK_Sleep(void);
void CLK_EnableLDO( uint32_t u32LDOSel );
void CLK_DisableLDO(void);

   

   

   







 
#line 9152 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\crc.h"
 









 









	

 



 



 




   



 
	
 
 
 
int32_t CRC_Open(void);
int32_t CRC_Init(uint32_t eLSB, int32_t i32PacketLen);
int16_t CRC_Calc( uint32_t *Data, int32_t i32PacketLen);
void CRC_Close(void);

   

   

   







 
#line 9153 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\dpwm.h"
 









 










 



 



  

 
 
 
#line 43 "..\\..\\..\\..\\Library\\StdDriver\\inc\\dpwm.h"





   



 











 









 






 

















 







 







 







 







 






 






 






 
















 



void DPWM_Open(void);
void DPWM_Close(void);
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32count);
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate);
uint32_t DPWM_GetSampleRate(void);

   

   

   







 
#line 9154 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"
 









 










 



 



 

 
 
 









 
 
 









 
 
 




   




 

 
 
 

#line 82 "..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"

 
 
 

extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Close(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern void FMC_SetWaitState(uint32_t u32WaitCfg);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadDID(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


   

   

   







 
#line 9155 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"
 








 










 



 



 
	


 
 
 





 
 
 






 
 
 



 
 
 






#line 81 "..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"

   



 










 











 











 











 











 











 











 












 



















 










 











 










 










 



void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);

   

   

   








 
#line 9156 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"
 








 











 



 



 









   




 







 







 







 







 







 








 







 









 


uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
void I2C_Close(I2C_T *i2c);
void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_GetData(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);

   

   

   







 
#line 9157 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"









 



#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
 









 




















 











 

#line 9171 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"

#line 15 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"








 



 



 





 



 





 



 



 
#line 64 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"

#line 73 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2s.h"

 



 



   



 
 
 
 








 
static __inline void I2S_ENABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CTL |= (0x1ul << (16));
    else
        i2s->CTL |= (0x1ul << (17));
}









 
static __inline void I2S_DISABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CTL &= ~(0x1ul << (16));
    else
        i2s->CTL &= ~(0x1ul << (17));
}






 







 







 







 







 







 







 







 







 







 







 







 








 







 








 








 







 







 


uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat, uint32_t u32AudioInterface);
void I2S_Close(I2S_T *i2s);
void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask);
void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);

   


   

   



 

#line 9158 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\osc.h"
 








 
 










 



 



 
	
 
 
 
#line 43 "..\\..\\..\\..\\Library\\StdDriver\\inc\\osc.h"

#line 52 "..\\..\\..\\..\\Library\\StdDriver\\inc\\osc.h"
	
 
 
 
#line 63 "..\\..\\..\\..\\Library\\StdDriver\\inc\\osc.h"

 
 
 
#line 75 "..\\..\\..\\..\\Library\\StdDriver\\inc\\osc.h"

int32_t OSC_Trim(int32_t i32Target, uint8_t u8TrimIdx);

   

   

   







 

#line 9159 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"
 









 










 



 



 

 
 
 




 
 
 
#line 48 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"

 
 
 




 
 
 
#line 68 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"

   



 







 










 











 











 











 











 











 
#line 161 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"









 


void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferDirection(uint32_t u32Ch, uint32_t u32Direction);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);


   

   

   







 
#line 9160 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"
 








 










 



 



 
	
#line 40 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"



 
 
 




   




 










 
#line 76 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"








 









 












 
















 












 













 




uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntType);
void PWM_DisableInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);


   

   

   







 
#line 9161 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"
 








 












 



 




 
















#line 57 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

#line 65 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"
   




 



 
typedef struct {
    uint32_t u32Year;           
    uint32_t u32Month;          
    uint32_t u32Day;            
    uint32_t u32DayOfWeek;      
    uint32_t u32Hour;           
    uint32_t u32Minute;         
    uint32_t u32Second;         
    uint32_t u32TimeScale;      
    uint32_t u32AmPm;           
} S_RTC_TIME_DATA_T;

   






 










 









 









 









 









 



void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX100);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_EnableWakeUp(void);
void RTC_DisableWakeUp(void);


   


   

   









 
#line 9162 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"
 








 










 



 



 



















   



 






 







 









 









 









 









 







 








 








 








 








 








 








 








 









 








 








 









 








 









 







 







 







 













 







 







 







 







 







 








 








 


uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32BusClock, uint32_t u32VarClock);
void     SPI_Close(SPI_T *spi);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void     SPI_SetVarClock(SPI_T *spi, uint32_t u32VarClock);
uint32_t SPI_GetBusClock(SPI_T *spi);
uint32_t SPI_GetVarClock(SPI_T *spi);

   

   

   







 
#line 9163 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 








 
 










 



 



 

 
 
 





 	
 
 
 
#line 60 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 


#line 82 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"


#line 92 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"

 
 
 






 































































































































   



 











void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_Lock(uint8_t u8Lock);
void SYS_UnlockReg(void);
uint8_t SYS_Unlock(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);

   

   

   







 
#line 9164 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\talarm.h"
 








 












 



 




 

 
 
 






 
 
 






   



 





 
static __inline void TALARM_EnableInt(BODTALM_T *TALM)
{
     TALM->TALMCTL |= (0x1ul << (2));    
}





 
static __inline void TALARM_DisableInt(BODTALM_T *TALM)
{
     TALM->TALMCTL &= (~(0x1ul << (2)));    
}





 
static __inline void TALARM_ClearIntFlag(BODTALM_T *TALM)
{
    TALM->TALMCTL |= (0x1ul << (3));
}







 
static __inline uint32_t TALARM_GetIntFlag(BODTALM_T *TALM)
{
    return ((TALM->TALMCTL&(0x1ul << (3))) ? 1 : 0 );
}







 
static __inline uint32_t TALARM_GetOutput(BODTALM_T *TALM)
{
     return ((TALM->TALMCTL&(0x1ul << (1))) ? 1 : 0);         	
}

void TALARM_Open(uint8_t u8TALMVL);
void TALARM_Close(void);

   

   

   







 




#line 9165 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\timer.h"
 








 










 



 



 





   




 







 









 








 






 
static __inline void TIMER_Start(TMR_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}





 
static __inline void TIMER_Stop(TMR_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}





 
static __inline void TIMER_EnableInt(TMR_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}





 
static __inline void TIMER_DisableInt(TMR_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}







 
static __inline uint32_t TIMER_GetIntFlag(TMR_T *timer)
{
    return(timer->INTSTS & (0x1ul << (0)) ? 1 : 0);
}





 
static __inline void TIMER_ClearIntFlag(TMR_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}




 
static __inline uint32_t TIMER_GetCounter(TMR_T *timer)
{
    return timer->CNT;
}

uint32_t TIMER_Open(TMR_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void     TIMER_Close(TMR_T *timer);
void     TIMER_Delay(TMR_T *timer, uint32_t u32Usec);
uint32_t TIMER_GetModuleClock(TMR_T *timer);
uint32_t TIMER_GetWorkingFreq(TMR_T *timer);


   

   

   







 
#line 9166 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\uart.h"
 








 











 



 



 

 
 
 











 
 
 















 
 
 



 
 
 




 
 
 





   




 











 










 











 









 










 










 









 










 











 












 












 










 










 



















 


















 




















 








 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= (0x1ul << (1));
}






 
static __inline void UART_SET_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}







 









 



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 




#line 9167 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"
 








 












 



 




 
	
#line 42 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"

   




 





 






 








 








 







 


void WDT_Open( uint32_t u32TimeoutInterval, uint32_t u32EnableReset );
void WDT_Close(void);
void WDT_EnableInt(void);
void WDT_DisableInt(void);
void WDT_ClearResetFlag(void);
void WDT_ClearTimeOutIntFlag(void);
void WDT_ResetCounter(void);

   

   

   







 
#line 9168 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\ISD9100.h"




#line 11 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\Platform.h"



#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\NVTTypes.h"




 





#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 13 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\NVTTypes.h"










typedef void *				PVOID;




typedef unsigned char		BOOL;




typedef unsigned char *		PBOOL;




typedef signed char			INT8;




typedef signed char *		PINT8;




typedef unsigned char		UINT8;




typedef unsigned char *		PUINT8;




typedef signed short		INT16;




typedef signed short *		PINT16;




typedef unsigned short		UINT16;




typedef unsigned short *	PUINT16;




typedef signed int			INT32;




typedef signed int *		PINT32;




typedef unsigned int		UINT32;




typedef unsigned int *		PUINT32;


#line 119 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\NVTTypes.h"



typedef signed __int64		INT64;




typedef signed __int64 *	PINT64;




typedef unsigned __int64	UINT64;




typedef unsigned __int64	*PUINT64;






typedef float				FLOAT;




typedef float *				PFLOAT;




typedef double				DOUBLE;




typedef double *			PDOUBLE;




typedef signed char			CHAR;




typedef signed char *		PCHAR;




typedef signed char *		PSTR;




typedef const signed char *	PCSTR;


#line 206 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\NVTTypes.h"
typedef	wchar_t				WCHAR;




typedef	wchar_t *			PWCHAR;




typedef	wchar_t *			PWSTR;




typedef	const wchar_t *		PCWSTR;






typedef UINT32				SIZE_T;




typedef volatile UINT8		REG8;




typedef volatile UINT16		REG16;




typedef volatile UINT32		REG32;




typedef	unsigned char		BYTE;


typedef UINT8				ERRCODE;



#line 15 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\ISD9100\\Include\\Platform.h"

#line 12 "..\\ConfigApp.h"
#line 1 "..\\ConfigIP.h"
 
 
 
 
 



#line 10 "..\\ConfigIP.h"








 
 
 
 












#line 40 "..\\ConfigIP.h"
												
												
												
												
												
												




#line 57 "..\\ConfigIP.h"
												
												
												
												
												
												
												





























#line 100 "..\\ConfigIP.h"


#line 13 "..\\ConfigApp.h"
#line 1 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"




 





#line 12 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"
#line 1 "..\\..\\..\\..\\Library\\Framework\\inc\\SysInfra.h"




 





#line 12 "..\\..\\..\\..\\Library\\Framework\\inc\\SysInfra.h"
#line 1 "..\\..\\..\\..\\Library\\Framework\\inc\\ModuleID.h"




 





typedef enum
{
	
	
	
	MODULE_ID_DRVPROTECT		= 0,		
	MODULE_ID_DRVINT			= 1,		
	MODULE_ID_DRVADC			= 2,		
	MODULE_ID_DRVFMC			= 3,		
	MODULE_ID_DRVAIC			= 4,		
	MODULE_ID_DRVAPU			= 6,		
	MODULE_ID_DRVAUDIOADC		= 8,		
	MODULE_ID_DRVCACHE			= 10,		
	MODULE_ID_DRVAES			= 11,		
	MODULE_ID_DRVEBI			= 12,		
	MODULE_ID_DRVEDMA			= 13,		
	MODULE_ID_DRVGDMA			= 14,		
	MODULE_ID_DRVFSC			= 15,		
	MODULE_ID_DRVGE				= 16,		
	MODULE_ID_DRVGPIO			= 18,		

	MODULE_ID_DRVGPU			= 20,		
	MODULE_ID_DRVI2C			= 22,		
	MODULE_ID_DRVI2S			= 24,		
	MODULE_ID_DRVI2SM			= 26,		
	MODULE_ID_DRVMPU			= 28,		
	MODULE_ID_DRVNAND			= 30,		
	MODULE_ID_DRVNAND512		= 31,		
	MODULE_ID_DRVNOR			= 32,		
	MODULE_ID_DRVPWM			= 34,		
	MODULE_ID_DRVRTC			= 36,		
	MODULE_ID_DRVSDCARD			= 38,		
	MODULE_ID_DRVSIO			= 39,		

	MODULE_ID_DRVSPI			= 40,		
	MODULE_ID_DRVSPIMS			= 41,		
	MODULE_ID_DRVSPIFLASH		= 42,		
	MODULE_ID_DRVSPIM			= 43,		
	MODULE_ID_DRVSYS			= 44,		
	MODULE_ID_DRVSPU			= 45,		
	MODULE_ID_DRVTIMER			= 46,		
	MODULE_ID_DRVUART			= 48,		
	MODULE_ID_DRVUSB			= 50,		
	MODULE_ID_DRVUSBH			= 52,		
	MODULE_ID_DRVVDMA			= 54,		
	MODULE_ID_DRVVIDEOIN		= 56,		
	MODULE_ID_DRVVPOST			= 58,		

	MODULE_ID_DRVVRAM			= 60,		
	MODULE_ID_DRVW55U02			= 62,		
	MODULE_ID_DRVI2CH			= 64,		
	MODULE_ID_DRVWDT			= 66,		
	MODULE_ID_DRVJPEG			= 68,		
	
	MODULE_ID_DRVZEROG			= 70,		
	MODULE_ID_DRVSI2C			= 71,		
	MODULE_ID_DRVRFWYS			= 72,		
	MODULE_ID_DRVBLT			= 73,		
	MODULE_ID_DRVSMB			= 74,		
	MODULE_ID_DRVSDRM			= 75,		

	
	MODULE_ID_AACDEC			= 80,		
	MODULE_ID_AEC				= 81,		
	MODULE_ID_BEATDET			= 82,		
	MODULE_ID_SNDEFF			= 83,		
	MODULE_ID_AUDIOSYN			= 84,		
	MODULE_ID_G726ADPCM			= 85,		
	MODULE_ID_IMAADPCM			= 86,		
	MODULE_ID_NUSOUND			= 87,		
	MODULE_ID_MP3DEC			= 88,		
	MODULE_ID_NUONECODEC		= 89,		
	MODULE_ID_PITCHCHANGE		= 90,		
	MODULE_ID_NOISEFILTER		= 91,		
	MODULE_ID_WAVFILEUTIL		= 92,		
	MODULE_ID_BNDET				= 93,		
	MODULE_ID_MIDISYN			= 94,		
	MODULE_ID_VOICECHANGE		= 95,		
	MODULE_ID_WMADEC			= 96,		
	MODULE_ID_WATERMARK			= 97,		
	MODULE_ID_WMADECDRM			= 98,		
	MODULE_ID_AUDIOCTRL			= 100,		
	MODULE_ID_EQ				= 106,		
	MODULE_ID_OGGDEC			= 110,		
	MODULE_ID_MP3ENC			= 112,		
	MODULE_ID_UADEC				= 114,		
	MODULE_ID_ULSPEECHDEC		= 115,		
	MODULE_ID_USPEECHDEC		= 116,		
	MODULE_ID_SPEECHRECOG		= 118,		
	MODULE_ID_REVERB			= 119,		

	
	MODULE_ID_FS				= 120,		
	
	
	MODULE_ID_FL				= 128,		
	
	
	MODULE_ID_KEYPAD			= 130,		
	MODULE_ID_IRCTRL			= 131,		
	MODULE_ID_LWIP				= 132,		
	MODULE_ID_SEMIHOST			= 133,		
	MODULE_ID_WLANMGR			= 134,		
	MODULE_ID_VSCOMMU			= 135,		
	MODULE_ID_HTTPD				= 136,		
	MODULE_ID_SOFTUART			= 137,		
	MODULE_ID_VIRTUALCOM		= 139,		

	
	MODULE_ID_GFXRESLDR			= 140,		
	MODULE_ID_GFXLIB			= 141,		
	MODULE_ID_IMGPROC			= 142,		
	MODULE_ID_GIFDEC			= 143,		
	MODULE_ID_JPEG				= 144,		
	MODULE_ID_PNGDEC			= 146,		
	MODULE_ID_BARCODE2D			= 148,		
	MODULE_ID_PTNRECOG			= 150,		
	MODULE_ID_MOTIONDET			= 152,		
	
	
	MODULE_ID_STORIF			= 160,		
	MODULE_ID_SDCARD			= 161,		
	MODULE_ID_SYSNAND			= 162,		
	MODULE_ID_SPIFLASH			= 163,		
	MODULE_ID_WTRIF				= 164,		
	MODULE_ID_NORFLASH			= 165,		
	MODULE_ID_SYSNANDLITE		= 166,		
	MODULE_ID_XTRAROM			= 167,		
	MODULE_ID_NOR				= 169,		
	MODULE_ID_W55F					= 156,		
	MODULE_ID_SYSNAND512		= 157,		
	MODULE_ID_SYSNANDLITE512	= 158,		
	MODULE_ID_SYSNANDOTP		= 159,		
	
	
	MODULE_ID_INTMGR			= 180,		
	MODULE_ID_BLKLDR			= 181,		
	MODULE_ID_MEMMGR			= 182,		
	MODULE_ID_EVTMGR			= 183,		
	MODULE_ID_PROF				= 184,		
	MODULE_ID_PROGLDR			= 186,		
	MODULE_ID_SYSINFRA			= 188,		
	MODULE_ID_TIMERCTRL			= 190,		
	MODULE_ID_TIMEUTIL			= 192,		
	MODULE_ID_CONPROGLDR		= 194,		
	MODULE_ID_IXML				= 185,		
	
	
	MODULE_ID_USBCOREH			= 78,		
	MODULE_ID_HID				= 220,		
	MODULE_ID_MASSSTOR			= 222,		
	MODULE_ID_MASSSTORHID		= 224,		
	MODULE_ID_MASSSTORLITE		= 226,		
	MODULE_ID_MTP				= 230,		
	MODULE_ID_USBINFRA			= 232,		
	MODULE_ID_UAC				= 234,		
	MODULE_ID_UAVC				= 236,		
	MODULE_ID_UVC				= 238,		
	MODULE_ID_MASSSTORH			= 252,		
	MODULE_ID_HIDH				= 254,		

	
	MODULE_ID_MSDRMPD			= 228,		
	
	
	MODULE_ID_AVICODEC			= 240,		
	MODULE_ID_MJPEG				= 242,		
	MODULE_ID_WIVICORE			= 244,		
	MODULE_ID_WIVI				= 246,		
	MODULE_ID_AVCTRL			= 248,		
	MODULE_ID_AVIUTIL			= 250,		
	
	
	MODULE_ID_WTCHAN			= 168,		
	MODULE_ID_WTCMDSERV			= 170,		
	MODULE_ID_WTDISPLAY			= 172,		
	MODULE_ID_WTMEDIA			= 174,		
	MODULE_ID_WTSYS				= 176,		
	MODULE_ID_WTTRANS			= 178,		
	
	
	MODULE_ID_WTRFCHAN			= 171,		
	MODULE_ID_WTRFCMDSERV		= 173,		
	MODULE_ID_WTRFMEDIA			= 175,		
	MODULE_ID_WTRFSYS			= 177,		
	MODULE_ID_WTRFTRANS			= 179		

	
} E_SYSINFRA_MODULE_ID;


#line 13 "..\\..\\..\\..\\Library\\Framework\\inc\\SysInfra.h"























UINT32 
SysInfra_GetVersion (void);

UINT32
SysInfra_CountLeadingZero (
	UINT32 u32Val
);

static __inline UINT32
SysInfra_CountLeadingOne (
	UINT32 u32Val
)
{
    return SysInfra_CountLeadingZero (~u32Val);
}

UINT32
SysInfra_CountTrailingZero (
	UINT32 u32Val
);

static __inline UINT32
SysInfra_CountTrailingOne (
	UINT32 u32Val
)
{
	return SysInfra_CountTrailingZero (~u32Val);
}










#line 13 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"




















#line 42 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"


#line 65 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"



#line 76 "..\\..\\..\\..\\Library\\Storage\\inc\\SPIFlash.h"

typedef struct
{
	SPI_T   *psSpiHandler;				
	UINT32  u32FlashSize;				
	UINT8   u8SlaveDevice;              
	UINT8   u8Flag;
} S_SPIFLASH_HANDLER;

typedef enum
{
	eSTATUS_REG1 = 0x00,
	eSTATUS_REG2 = 0x30
} E_SPIFLASH_STATUS_REGISTER;




 
 
 
void
SPIFlash_Write(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Addr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_WritePage(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32PageAddr,
	PUINT8 pau8Data
);

void
SPIFlash_WriteStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
);

void
SPIFlash_WriteEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

UINT32
SPIFlash_WriteData(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32SPIAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_WriteDataAlign(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	PUINT8 pau8Data
);

 
 
 
void
SPIFlash_Read(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_BurstRead(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_ReadStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
);

void
SPIFlash_ReadEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_ReadData(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_ReadDataAlign(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

 
 
 
void
SPIFlash_Erase(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr,
	UINT32 u32AddIncreament,
	UINT16 u16EraseCount
);

static __inline
void
SPIFlash_Erase64K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8  u8IndexOf64K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, ((UINT32)0xD8<<24)|(u8IndexOf64K<<16), (1<<16), u16EraseCount );
}

static __inline
void
SPIFlash_Erase4K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf4K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, (0x20<<24)|(u16IndexOf4K<<12), (1<<12), u16EraseCount );
}

static __inline
void
SPIFlash_Erase32K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf32K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, (0x52<<24)|(u16IndexOf32K<<15), (1<<15), u16EraseCount );
}

void
SPIFlash_EraseChip(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);


void
SPIFlash_EraseStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr
);

static __inline
void
SPIFlash_Erase64KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8  u8IndexOf64K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, ((UINT32)0xD8<<24)|(u8IndexOf64K<<16));
}

static __inline
void
SPIFlash_Erase4KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf4K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, (0x20<<24)|(u16IndexOf4K<<12));
}

static __inline
void
SPIFlash_Erase32KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf32K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, (0x52<<24)|(u16IndexOf32K<<15) );
}

void
SPIFlash_EraseChipStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

 
 
 
void
SPIFlash_ChipWriteEnable(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableWrite
);

void
SPIFlash_GlobalProtect(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableGlobalProtect
);

 
 
 
void
SPIFlash_Open(
	SPI_T *psSpiHandler,
	UINT8 u8DrvSlaveDevice,
	UINT32 u32SpiClk,
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

static __inline
void
SPIFlash_Close(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	SPI_Close(psSpiFlashHandler->psSpiHandler);
}

static __inline
UINT32
SPIFlash_GetSPIClock(
   S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	return SPI_GetBusClock(psSpiFlashHandler->psSpiHandler);
}

void
SPIFlash_SendRecOneData(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Data,
	UINT8  u8DataLen
);

void
SPIFlash_GetChipInfo(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

UINT8
SPIFlash_ReadStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	E_SPIFLASH_STATUS_REGISTER eStatusReg
);

void
SPIFlash_WriteStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8 u8Status
);

void
SPIFlash_PowerDown(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL	bEnable
);

void
SPIFlash_WaitReady(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

BOOL
SPIFlash_CheckBusy(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

UINT32
SPIFlash_GetVersion(void);








#line 14 "..\\ConfigApp.h"










#line 30 "..\\ConfigApp.h"


									
										





















#line 61 "..\\ConfigApp.h"



































#line 5 "..\\..\\..\\..\\Library\\Framework\\inc\\AudioMixer.h"
#line 1 "..\\..\\..\\..\\Library\\Framework\\inc\\PlaybackRecord.h"
 
 
 
 
 




#line 11 "..\\..\\..\\..\\Library\\Framework\\inc\\PlaybackRecord.h"
#line 1 "..\\..\\..\\..\\Library\\Framework\\inc\\BufCtrl.h"



#line 5 "..\\..\\..\\..\\Library\\Framework\\inc\\BufCtrl.h"






	
#line 22 "..\\..\\..\\..\\Library\\Framework\\inc\\BufCtrl.h"






typedef struct sBufCtrl
{
	UINT8 u8Flag;		
	
	UINT16 u16BufCount;					
	INT16  *pi16Buf;					
	UINT16 u16BufReadIdx;				
	UINT16 u16BufWriteIdx;				
	
	UINT16 u16SampleRate;				
	UINT16 u16FrameSize;				
	UINT16 u16ReSamplingCalculation;	
}S_BUF_CTRL;


typedef UINT8 (*PFN_DATA_REQUEST_CALLBACK)(void *pParam, INT16 i16DataBufCount, INT16 ai16DataBuf[]);
typedef struct sCallbackCtrl
{
	UINT8 u8Flag;		
	
	PFN_DATA_REQUEST_CALLBACK  pfnFunc;
	UINT8 *pu8Param;
} S_BUF_CTRL_CALLBACK;


void BufCtrl_ReadWithCount(S_BUF_CTRL *psSrc, UINT16 u16ConsumeCount, INT16 *pi16Des);
void BufCtrl_WriteWithCount(S_BUF_CTRL *psDes, UINT16 u16WriteCount, INT16 *pi16Src);

#line 12 "..\\..\\..\\..\\Library\\Framework\\inc\\PlaybackRecord.h"










#line 35 "..\\..\\..\\..\\Library\\Framework\\inc\\PlaybackRecord.h"

#line 53 "..\\..\\..\\..\\Library\\Framework\\inc\\PlaybackRecord.h"
	

















void Playback_StartPlay(void);












void Playback_StopPlay(void);

void Record_StopRec(void);

void 
Record_SetInBufCallback(
	S_BUF_CTRL_CALLBACK *psAdcBufCtrl,
	PFN_DATA_REQUEST_CALLBACK pfnSetIntputData,
	void* pWorkBuf
);

void 
Record_SetInBufRecord(
	S_BUF_CTRL* psInBufCtrl,
	UINT16 u16BufSize,
	INT16* pi16Buf,
	UINT16 u16FrameSize,
	UINT16 u16SampleRate
);

















void
Playback_SetOutputBuf(
	S_BUF_CTRL* psOutBufCtrl,
	UINT16 u16BufSize,
	INT16* pi16Buf,
	UINT16 u16FrameSize,
	UINT16 u16SampleRate
);














BOOL
Playback_NeedUpdateOutputBuf(
	S_BUF_CTRL* psOutBufCtrl
);













void
Playback_UpdateOutputBuf(
	S_BUF_CTRL* psOutBufCtrl
);

void 
Playback_Initiate(
	void
	);

void 
Playback_Add(
	UINT8 u8Channel,
	S_BUF_CTRL *psBufCtrl
);

void
Playback_Remove(
	UINT8 u8Channel
	);










void Playback_PauseCtrl(uint8_t u8Channel,BOOL bEnable);










void Playback_MuteCtrl(uint8_t u8Channel,BOOL bEnable);















BOOL Playback_SetVolumeDB(uint8_t u8Channel,uint32_t u32DBValue);












void Playback_AdjustVolumeDBStep(uint8_t u8Channel, int8_t i8AdjustDBStep, uint16_t u16StepIntervalBytes);













BOOL Playback_IsTargetVolume(uint8_t u8Channel);










void Playback_VolumeFadeOut(uint8_t u8ChannelIndex,uint16_t u16StepIntervalBytes);










void Playback_VolumeFadeIn(uint8_t u8ChannelIndex,uint16_t u16StepIntervalBytes);










void Playback_ProcessVolume(uint8_t u8Channel,int16_t* pi16Data);









void Playback_ResetChannelVolume(uint8_t u8Channel);

void Record_Add(
	S_BUF_CTRL *psAdcBufCtrl,
	UINT32 u32SampleRate
);

void Record_StartRec(void);





#line 6 "..\\..\\..\\..\\Library\\Framework\\inc\\AudioMixer.h"






UINT8 AudioMixer_MixProcess(S_BUF_CTRL **pParam, INT16 i16DataBufCount, INT16 ai16DataBuf[]);

UINT32 AudioMixer_Initiate(S_BUF_CTRL_CALLBACK *psMixerCtl, S_BUF_CTRL **ppsInBufCtrlList);

UINT32 AudioMixer_ChangeSR(S_BUF_CTRL **ppsInBufCtrlList);

#line 2 "..\\..\\..\\..\\Library\\Framework\\src\\AudioMixer.c"


#line 84 "..\\..\\..\\..\\Library\\Framework\\src\\AudioMixer.c"
UINT8 AudioMixer_MixProcess(S_BUF_CTRL **ppInBufCtrlList, INT16 i16DataBufCount, INT16 ai16DataBuf[])
{
	S_BUF_CTRL *psInBufCtrl;
	INT32 i32Temp;
	INT16 *pi16Src, i;
	UINT8 u8Ch = 0;
	UINT16 u16DupSampleCalculate, u16DupSampleRatio;
	
	for( i = 0; i < i16DataBufCount; i ++)
		ai16DataBuf[i] = 0;
	
	while( (psInBufCtrl = *ppInBufCtrlList ++) != ((S_BUF_CTRL*)0xffffffff) )
	{
		if ( (psInBufCtrl == 0) || (((psInBufCtrl)->u8Flag & 2) != 0) )
			continue;
		
		if ((((psInBufCtrl)->u8Flag & 8)!=0))
		{
			if (Playback_IsTargetVolume(u8Ch))
				continue;
		}		
			
		pi16Src = &psInBufCtrl->pi16Buf[psInBufCtrl->u16BufReadIdx];
		if (  psInBufCtrl->u16ReSamplingCalculation == 0 )
		{
			
			
			i = 0;
			do 
			{
				Playback_ProcessVolume(u8Ch, pi16Src);
				i32Temp = ai16DataBuf[i]; 
				i32Temp += *pi16Src++;
				if (i32Temp> (32767)) 
					i32Temp = (32767);
				else if (i32Temp < (-32768)) 
					i32Temp = (-32768);
				ai16DataBuf[i] = i32Temp;

				if ((psInBufCtrl->u16BufReadIdx+=1) >= psInBufCtrl->u16BufCount)
				{	
					psInBufCtrl->u16BufReadIdx = 0;
					pi16Src = &psInBufCtrl->pi16Buf[0];
				}
			}while(++i < i16DataBufCount);
				
			continue;
		}
			
			
			
			
			
			
			
			
			
			
			
			
		u16DupSampleRatio = psInBufCtrl->u16ReSamplingCalculation >> 8;
		u16DupSampleCalculate = psInBufCtrl->u16ReSamplingCalculation & 0xff;
		
		for( i = 0; i < i16DataBufCount; i ++)
		{
#line 173 "..\\..\\..\\..\\Library\\Framework\\src\\AudioMixer.c"
			
			INT32 i32PrevPCM, i32NextPCM;
			i32PrevPCM = *pi16Src;
			if ((psInBufCtrl->u16BufReadIdx+1) >= psInBufCtrl->u16BufCount)
				i32NextPCM = psInBufCtrl->pi16Buf[0];
			else
				i32NextPCM = *(pi16Src+1);
			
			Playback_ProcessVolume(u8Ch, pi16Src);
			Playback_ProcessVolume(u8Ch, (INT16 *)&i32NextPCM);
			
			i32Temp = ai16DataBuf[i];
			i32Temp += ((i32PrevPCM*(0x100-u16DupSampleCalculate) + (i32NextPCM*u16DupSampleCalculate))>>8);
			u16DupSampleCalculate = u16DupSampleCalculate+u16DupSampleRatio;
			if ( u16DupSampleCalculate >= 0x100 )
			{
				u16DupSampleCalculate -= 0x100;
				if ((++psInBufCtrl->u16BufReadIdx) >= psInBufCtrl->u16BufCount)
				{
					psInBufCtrl->u16BufReadIdx = 0;
					pi16Src = &psInBufCtrl->pi16Buf[0];
				}
				else
					pi16Src ++;
			}

			
			if (i32Temp > (32767)) 
				i32Temp = (32767);
			else if (i32Temp < (-32768)) 
				i32Temp = (-32768);
			ai16DataBuf[i] = i32Temp; 
		}
			
		psInBufCtrl->u16ReSamplingCalculation = (psInBufCtrl->u16ReSamplingCalculation&0xff00) | u16DupSampleCalculate;
	}
			
	return i16DataBufCount;
}


UINT32 AudioMixer_ChangeSR(S_BUF_CTRL **ppsInBufCtrlList)
{
	UINT32 u32MaxSampleRate;
	S_BUF_CTRL *psInBufCtrl;
	S_BUF_CTRL **ppsInBufCtrlListTemp;
	UINT8 u8DupSmampleRatio;
	
	u32MaxSampleRate = 0;
	ppsInBufCtrlListTemp = ppsInBufCtrlList;
	while( (psInBufCtrl =*ppsInBufCtrlListTemp ++)!= ((S_BUF_CTRL*)0xffffffff) )
	{
		if ( (psInBufCtrl == 0) || (((psInBufCtrl)->u8Flag & 2) != 0) )
			continue;
		if (psInBufCtrl->u16SampleRate == 0)
		{
			*ppsInBufCtrlListTemp = 0;
			continue;
		}
		if ( psInBufCtrl->u16SampleRate > u32MaxSampleRate )
			u32MaxSampleRate = psInBufCtrl->u16SampleRate;
	}

	ppsInBufCtrlListTemp = ppsInBufCtrlList;
	while( (psInBufCtrl =*ppsInBufCtrlListTemp ++)!= ((S_BUF_CTRL*)0xffffffff) )
	{
		if ( (psInBufCtrl == 0) || (((psInBufCtrl)->u8Flag & 2) != 0) )
			continue;
		
		if ( psInBufCtrl->u16SampleRate < u32MaxSampleRate )
		{




			u8DupSmampleRatio = (UINT8)(((((UINT32)psInBufCtrl->u16SampleRate)<<16)/u32MaxSampleRate)>>8);
			psInBufCtrl->u16ReSamplingCalculation = (u8DupSmampleRatio<<8);

		}
		else
			psInBufCtrl->u16ReSamplingCalculation = 0;
	}
	
	return u32MaxSampleRate;
}



UINT32 AudioMixer_Initiate(S_BUF_CTRL_CALLBACK *psMixerCtl, S_BUF_CTRL **ppsInBufCtrlList)
{
	psMixerCtl->u8Flag = 1;
	psMixerCtl->pfnFunc = (PFN_DATA_REQUEST_CALLBACK)AudioMixer_MixProcess;
	psMixerCtl->pu8Param = (void *)ppsInBufCtrlList;

	return AudioMixer_ChangeSR(ppsInBufCtrlList);
}

