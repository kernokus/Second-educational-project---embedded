///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     13/Mar/2020  20:04:46 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_4\main.c                      /
//    Command line =  F:\ProjectsE\MCU\FiltTest_4\main.c -D NDEBUG -D         /
//                    STM32F051 -D USE_STDPERIPH_DRIVER -lB                   /
//                    F:\ProjectsE\MCU\FiltTest_4\Release\List\ -o            /
//                    F:\ProjectsE\MCU\FiltTest_4\Release\Obj\                /
//                    --endian=little --cpu=Cortex-M0 -e --fpu=None           /
//                    --dlib_config "F:\Program Files\IAR Systems\Embedded    /
//                    Workbench 6.5\arm\INC\c\DLib_Config_Normal.h" -I        /
//                    F:\ProjectsE\MCU\FiltTest_4\ -I                         /
//                    F:\ProjectsE\MCU\FiltTest_4\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\CMSIS\Device\ST\STM32F0xx\Include\ -I     /
//                    F:\ProjectsE\MCU\FiltTest_4\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\CMSIS\Include\ -I                         /
//                    F:\ProjectsE\MCU\FiltTest_4\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\inc\ -Oh       /
//    List file    =  F:\ProjectsE\MCU\FiltTest_4\Release\List\main.s         /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME main

        #define SHT_PROGBITS 0x1

        EXTERN GPIO_Init
        EXTERN NVIC_Init
        EXTERN RCC_AHBPeriphClockCmd
        EXTERN RCC_APB2PeriphClockCmd
        EXTERN TIM_ClearITPendingBit
        EXTERN TIM_Cmd
        EXTERN TIM_GetITStatus
        EXTERN TIM_ITConfig
        EXTERN TIM_TimeBaseInit

        PUBLIC Enc_A_Prev
        PUBLIC Enc_B_Prev
        PUBLIC Enc_Process
        PUBLIC SD_State
        PUBLIC TIM1_BRK_UP_TRG_COM_IRQHandler
        PUBLIC Task_1
        PUBLIC Task_2
        PUBLIC Task_3
        PUBLIC Task_3_cnt
        PUBLIC Task_3_pos
        PUBLIC Timer1_Init
        PUBLIC main

// F:\ProjectsE\MCU\FiltTest_4\main.c
//    1 #include "stm32f0xx_gpio.h"
//    2 ////////////////////////////////////////////////////////////////////
//    3 typedef unsigned char  BYTE;
//    4 typedef unsigned short WORD;
//    5 typedef signed short   SWORD;
//    6 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//    7 void Timer1_Init()
//    8 {
Timer1_Init:
        PUSH     {R4,LR}
        SUB      SP,SP,#+16
//    9   NVIC_InitTypeDef NVIC_InitStructure;
//   10   TIM_TimeBaseInitTypeDef TIM;
//   11 
//   12   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        MOVS     R1,#+1
        LSLS     R0,R1,#+11
        BL       RCC_APB2PeriphClockCmd
//   13 
//   14   /* Enable the TIM gloabal Interrupt */
//   15   NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
        MOV      R0,SP
        MOVS     R1,#+13
        STRB     R1,[R0, #+0]
//   16   NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
        MOVS     R4,#+0
        STRB     R4,[R0, #+1]
//   17   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        MOVS     R1,#+1
        STRB     R1,[R0, #+2]
//   18   NVIC_Init(&NVIC_InitStructure);
        BL       NVIC_Init
//   19 
//   20   TIM.TIM_Period = 100 - 1;   //Firq = Ftim / 500 = 10000 Hz
        MOVS     R0,#+99
        STR      R0,[SP, #+8]
//   21   TIM.TIM_Prescaler = 48 - 1; //Ftim = 48000000 / 48 = 1000000 Hz
        ADD      R0,SP,#+4
        MOVS     R1,#+47
        STRH     R1,[R0, #+0]
//   22   TIM.TIM_ClockDivision = 0;
        STRH     R4,[R0, #+8]
//   23   TIM.TIM_CounterMode = TIM_CounterMode_Up;
        STRH     R4,[R0, #+2]
//   24   TIM.TIM_RepetitionCounter = 0;
        STRB     R4,[R0, #+10]
//   25   TIM_TimeBaseInit(TIM1, &TIM);
        LDR      R4,??DataTable6  ;; 0x40012c00
        ADD      R1,SP,#+4
        MOVS     R0,R4
        BL       TIM_TimeBaseInit
//   26 
//   27   /* TIM IT enable */
//   28   TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
        MOVS     R2,#+1
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       TIM_ITConfig
//   29 
//   30   /* TIM counter enable */
//   31   TIM_Cmd(TIM1, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       TIM_Cmd
//   32 }
        ADD      SP,SP,#+16
        POP      {R4,PC}          ;; return
//   33 ////////////////////////////////////////////////////////////////////

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//   34 BYTE  Enc_A_Prev = 1;
Enc_A_Prev:
        DATA
        DC8 1
//   35 BYTE  Enc_B_Prev = 1;
Enc_B_Prev:
        DC8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   36 BYTE  SD_State   = 0;
SD_State:
        DS8 1
//   37 #define SD_DIS  GPIO_Pin_1 //PA1 EN
//   38 #define SD_DIR  GPIO_Pin_2 //PA2 DIR
//   39 #define SD_STEP GPIO_Pin_3 //PA3 STEP
//   40 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   41 BYTE Enc_Process()
//   42 {
Enc_Process:
        PUSH     {R3-R5}
//   43   BYTE tmp;
//   44   BYTE A = 1;
//   45   BYTE B = 1;
//   46   BYTE sd_CMD = 0; //nothing to do
        MOVS     R0,#+0
//   47   //PB1, PB2
//   48   tmp = (GPIOB->IDR >> 1) & 0x03;
        LDR      R1,??DataTable6_1  ;; 0x48000410
        LDRH     R1,[R1, #+0]
        LSLS     R1,R1,#+24
        LSRS     R1,R1,#+25
        LSLS     R1,R1,#+30
        LSRS     R1,R1,#+30
//   49   A = tmp & 0x01;
        MOVS     R3,#+1
        ANDS     R3,R3,R1
//   50   B = (tmp >> 1);
        LSRS     R1,R1,#+1
//   51 
//   52   if((Enc_B_Prev == 1) && (B == 0))
        LDR      R4,??DataTable6_2
        LDRB     R5,[R4, #+1]
        CMP      R5,#+1
        BNE      ??Enc_Process_0
        CMP      R1,#+0
        BNE      ??Enc_Process_1
//   53   {
//   54     //1->0
//   55     if(A == 1)
        CMP      R3,#+0
        BEQ      ??Enc_Process_2
        B        ??Enc_Process_3
//   56       sd_CMD = 1;//по часовой
//   57     else
//   58       sd_CMD = 2;//против часовой
//   59   }
//   60   if((Enc_B_Prev == 0) && (B == 1))
??Enc_Process_0:
        CMP      R5,#+0
        BNE      ??Enc_Process_1
        CMP      R1,#+0
        BEQ      ??Enc_Process_1
//   61   {
//   62     //1->0
//   63     if(A == 1)
        CMP      R3,#+0
        BEQ      ??Enc_Process_3
//   64       sd_CMD = 2;//против часовой
??Enc_Process_2:
        MOVS     R0,#+2
        B        ??Enc_Process_1
//   65     else
//   66       sd_CMD = 1;//по часовой
??Enc_Process_3:
        MOVS     R0,#+1
//   67   }
//   68   Enc_A_Prev = A;
??Enc_Process_1:
        STRB     R3,[R4, #+0]
//   69   Enc_B_Prev = B;
        STRB     R1,[R4, #+1]
//   70   return sd_CMD;
        POP      {R1,R4,R5}
        BX       LR               ;; return
//   71 }
//   72 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   73 void Task_1()
//   74 {
//   75   if(SD_State == 0)
Task_1:
        LDR      R0,??DataTable6_3  ;; 0x48000018
        MOVS     R1,#+8
        LDR      R2,??DataTable6_4
        LDRB     R3,[R2, #+0]
        CMP      R3,#+0
        BNE      ??Task_1_0
//   76   {
//   77     SD_State = 1;
        MOVS     R3,#+1
//   78     GPIOA->BSRR = SD_STEP;
        STR      R1,[R0, #+0]
        B        ??Task_1_1
//   79   }
//   80   else
//   81   {
//   82     SD_State = 0;
??Task_1_0:
        MOVS     R3,#+0
//   83     GPIOA->BRR = SD_STEP;
        STRH     R1,[R0, #+16]
??Task_1_1:
        STRB     R3,[R2, #+0]
//   84   }
//   85 }
        BX       LR               ;; return
//   86 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   87 void Task_2(BYTE CMD)
//   88 {
//   89   switch(SD_State)
Task_2:
        LDR      R1,??DataTable6_4
        LDRB     R2,[R1, #+0]
        CMP      R2,#+0
        BEQ      ??Task_2_0
        CMP      R2,#+1
        BEQ      ??Task_2_1
        BX       LR
//   90   {
//   91     case 0://wait CMD to step drive
//   92       if(CMD == 1)
??Task_2_0:
        CMP      R0,#+1
        BNE      ??Task_2_2
//   93       {
//   94         GPIOA->BSRR = SD_DIR;
        LDR      R0,??DataTable6_3  ;; 0x48000018
        MOVS     R2,#+4
        STR      R2,[R0, #+0]
//   95         GPIOA->BSRR = SD_STEP;
        B.N      ??Task_2_3
//   96         SD_State = 1;
//   97       }
//   98       else if(CMD == 2)
??Task_2_2:
        CMP      R0,#+2
        BNE      ??Task_2_4
//   99       {
//  100         GPIOA->BRR = SD_DIR;
        LDR      R0,??DataTable6_3  ;; 0x48000018
        MOVS     R2,#+4
        STRH     R2,[R0, #+16]
//  101         GPIOA->BSRR = SD_STEP;
??Task_2_3:
        MOVS     R2,#+8
        STR      R2,[R0, #+0]
//  102         SD_State = 1;
        MOVS     R0,#+1
        STRB     R0,[R1, #+0]
        BX       LR
//  103       }
//  104       break;
//  105 
//  106     case 1:
//  107       SD_State = 0;
??Task_2_1:
        MOVS     R0,#+0
        STRB     R0,[R1, #+0]
//  108       GPIOA->BRR = SD_STEP;
        LDR      R0,??DataTable6_3  ;; 0x48000018
        MOVS     R1,#+8
        STRH     R1,[R0, #+16]
//  109       break;
//  110   }
//  111 }
??Task_2_4:
        BX       LR               ;; return
//  112 ////////////////////////////////////////////////////////////////////

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  113 SWORD Task_3_pos = 0;
//  114 BYTE  Task_3_cnt = 0;
Task_3_cnt:
        DS8 1
        DS8 1
Task_3_pos:
        DS8 2

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  115 void Task_3(BYTE CMD)
//  116 {
Task_3:
        PUSH     {R3,R4}
//  117   if(CMD == 1)
        LDR      R1,??DataTable6_5
        MOVS     R2,#+2
        LDRSH    R2,[R1, R2]
        CMP      R0,#+1
        BNE      ??Task_3_0
//  118     Task_3_pos++;
        ADDS     R2,R2,#+1
        B        ??Task_3_1
//  119   else if(CMD == 2)
??Task_3_0:
        CMP      R0,#+2
        BNE      ??Task_3_1
//  120     Task_3_pos--;
        SUBS     R2,R2,#+1
??Task_3_1:
        STRH     R2,[R1, #+2]
//  121 
//  122   if(Task_3_pos >= 0)
        LDRB     R2,[R1, #+0]
        MOVS     R0,#+2
        LDRSH    R3,[R1, R0]
        LDR      R0,??DataTable6_3  ;; 0x48000018
        MOVS     R4,#+4
        CMP      R3,#+0
        BMI      ??Task_3_2
//  123   {
//  124     GPIOA->BRR = SD_DIR;
        STRH     R4,[R0, #+16]
//  125     Task_3_cnt += Task_3_pos;
        LDRH     R3,[R1, #+2]
        ADDS     R2,R2,R3
        B        ??Task_3_3
//  126   }
//  127   else
//  128   {
//  129     GPIOA->BSRR = SD_DIR;
??Task_3_2:
        STR      R4,[R0, #+0]
//  130     Task_3_cnt -= Task_3_pos;
        SUBS     R2,R2,R3
??Task_3_3:
        STRB     R2,[R1, #+0]
//  131   }
//  132 
//  133   if(Task_3_cnt & (1<<5))
        MOVS     R2,#+8
        LDRB     R1,[R1, #+0]
        LSLS     R1,R1,#+26
        BPL      ??Task_3_4
//  134     GPIOA->BSRR = SD_STEP;
        STR      R2,[R0, #+0]
        B        ??Task_3_5
//  135   else
//  136     GPIOA->BRR = SD_STEP;
??Task_3_4:
        STRH     R2,[R0, #+16]
//  137 }
??Task_3_5:
        POP      {R0,R4}
        BX       LR               ;; return
//  138 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  139 void TIM1_BRK_UP_TRG_COM_IRQHandler()
//  140 {
TIM1_BRK_UP_TRG_COM_IRQHandler:
        PUSH     {R4,LR}
//  141   if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
        LDR      R4,??DataTable6  ;; 0x40012c00
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       TIM_GetITStatus
        CMP      R0,#+0
        BEQ      ??TIM1_BRK_UP_TRG_COM_IRQHandler_0
//  142   {
//  143     BYTE what_to_do = Enc_Process();
        BL       Enc_Process
//  144 
//  145     //Task_1();
//  146     //Task_2(what_to_do);
//  147     Task_3(what_to_do);
        BL       Task_3
//  148 
//  149     TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       TIM_ClearITPendingBit
//  150   }
//  151 }
??TIM1_BRK_UP_TRG_COM_IRQHandler_0:
        POP      {R4,PC}          ;; return
//  152 ////////////////////////////////////////////////////////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  153 int main()
//  154 {
main:
        PUSH     {R2-R6,LR}
//  155   //¬ключаем тактирование GPIO
//  156   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+192
        LSLS     R0,R0,#+11       ;; #+393216
        BL       RCC_AHBPeriphClockCmd
//  157 
//  158   GPIO_InitTypeDef GPIO_InitStructure;
//  159   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
        MOVS     R0,#+6
        STR      R0,[SP, #+0]
//  160   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        MOVS     R4,#+0
        MOV      R5,SP
        STRH     R4,[R5, #+4]
//  161   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
//  162   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        STRB     R4,[R5, #+6]
//  163   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        MOVS     R6,#+1
        STRB     R6,[R5, #+7]
//  164   GPIO_Init(GPIOB, &GPIO_InitStructure);
        MOV      R1,SP
        LDR      R0,??DataTable6_6  ;; 0x48000400
        BL       GPIO_Init
//  165 
//  166   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
        MOVS     R0,#+14
        STR      R0,[SP, #+0]
//  167   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        STRB     R6,[R5, #+4]
//  168   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
        STRB     R4,[R5, #+5]
//  169   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        STRB     R4,[R5, #+6]
//  170   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        STRB     R6,[R5, #+7]
//  171   GPIO_Init(GPIOA, &GPIO_InitStructure);
        MOVS     R4,#+144
        LSLS     R4,R4,#+23       ;; #+1207959552
        MOV      R1,SP
        MOVS     R0,R4
        BL       GPIO_Init
//  172 
//  173   Timer1_Init();
        BL       Timer1_Init
//  174 
//  175   //GPIOA->BSRR = SD_DIS; //disable step drive
//  176   GPIOA->BRR = SD_DIS;  //enable step drive
        MOVS     R0,#+2
        STRH     R0,[R4, #+40]
//  177 
//  178   GPIOA->BRR = SD_DIR;//task1
        MOVS     R0,#+4
        STRH     R0,[R4, #+40]
//  179 
//  180   while(1)
??main_0:
        B        ??main_0
//  181   {
//  182     //GPIOA->BSRR = GPIO_Pin_1;
//  183     //GPIOA->BRR = GPIO_Pin_1;
//  184     //GPIO_SetBits(GPIOA, GPIO_Pin_1);
//  185     //GPIO_ResetBits(GPIOA, GPIO_Pin_1);
//  186   }
//  187 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6:
        DC32     0x40012c00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_1:
        DC32     0x48000410

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_2:
        DC32     Enc_A_Prev

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_3:
        DC32     0x48000018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_4:
        DC32     SD_State

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_5:
        DC32     Task_3_cnt

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable6_6:
        DC32     0x48000400

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
// 
//   5 bytes in section .bss
//   2 bytes in section .data
// 434 bytes in section .text
// 
// 434 bytes of CODE memory
//   7 bytes of DATA memory
//
//Errors: none
//Warnings: none
