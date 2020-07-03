///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:53 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    tim.c                                                   /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    tim.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER    /
//                    -lB F:\ProjectsE\MCU\FiltTest_2\Release\List\ -o        /
//                    F:\ProjectsE\MCU\FiltTest_2\Release\Obj\                /
//                    --endian=little --cpu=Cortex-M0 -e --fpu=None           /
//                    --dlib_config "F:\Program Files\IAR Systems\Embedded    /
//                    Workbench 6.5\arm\INC\c\DLib_Config_Normal.h" -I        /
//                    F:\ProjectsE\MCU\FiltTest_2\ -I                         /
//                    F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\CMSIS\Device\ST\STM32F0xx\Include\ -I     /
//                    F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\CMSIS\Include\ -I                         /
//                    F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\inc\ -Oh       /
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_tim. /
//                    s                                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_tim

        #define SHT_PROGBITS 0x1

        EXTERN RCC_APB1PeriphResetCmd
        EXTERN RCC_APB2PeriphResetCmd

        PUBLIC TIM_ARRPreloadConfig
        PUBLIC TIM_BDTRConfig
        PUBLIC TIM_BDTRStructInit
        PUBLIC TIM_CCPreloadControl
        PUBLIC TIM_CCxCmd
        PUBLIC TIM_CCxNCmd
        PUBLIC TIM_ClearFlag
        PUBLIC TIM_ClearITPendingBit
        PUBLIC TIM_ClearOC1Ref
        PUBLIC TIM_ClearOC2Ref
        PUBLIC TIM_ClearOC3Ref
        PUBLIC TIM_ClearOC4Ref
        PUBLIC TIM_Cmd
        PUBLIC TIM_CounterModeConfig
        PUBLIC TIM_CtrlPWMOutputs
        PUBLIC TIM_DMACmd
        PUBLIC TIM_DMAConfig
        PUBLIC TIM_DeInit
        PUBLIC TIM_ETRClockMode1Config
        PUBLIC TIM_ETRClockMode2Config
        PUBLIC TIM_ETRConfig
        PUBLIC TIM_EncoderInterfaceConfig
        PUBLIC TIM_ForcedOC1Config
        PUBLIC TIM_ForcedOC2Config
        PUBLIC TIM_ForcedOC3Config
        PUBLIC TIM_ForcedOC4Config
        PUBLIC TIM_GenerateEvent
        PUBLIC TIM_GetCapture1
        PUBLIC TIM_GetCapture2
        PUBLIC TIM_GetCapture3
        PUBLIC TIM_GetCapture4
        PUBLIC TIM_GetCounter
        PUBLIC TIM_GetFlagStatus
        PUBLIC TIM_GetITStatus
        PUBLIC TIM_GetPrescaler
        PUBLIC TIM_ICInit
        PUBLIC TIM_ICStructInit
        PUBLIC TIM_ITConfig
        PUBLIC TIM_ITRxExternalClockConfig
        PUBLIC TIM_InternalClockConfig
        PUBLIC TIM_OC1FastConfig
        PUBLIC TIM_OC1Init
        PUBLIC TIM_OC1NPolarityConfig
        PUBLIC TIM_OC1PolarityConfig
        PUBLIC TIM_OC1PreloadConfig
        PUBLIC TIM_OC2FastConfig
        PUBLIC TIM_OC2Init
        PUBLIC TIM_OC2NPolarityConfig
        PUBLIC TIM_OC2PolarityConfig
        PUBLIC TIM_OC2PreloadConfig
        PUBLIC TIM_OC3FastConfig
        PUBLIC TIM_OC3Init
        PUBLIC TIM_OC3NPolarityConfig
        PUBLIC TIM_OC3PolarityConfig
        PUBLIC TIM_OC3PreloadConfig
        PUBLIC TIM_OC4FastConfig
        PUBLIC TIM_OC4Init
        PUBLIC TIM_OC4PolarityConfig
        PUBLIC TIM_OC4PreloadConfig
        PUBLIC TIM_OCStructInit
        PUBLIC TIM_PWMIConfig
        PUBLIC TIM_PrescalerConfig
        PUBLIC TIM_RemapConfig
        PUBLIC TIM_SelectCCDMA
        PUBLIC TIM_SelectCOM
        PUBLIC TIM_SelectHallSensor
        PUBLIC TIM_SelectInputTrigger
        PUBLIC TIM_SelectMasterSlaveMode
        PUBLIC TIM_SelectOCREFClear
        PUBLIC TIM_SelectOCxM
        PUBLIC TIM_SelectOnePulseMode
        PUBLIC TIM_SelectOutputTrigger
        PUBLIC TIM_SelectSlaveMode
        PUBLIC TIM_SetAutoreload
        PUBLIC TIM_SetClockDivision
        PUBLIC TIM_SetCompare1
        PUBLIC TIM_SetCompare2
        PUBLIC TIM_SetCompare3
        PUBLIC TIM_SetCompare4
        PUBLIC TIM_SetCounter
        PUBLIC TIM_SetIC1Prescaler
        PUBLIC TIM_SetIC2Prescaler
        PUBLIC TIM_SetIC3Prescaler
        PUBLIC TIM_SetIC4Prescaler
        PUBLIC TIM_TIxExternalClockConfig
        PUBLIC TIM_TimeBaseInit
        PUBLIC TIM_TimeBaseStructInit
        PUBLIC TIM_UpdateDisableConfig
        PUBLIC TIM_UpdateRequestConfig

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_tim.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_tim.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides firmware functions to manage the following 
//    8   *          functionalities of the TIM peripheral:
//    9   *            + TimeBase management
//   10   *            + Output Compare management
//   11   *            + Input Capture management
//   12   *            + Interrupts, DMA and flags management
//   13   *            + Clocks management
//   14   *            + Synchronization management
//   15   *            + Specific interface management
//   16   *            + Specific remapping management      
//   17   *              
//   18   *  @verbatim
//   19   
//   20  ===============================================================================
//   21                     ##### How to use this driver #####
//   22  ===============================================================================
//   23     [..] This driver provides functions to configure and program the TIM 
//   24          of all STM32F0xx devices These functions are split in 8 groups: 
//   25          (#) TIM TimeBase management: this group includes all needed functions 
//   26              to configure the TM Timebase unit:
//   27              (++) Set/Get Prescaler.
//   28              (++) Set/Get Autoreload.
//   29              (++) Counter modes configuration.
//   30              (++) Set Clock division.
//   31              (++) Select the One Pulse mode.
//   32              (++) Update Request Configuration.
//   33              (++) Update Disable Configuration.
//   34              (++) Auto-Preload Configuration.
//   35              (++) Enable/Disable the counter.
//   36   
//   37          (#) TIM Output Compare management: this group includes all needed 
//   38              functions to configure the Capture/Compare unit used in Output 
//   39              compare mode: 
//   40              (++) Configure each channel, independently, in Output Compare mode.
//   41              (++) Select the output compare modes.
//   42              (++) Select the Polarities of each channel.
//   43              (++) Set/Get the Capture/Compare register values.
//   44              (++) Select the Output Compare Fast mode. 
//   45              (++) Select the Output Compare Forced mode.  
//   46              (++) Output Compare-Preload Configuration. 
//   47              (++) Clear Output Compare Reference.
//   48              (++) Select the OCREF Clear signal.
//   49              (++) Enable/Disable the Capture/Compare Channels.    
//   50   
//   51          (#) TIM Input Capture management: this group includes all needed 
//   52              functions to configure the Capture/Compare unit used in 
//   53              Input Capture mode:
//   54              (++) Configure each channel in input capture mode.
//   55              (++) Configure Channel1/2 in PWM Input mode.
//   56              (++) Set the Input Capture Prescaler.
//   57              (++) Get the Capture/Compare values.  
//   58              
//   59         (#) Advanced-control timers (TIM1) specific features
//   60             (++) Configures the Break input, dead time, Lock level, the OSSI,
//   61                  the OSSR State and the AOE(automatic output enable)
//   62             (++) Enable/Disable the TIM peripheral Main Outputs
//   63             (++) Select the Commutation event
//   64             (++) Set/Reset the Capture Compare Preload Control bit     
//   65   
//   66          (#) TIM interrupts, DMA and flags management.
//   67              (++) Enable/Disable interrupt sources.
//   68              (++) Get flags status.
//   69              (++) Clear flags/ Pending bits.
//   70              (++) Enable/Disable DMA requests. 
//   71              (++) Configure DMA burst mode.
//   72              (++) Select CaptureCompare DMA request.  
//   73   
//   74          (#) TIM clocks management: this group includes all needed functions 
//   75              to configure the clock controller unit:
//   76              (++) Select internal/External clock.
//   77              (++) Select the external clock mode: ETR(Mode1/Mode2), TIx or ITRx.
//   78   
//   79          (#) TIM synchronization management: this group includes all needed. 
//   80              functions to configure the Synchronization unit:
//   81              (++) Select Input Trigger.  
//   82              (++) Select Output Trigger.  
//   83              (++) Select Master Slave Mode. 
//   84              (++) ETR Configuration when used as external trigger.   
//   85   
//   86          (#) TIM specific interface management, this group includes all 
//   87              needed functions to use the specific TIM interface:
//   88              (++) Encoder Interface Configuration.
//   89              (++) Select Hall Sensor.   
//   90   
//   91          (#) TIM specific remapping management includes the Remapping 
//   92              configuration of specific timers
//   93   
//   94 @endverbatim
//   95   *    
//   96   ******************************************************************************
//   97   * @attention
//   98   *
//   99   * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
//  100   *
//  101   * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//  102   * You may not use this file except in compliance with the License.
//  103   * You may obtain a copy of the License at:
//  104   *
//  105   *        http://www.st.com/software_license_agreement_liberty_v2
//  106   *
//  107   * Unless required by applicable law or agreed to in writing, software 
//  108   * distributed under the License is distributed on an "AS IS" BASIS, 
//  109   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  110   * See the License for the specific language governing permissions and
//  111   * limitations under the License.
//  112   *
//  113   ******************************************************************************
//  114   */
//  115 
//  116 /* Includes ------------------------------------------------------------------*/
//  117 #include "stm32f0xx_tim.h"
//  118 #include "stm32f0xx_rcc.h"
//  119 
//  120 /** @addtogroup STM32F0xx_StdPeriph_Driver
//  121   * @{
//  122   */
//  123 
//  124 /** @defgroup TIM 
//  125   * @brief TIM driver modules
//  126   * @{
//  127   */
//  128 
//  129 /* Private typedef -----------------------------------------------------------*/
//  130 /* Private define ------------------------------------------------------------*/
//  131 
//  132 /* ---------------------- TIM registers bit mask ------------------------ */
//  133 #define SMCR_ETR_MASK               ((uint16_t)0x00FF) 
//  134 #define CCMR_OFFSET                 ((uint16_t)0x0018)
//  135 #define CCER_CCE_SET                ((uint16_t)0x0001)
//  136 #define CCER_CCNE_SET               ((uint16_t)0x0004) 
//  137   
//  138 /* Private macro -------------------------------------------------------------*/
//  139 /* Private variables ---------------------------------------------------------*/
//  140 /* Private function prototypes -----------------------------------------------*/
//  141 
//  142 static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//  143                        uint16_t TIM_ICFilter);
//  144 static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//  145                        uint16_t TIM_ICFilter);
//  146 static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//  147                        uint16_t TIM_ICFilter);
//  148 static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//  149                        uint16_t TIM_ICFilter);
//  150 /* Private functions ---------------------------------------------------------*/
//  151 
//  152 /** @defgroup TIM_Private_Functions
//  153   * @{
//  154   */
//  155 
//  156 /** @defgroup TIM_Group1 TimeBase management functions
//  157  *  @brief   TimeBase management functions 
//  158  *
//  159 @verbatim
//  160  ===============================================================================
//  161                  ##### TimeBase management functions #####
//  162  ===============================================================================
//  163   
//  164         *** TIM Driver: how to use it in Timing(Time base) Mode ***
//  165  ===============================================================================
//  166     [..] To use the Timer in Timing(Time base) mode, the following steps are 
//  167          mandatory:
//  168          (#) Enable TIM clock using 
//  169              RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function.
//  170          (#) Fill the TIM_TimeBaseInitStruct with the desired parameters.
//  171          (#) Call TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStruct) to configure 
//  172              the Time Base unit with the corresponding configuration.
//  173          (#) Enable the NVIC if you need to generate the update interrupt. 
//  174          (#) Enable the corresponding interrupt using the function 
//  175              TIM_ITConfig(TIMx, TIM_IT_Update). 
//  176          (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
//  177     [..]
//  178         (@) All other functions can be used seperatly to modify, if needed,
//  179             a specific feature of the Timer. 
//  180 
//  181 @endverbatim
//  182   * @{
//  183   */
//  184 
//  185 /**
//  186   * @brief  Deinitializes the TIMx peripheral registers to their default reset values.
//  187   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM peripheral.
//  188   * @note   TIM7 is applicable only for STM32F072 devices
//  189   * @note   TIM6 is not applivable for STM32F031 devices.
//  190   * @note   TIM2 is not applicable for STM32F030 devices.    
//  191   * @retval None
//  192   *   
//  193   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  194 void TIM_DeInit(TIM_TypeDef* TIMx)
//  195 {
TIM_DeInit:
        PUSH     {R4,LR}
//  196   /* Check the parameters */
//  197   assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
//  198 
//  199   if (TIMx == TIM1)
        LDR      R1,??DataTable7  ;; 0x40012c00
        CMP      R0,R1
        BNE      ??TIM_DeInit_0
//  200   {
//  201     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
        LSRS     R4,R1,#+19
        B        ??TIM_DeInit_1
//  202     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);  
//  203   }     
//  204   else if (TIMx == TIM2)
??TIM_DeInit_0:
        MOVS     R1,#+128
        LSLS     R1,R1,#+23       ;; #+1073741824
        CMP      R0,R1
        BNE      ??TIM_DeInit_2
//  205   {
//  206     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+1
        BL       RCC_APB1PeriphResetCmd
//  207     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,#+1
        B.N      ??TIM_DeInit_3
//  208   }
//  209   else if (TIMx == TIM3)
??TIM_DeInit_2:
        LDR      R1,??DataTable8  ;; 0x40000400
        CMP      R0,R1
        BNE      ??TIM_DeInit_4
//  210   {
//  211     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+2
        BL       RCC_APB1PeriphResetCmd
//  212     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,#+2
        B.N      ??TIM_DeInit_3
//  213   }
//  214   else if (TIMx == TIM6)
??TIM_DeInit_4:
        LDR      R1,??DataTable8_1  ;; 0x40001000
        CMP      R0,R1
        BNE      ??TIM_DeInit_5
//  215   {
//  216     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+16
        BL       RCC_APB1PeriphResetCmd
//  217     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,#+16
        B.N      ??TIM_DeInit_3
//  218   } 
//  219   else if (TIMx == TIM7)
??TIM_DeInit_5:
        LDR      R1,??DataTable8_2  ;; 0x40001400
        CMP      R0,R1
        BNE      ??TIM_DeInit_6
//  220   {
//  221     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM7, ENABLE);
        MOVS     R1,#+1
        MOVS     R0,#+32
        BL       RCC_APB1PeriphResetCmd
//  222     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM7, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,#+32
        B.N      ??TIM_DeInit_3
//  223   }
//  224   else if (TIMx == TIM14) 
??TIM_DeInit_6:
        LDR      R1,??DataTable8_3  ;; 0x40002000
        CMP      R0,R1
        BNE      ??TIM_DeInit_7
//  225   {       
//  226     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, ENABLE);
        LSRS     R4,R1,#+22
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       RCC_APB1PeriphResetCmd
//  227     RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM14, DISABLE);  
        MOVS     R1,#+0
        MOVS     R0,R4
??TIM_DeInit_3:
        BL       RCC_APB1PeriphResetCmd
        POP      {R4,PC}
//  228   }        
//  229   else if (TIMx == TIM15)
??TIM_DeInit_7:
        LDR      R1,??DataTable8_4  ;; 0x40014000
        CMP      R0,R1
        BNE      ??TIM_DeInit_8
//  230   {
//  231     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM15, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+9        ;; #+65536
        B        ??TIM_DeInit_1
//  232     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM15, DISABLE);
//  233   } 
//  234   else if (TIMx == TIM16)
??TIM_DeInit_8:
        LDR      R1,??DataTable8_5  ;; 0x40014400
        CMP      R0,R1
        BNE      ??TIM_DeInit_9
//  235   {
//  236     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM16, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+10       ;; #+131072
        B        ??TIM_DeInit_1
//  237     RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM16, DISABLE);
//  238   } 
//  239   else
//  240   {
//  241     if (TIMx == TIM17)
??TIM_DeInit_9:
        LDR      R1,??DataTable11  ;; 0x40014800
        CMP      R0,R1
        BNE      ??TIM_DeInit_10
//  242     {
//  243       RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+11       ;; #+262144
??TIM_DeInit_1:
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       RCC_APB2PeriphResetCmd
//  244       RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM17, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,R4
        BL       RCC_APB2PeriphResetCmd
//  245     }  
//  246   }
//  247      
//  248 }
??TIM_DeInit_10:
        POP      {R4,PC}          ;; return
//  249 
//  250 /**
//  251   * @brief  Initializes the TIMx Time Base Unit peripheral according to 
//  252   *         the specified parameters in the TIM_TimeBaseInitStruct.
//  253   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  254   *         peripheral.
//  255   * @note   TIM7 is applicable only for STM32F072 devices
//  256   * @note   TIM6 is not applivable for STM32F031 devices.
//  257   * @note   TIM2 is not applicable for STM32F030 devices.  
//  258   * @param  TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef
//  259   *         structure that contains the configuration information for
//  260   *         the specified TIM peripheral.
//  261   * @retval None
//  262   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  263 void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
//  264 {
TIM_TimeBaseInit:
        PUSH     {R3,R4}
//  265   uint16_t tmpcr1 = 0;
//  266 
//  267   /* Check the parameters */
//  268   assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
//  269   assert_param(IS_TIM_COUNTER_MODE(TIM_TimeBaseInitStruct->TIM_CounterMode));
//  270   assert_param(IS_TIM_CKD_DIV(TIM_TimeBaseInitStruct->TIM_ClockDivision));
//  271 
//  272   tmpcr1 = TIMx->CR1;  
        LDRH     R3,[R0, #+0]
//  273 
//  274   if((TIMx == TIM1) || (TIMx == TIM2) || (TIMx == TIM3))
        LDR      R2,??DataTable7  ;; 0x40012c00
        CMP      R0,R2
        BEQ      ??TIM_TimeBaseInit_0
        MOVS     R4,#+128
        LSLS     R4,R4,#+23       ;; #+1073741824
        CMP      R0,R4
        BEQ      ??TIM_TimeBaseInit_0
        LDR      R4,??DataTable8  ;; 0x40000400
        CMP      R0,R4
        BNE      ??TIM_TimeBaseInit_1
//  275   {
//  276     /* Select the Counter Mode */
//  277     tmpcr1 &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR | TIM_CR1_CMS)));
//  278     tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_CounterMode;
??TIM_TimeBaseInit_0:
        LDR      R4,??DataTable9  ;; 0xff8f
        ANDS     R4,R4,R3
        LDRH     R3,[R1, #+2]
        ORRS     R3,R3,R4
//  279   }
//  280  
//  281   if(TIMx != TIM6)
??TIM_TimeBaseInit_1:
        LDR      R4,??DataTable8_1  ;; 0x40001000
        CMP      R0,R4
        BEQ      ??TIM_TimeBaseInit_2
//  282   {
//  283     /* Set the clock division */
//  284     tmpcr1 &= (uint16_t)(~((uint16_t)TIM_CR1_CKD));
//  285     tmpcr1 |= (uint32_t)TIM_TimeBaseInitStruct->TIM_ClockDivision;
        LDR      R4,??DataTable9_1  ;; 0xfcff
        ANDS     R4,R4,R3
        LDRH     R3,[R1, #+8]
        ORRS     R3,R3,R4
//  286   }
//  287 
//  288   TIMx->CR1 = tmpcr1;
??TIM_TimeBaseInit_2:
        STRH     R3,[R0, #+0]
//  289 
//  290   /* Set the Autoreload value */
//  291   TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period ;
        LDR      R3,[R1, #+4]
        STR      R3,[R0, #+44]
//  292  
//  293   /* Set the Prescaler value */
//  294   TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;
        LDRH     R3,[R1, #+0]
        STRH     R3,[R0, #+40]
//  295     
//  296   if ((TIMx == TIM1) || (TIMx == TIM15)|| (TIMx == TIM16) || (TIMx == TIM17))  
        CMP      R0,R2
        BEQ      ??TIM_TimeBaseInit_3
        LDR      R2,??DataTable8_4  ;; 0x40014000
        CMP      R0,R2
        BEQ      ??TIM_TimeBaseInit_3
        LDR      R2,??DataTable8_5  ;; 0x40014400
        CMP      R0,R2
        BEQ      ??TIM_TimeBaseInit_3
        LDR      R2,??DataTable11  ;; 0x40014800
        CMP      R0,R2
        BNE      ??TIM_TimeBaseInit_4
//  297   {
//  298     /* Set the Repetition Counter value */
//  299     TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
??TIM_TimeBaseInit_3:
        LDRB     R1,[R1, #+10]
        STRH     R1,[R0, #+48]
//  300   }
//  301 
//  302   /* Generate an update event to reload the Prescaler and the Repetition counter
//  303      values immediately */
//  304   TIMx->EGR = TIM_PSCReloadMode_Immediate;           
??TIM_TimeBaseInit_4:
        MOVS     R1,#+1
        STRH     R1,[R0, #+20]
//  305 }
        POP      {R0,R4}
        BX       LR               ;; return
//  306 
//  307 /**
//  308   * @brief  Fills each TIM_TimeBaseInitStruct member with its default value.
//  309   * @param  TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef structure
//  310   *         which will be initialized.
//  311   * @retval None
//  312   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  313 void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
//  314 {
//  315   /* Set the default configuration */
//  316   TIM_TimeBaseInitStruct->TIM_Period = 0xFFFFFFFF;
TIM_TimeBaseStructInit:
        MOVS     R1,#+0
        MVNS     R1,R1            ;; #-1
        STR      R1,[R0, #+4]
//  317   TIM_TimeBaseInitStruct->TIM_Prescaler = 0x0000;
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
//  318   TIM_TimeBaseInitStruct->TIM_ClockDivision = TIM_CKD_DIV1;
        STRH     R1,[R0, #+8]
//  319   TIM_TimeBaseInitStruct->TIM_CounterMode = TIM_CounterMode_Up;
        STRH     R1,[R0, #+2]
//  320   TIM_TimeBaseInitStruct->TIM_RepetitionCounter = 0x0000;
        STRB     R1,[R0, #+10]
//  321 }
        BX       LR               ;; return
//  322 
//  323 /**
//  324   * @brief  Configures the TIMx Prescaler.
//  325   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM peripheral.
//  326   * @note   TIM7 is applicable only for STM32F072 devices
//  327   * @note   TIM6 is not applivable for STM32F031 devices.
//  328   * @note   TIM2 is not applicable for STM32F030 devices.    
//  329   * @param  Prescaler: specifies the Prescaler Register value
//  330   * @param  TIM_PSCReloadMode: specifies the TIM Prescaler Reload mode
//  331   *          This parameter can be one of the following values:
//  332   *            @arg TIM_PSCReloadMode_Update: The Prescaler is loaded at the update event.
//  333   *            @arg TIM_PSCReloadMode_Immediate: The Prescaler is loaded immediatly.
//  334   * @retval None
//  335   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  336 void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode)
//  337 {
//  338   /* Check the parameters */
//  339   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  340   assert_param(IS_TIM_PRESCALER_RELOAD(TIM_PSCReloadMode));
//  341   
//  342   /* Set the Prescaler value */
//  343   TIMx->PSC = Prescaler;
TIM_PrescalerConfig:
        STRH     R1,[R0, #+40]
//  344   /* Set or reset the UG Bit */
//  345   TIMx->EGR = TIM_PSCReloadMode;
        STRH     R2,[R0, #+20]
//  346 }
        BX       LR               ;; return
//  347 
//  348 /**
//  349   * @brief  Specifies the TIMx Counter Mode to be used.
//  350   * @param  TIMx: where x can be 1, 2, or 3 to select the TIM peripheral.
//  351   * @note   TIM2 is not applicable for STM32F030 devices.  
//  352   * @param  TIM_CounterMode: specifies the Counter Mode to be used
//  353   *          This parameter can be one of the following values:
//  354   *            @arg TIM_CounterMode_Up: TIM Up Counting Mode
//  355   *            @arg TIM_CounterMode_Down: TIM Down Counting Mode
//  356   *            @arg TIM_CounterMode_CenterAligned1: TIM Center Aligned Mode1
//  357   *            @arg TIM_CounterMode_CenterAligned2: TIM Center Aligned Mode2
//  358   *            @arg TIM_CounterMode_CenterAligned3: TIM Center Aligned Mode3
//  359   * @retval None
//  360   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  361 void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode)
//  362 {
//  363   uint16_t tmpcr1 = 0;
//  364   
//  365   /* Check the parameters */
//  366   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
//  367   assert_param(IS_TIM_COUNTER_MODE(TIM_CounterMode));
//  368   
//  369   tmpcr1 = TIMx->CR1;
//  370   /* Reset the CMS and DIR Bits */
//  371   tmpcr1 &= (uint16_t)(~((uint16_t)(TIM_CR1_DIR | TIM_CR1_CMS)));
//  372   /* Set the Counter Mode */
//  373   tmpcr1 |= TIM_CounterMode;
//  374   /* Write to TIMx CR1 register */
//  375   TIMx->CR1 = tmpcr1;
TIM_CounterModeConfig:
        LDRH     R2,[R0, #+0]
        LDR      R3,??DataTable9  ;; 0xff8f
        ANDS     R3,R3,R2
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+0]
//  376 }
        BX       LR               ;; return
//  377 
//  378 /**
//  379   * @brief  Sets the TIMx Counter Register value
//  380   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  381   *          peripheral.
//  382   * @note   TIM7 is applicable only for STM32F072 devices
//  383   * @note   TIM6 is not applivable for STM32F031 devices.
//  384   * @note   TIM2 is not applicable for STM32F030 devices.    
//  385   * @param  Counter: specifies the Counter register new value.
//  386   * @retval None
//  387   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  388 void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter)
//  389 {
//  390   /* Check the parameters */
//  391    assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  392    
//  393   /* Set the Counter Register value */
//  394   TIMx->CNT = Counter;
TIM_SetCounter:
        STR      R1,[R0, #+36]
//  395 }
        BX       LR               ;; return
//  396 
//  397 /**
//  398   * @brief  Sets the TIMx Autoreload Register value
//  399   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM peripheral.
//  400   * @note   TIM7 is applicable only for STM32F072 devices
//  401   * @note   TIM6 is not applivable for STM32F031 devices.
//  402   * @note   TIM2 is not applicable for STM32F030 devices.    
//  403   * @param  Autoreload: specifies the Autoreload register new value.
//  404   * @retval None
//  405   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  406 void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload)
//  407 {
//  408   /* Check the parameters */
//  409   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  410   
//  411   /* Set the Autoreload Register value */
//  412   TIMx->ARR = Autoreload;
TIM_SetAutoreload:
        STR      R1,[R0, #+44]
//  413 }
        BX       LR               ;; return
//  414 
//  415 /**
//  416   * @brief  Gets the TIMx Counter value.
//  417   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  418   *         peripheral.
//  419   * @note   TIM7 is applicable only for STM32F072 devices
//  420   * @note   TIM6 is not applivable for STM32F031 devices.
//  421   * @note   TIM2 is not applicable for STM32F030 devices.    
//  422   * @retval Counter Register value.
//  423   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  424 uint32_t TIM_GetCounter(TIM_TypeDef* TIMx)
//  425 {
//  426   /* Check the parameters */
//  427   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  428   
//  429   /* Get the Counter Register value */
//  430   return TIMx->CNT;
TIM_GetCounter:
        LDR      R0,[R0, #+36]
        BX       LR               ;; return
//  431 }
//  432 
//  433 /**
//  434   * @brief  Gets the TIMx Prescaler value.
//  435   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  436   *         peripheral.
//  437   * @note   TIM7 is applicable only for STM32F072 devices
//  438   * @note   TIM6 is not applivable for STM32F031 devices.
//  439   * @note   TIM2 is not applicable for STM32F030 devices.    
//  440   * @retval Prescaler Register value.
//  441   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  442 uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx)
//  443 {
//  444   /* Check the parameters */
//  445   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  446   
//  447   /* Get the Prescaler Register value */
//  448   return TIMx->PSC;
TIM_GetPrescaler:
        LDRH     R0,[R0, #+40]
        BX       LR               ;; return
//  449 }
//  450 
//  451 /**
//  452   * @brief  Enables or Disables the TIMx Update event.
//  453   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  454   *         peripheral.
//  455   * @note   TIM7 is applicable only for STM32F072 devices
//  456   * @note   TIM6 is not applivable for STM32F031 devices.
//  457   * @note   TIM2 is not applicable for STM32F030 devices.    
//  458   * @param  NewState: new state of the TIMx UDIS bit
//  459   *          This parameter can be: ENABLE or DISABLE.
//  460   * @retval None
//  461   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  462 void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
//  463 {
//  464   /* Check the parameters */
//  465   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  466   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  467   
//  468   if (NewState != DISABLE)
TIM_UpdateDisableConfig:
        CMP      R1,#+0
        LDRH     R1,[R0, #+0]
        BEQ      ??TIM_UpdateDisableConfig_0
//  469   {
//  470     /* Set the Update Disable Bit */
//  471     TIMx->CR1 |= TIM_CR1_UDIS;
        MOVS     R2,#+2
        ORRS     R2,R2,R1
        B        ??TIM_UpdateDisableConfig_1
//  472   }
//  473   else
//  474   {
//  475     /* Reset the Update Disable Bit */
//  476     TIMx->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_UDIS);
??TIM_UpdateDisableConfig_0:
        LDR      R2,??DataTable18  ;; 0xfffd
        ANDS     R2,R2,R1
??TIM_UpdateDisableConfig_1:
        STRH     R2,[R0, #+0]
//  477   }
//  478 }
        BX       LR               ;; return
//  479 
//  480 /**
//  481   * @brief  Configures the TIMx Update Request Interrupt source.
//  482   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  483   *         peripheral.
//  484   * @note   TIM7 is applicable only for STM32F072 devices
//  485   * @note   TIM6 is not applivable for STM32F031 devices.
//  486   * @note   TIM2 is not applicable for STM32F030 devices.    
//  487   * @param  TIM_UpdateSource: specifies the Update source.
//  488   *          This parameter can be one of the following values:
//  489   *            @arg TIM_UpdateSource_Regular: Source of update is the counter
//  490   *                 overflow/underflow or the setting of UG bit, or an update
//  491   *                 generation through the slave mode controller.
//  492   *            @arg TIM_UpdateSource_Global: Source of update is counter overflow/underflow.
//  493   * @retval None
//  494   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  495 void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource)
//  496 {
//  497   /* Check the parameters */
//  498   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  499   assert_param(IS_TIM_UPDATE_SOURCE(TIM_UpdateSource));
//  500   
//  501   if (TIM_UpdateSource != TIM_UpdateSource_Global)
TIM_UpdateRequestConfig:
        CMP      R1,#+0
        LDRH     R1,[R0, #+0]
        BEQ      ??TIM_UpdateRequestConfig_0
//  502   {
//  503     /* Set the URS Bit */
//  504     TIMx->CR1 |= TIM_CR1_URS;
        MOVS     R2,#+4
        ORRS     R2,R2,R1
        B        ??TIM_UpdateRequestConfig_1
//  505   }
//  506   else
//  507   {
//  508     /* Reset the URS Bit */
//  509     TIMx->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_URS);
??TIM_UpdateRequestConfig_0:
        LDR      R2,??DataTable19  ;; 0xfffb
        ANDS     R2,R2,R1
??TIM_UpdateRequestConfig_1:
        STRH     R2,[R0, #+0]
//  510   }
//  511 }
        BX       LR               ;; return
//  512 
//  513 /**
//  514   * @brief  Enables or disables TIMx peripheral Preload register on ARR.
//  515   * @param  TIMx: where x can be  1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  516   *         peripheral.
//  517   * @note   TIM7 is applicable only for STM32F072 devices
//  518   * @note   TIM6 is not applivable for STM32F031 devices.
//  519   * @note   TIM2 is not applicable for STM32F030 devices.  
//  520   * @param  NewState: new state of the TIMx peripheral Preload register
//  521   *          This parameter can be: ENABLE or DISABLE.
//  522   * @retval None
//  523   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  524 void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
//  525 {
//  526   /* Check the parameters */
//  527   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  528   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  529   
//  530   if (NewState != DISABLE)
TIM_ARRPreloadConfig:
        CMP      R1,#+0
        LDRH     R1,[R0, #+0]
        BEQ      ??TIM_ARRPreloadConfig_0
//  531   {
//  532     /* Set the ARR Preload Bit */
//  533     TIMx->CR1 |= TIM_CR1_ARPE;
        MOVS     R2,#+128
        ORRS     R2,R2,R1
        B        ??TIM_ARRPreloadConfig_1
//  534   }
//  535   else
//  536   {
//  537     /* Reset the ARR Preload Bit */
//  538     TIMx->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_ARPE);
??TIM_ARRPreloadConfig_0:
        LDR      R2,??DataTable20  ;; 0xff7f
        ANDS     R2,R2,R1
??TIM_ARRPreloadConfig_1:
        STRH     R2,[R0, #+0]
//  539   }
//  540 }
        BX       LR               ;; return
//  541 
//  542 /**
//  543   * @brief  Selects the TIMx's One Pulse Mode.
//  544   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17 to select the TIM 
//  545   *         peripheral.
//  546   * @note   TIM7 is applicable only for STM32F072 devices
//  547   * @note   TIM6 is not applivable for STM32F031 devices.
//  548   * @note   TIM2 is not applicable for STM32F030 devices.    
//  549   * @param  TIM_OPMode: specifies the OPM Mode to be used.
//  550   *          This parameter can be one of the following values:
//  551   *            @arg TIM_OPMode_Single
//  552   *            @arg TIM_OPMode_Repetitive
//  553   * @retval None
//  554   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  555 void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode)
//  556 {
//  557   /* Check the parameters */
//  558   assert_param(IS_TIM_ALL_PERIPH(TIMx));
//  559   assert_param(IS_TIM_OPM_MODE(TIM_OPMode));
//  560   
//  561   /* Reset the OPM Bit */
//  562   TIMx->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_OPM);
TIM_SelectOnePulseMode:
        LDRH     R2,[R0, #+0]
        LDR      R3,??DataTable21  ;; 0xfff7
        B.N      ?Subroutine1
//  563   /* Configure the OPM Mode */
//  564   TIMx->CR1 |= TIM_OPMode;
//  565 }
//  566 
//  567 /**
//  568   * @brief  Sets the TIMx Clock Division value.
//  569   * @param  TIMx: where x can be  1, 2, 3, 14, 15, 16 and 17 to select the TIM peripheral.
//  570   * @note   TIM2 is not applicable for STM32F030 devices.  
//  571   * @param  TIM_CKD: specifies the clock division value.
//  572   *          This parameter can be one of the following value:
//  573   *            @arg TIM_CKD_DIV1: TDTS = Tck_tim
//  574   *            @arg TIM_CKD_DIV2: TDTS = 2*Tck_tim
//  575   *            @arg TIM_CKD_DIV4: TDTS = 4*Tck_tim
//  576   * @retval None
//  577   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  578 void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD)
//  579 {
//  580   /* Check the parameters */
//  581   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
//  582   assert_param(IS_TIM_CKD_DIV(TIM_CKD));
//  583   
//  584   /* Reset the CKD Bits */
//  585   TIMx->CR1 &= (uint16_t)~((uint16_t)TIM_CR1_CKD);
TIM_SetClockDivision:
        LDRH     R2,[R0, #+0]
        LDR      R3,??DataTable9_1  ;; 0xfcff
        REQUIRE ?Subroutine1
        ;; // Fall through to label ?Subroutine1
//  586   /* Set the CKD value */
//  587   TIMx->CR1 |= TIM_CKD;
//  588 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine1:
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+0]
        LDRH     R2,[R0, #+0]
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+0]
        BX       LR               ;; return
//  589 
//  590 /**
//  591   * @brief  Enables or disables the specified TIM peripheral.
//  592   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 and 17to select the TIMx
//  593   *         peripheral.
//  594   * @note   TIM7 is applicable only for STM32F072 devices
//  595   * @note   TIM6 is not applivable for STM32F031 devices.
//  596   * @note   TIM2 is not applicable for STM32F030 devices.    
//  597   * @param  NewState: new state of the TIMx peripheral.
//  598   *          This parameter can be: ENABLE or DISABLE.
//  599   * @retval None
//  600   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  601 void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
//  602 {
//  603   /* Check the parameters */
//  604   assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
//  605   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  606   
//  607   if (NewState != DISABLE)
TIM_Cmd:
        CMP      R1,#+0
        LDRH     R1,[R0, #+0]
        BEQ      ??TIM_Cmd_0
//  608   {
//  609     /* Enable the TIM Counter */
//  610     TIMx->CR1 |= TIM_CR1_CEN;
        MOVS     R2,#+1
        ORRS     R2,R2,R1
        B        ??TIM_Cmd_1
//  611   }
//  612   else
//  613   {
//  614     /* Disable the TIM Counter */
//  615     TIMx->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
??TIM_Cmd_0:
        LDR      R2,??DataTable18_1  ;; 0xfffe
        ANDS     R2,R2,R1
??TIM_Cmd_1:
        STRH     R2,[R0, #+0]
//  616   }
//  617 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable7:
        DC32     0x40012c00
//  618 
//  619 /**
//  620   * @}
//  621   */
//  622 
//  623 /** @defgroup TIM_Group2 Advanced-control timers (TIM1) specific features
//  624  *  @brief   Advanced-control timers (TIM1) specific features
//  625  *
//  626 @verbatim   
//  627  ===============================================================================
//  628       ##### Advanced-control timers (TIM1) specific features #####
//  629  ===============================================================================  
//  630   
//  631        ===================================================================      
//  632               *** TIM Driver: how to use the Break feature ***
//  633        =================================================================== 
//  634        [..] After configuring the Timer channel(s) in the appropriate Output Compare mode: 
//  635                          
//  636            (#) Fill the TIM_BDTRInitStruct with the desired parameters for the Timer
//  637                Break Polarity, dead time, Lock level, the OSSI/OSSR State and the 
//  638                AOE(automatic output enable).
//  639                
//  640            (#) Call TIM_BDTRConfig(TIMx, &TIM_BDTRInitStruct) to configure the Timer
//  641           
//  642            (#) Enable the Main Output using TIM_CtrlPWMOutputs(TIM1, ENABLE) 
//  643           
//  644            (#) Once the break even occurs, the Timer's output signals are put in reset
//  645                state or in a known state (according to the configuration made in
//  646                TIM_BDTRConfig() function).
//  647 
//  648 @endverbatim
//  649   * @{
//  650   */
//  651 /**
//  652   * @brief  Configures the: Break feature, dead time, Lock level, OSSI/OSSR State
//  653   *         and the AOE(automatic output enable).
//  654   * @param  TIMx: where x can be  1, 15, 16 or 17 to select the TIM 
//  655   * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef structure that
//  656   *         contains the BDTR Register configuration  information for the TIM peripheral.
//  657   * @retval None
//  658   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  659 void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct)
//  660 {
//  661   /* Check the parameters */
//  662   assert_param(IS_TIM_LIST2_PERIPH(TIMx));
//  663   assert_param(IS_TIM_OSSR_STATE(TIM_BDTRInitStruct->TIM_OSSRState));
//  664   assert_param(IS_TIM_OSSI_STATE(TIM_BDTRInitStruct->TIM_OSSIState));
//  665   assert_param(IS_TIM_LOCK_LEVEL(TIM_BDTRInitStruct->TIM_LOCKLevel));
//  666   assert_param(IS_TIM_BREAK_STATE(TIM_BDTRInitStruct->TIM_Break));
//  667   assert_param(IS_TIM_BREAK_POLARITY(TIM_BDTRInitStruct->TIM_BreakPolarity));
//  668   assert_param(IS_TIM_AUTOMATIC_OUTPUT_STATE(TIM_BDTRInitStruct->TIM_AutomaticOutput));
//  669   /* Set the Lock level, the Break enable Bit and the Ploarity, the OSSR State,
//  670      the OSSI State, the dead time value and the Automatic Output Enable Bit */
//  671   TIMx->BDTR = (uint32_t)TIM_BDTRInitStruct->TIM_OSSRState | TIM_BDTRInitStruct->TIM_OSSIState |
//  672              TIM_BDTRInitStruct->TIM_LOCKLevel | TIM_BDTRInitStruct->TIM_DeadTime |
//  673              TIM_BDTRInitStruct->TIM_Break | TIM_BDTRInitStruct->TIM_BreakPolarity |
//  674              TIM_BDTRInitStruct->TIM_AutomaticOutput;
TIM_BDTRConfig:
        LDRH     R2,[R1, #+0]
        LDRH     R3,[R1, #+2]
        ORRS     R3,R3,R2
        LDRH     R2,[R1, #+4]
        ORRS     R2,R2,R3
        LDRH     R3,[R1, #+6]
        ORRS     R3,R3,R2
        LDRH     R2,[R1, #+8]
        ORRS     R2,R2,R3
        LDRH     R3,[R1, #+10]
        ORRS     R3,R3,R2
        LDRH     R1,[R1, #+12]
        ORRS     R1,R1,R3
        ADDS     R0,R0,#+68
        STRH     R1,[R0, #+0]
//  675 }
        BX       LR               ;; return
//  676 
//  677 /**
//  678   * @brief  Fills each TIM_BDTRInitStruct member with its default value.
//  679   * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef structure which
//  680   *         will be initialized.
//  681   * @retval None
//  682   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  683 void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct)
//  684 {
//  685   /* Set the default configuration */
//  686   TIM_BDTRInitStruct->TIM_OSSRState = TIM_OSSRState_Disable;
TIM_BDTRStructInit:
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
//  687   TIM_BDTRInitStruct->TIM_OSSIState = TIM_OSSIState_Disable;
        STRH     R1,[R0, #+2]
//  688   TIM_BDTRInitStruct->TIM_LOCKLevel = TIM_LOCKLevel_OFF;
        STRH     R1,[R0, #+4]
//  689   TIM_BDTRInitStruct->TIM_DeadTime = 0x00;
        STRH     R1,[R0, #+6]
//  690   TIM_BDTRInitStruct->TIM_Break = TIM_Break_Disable;
        STRH     R1,[R0, #+8]
//  691   TIM_BDTRInitStruct->TIM_BreakPolarity = TIM_BreakPolarity_Low;
        STRH     R1,[R0, #+10]
//  692   TIM_BDTRInitStruct->TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
        STRH     R1,[R0, #+12]
//  693 }
        BX       LR               ;; return
//  694 
//  695 /**
//  696   * @brief  Enables or disables the TIM peripheral Main Outputs.
//  697   * @param  TIMx: where x can be 1, 15, 16 or 17 to select the TIMx peripheral.
//  698   * @param  NewState: new state of the TIM peripheral Main Outputs.
//  699   *          This parameter can be: ENABLE or DISABLE.
//  700   * @retval None
//  701   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  702 void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState)
//  703 {
//  704   /* Check the parameters */
//  705   assert_param(IS_TIM_LIST2_PERIPH(TIMx));
//  706   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  707   if (NewState != DISABLE)
TIM_CtrlPWMOutputs:
        CMP      R1,#+0
        BEQ      ??TIM_CtrlPWMOutputs_0
//  708   {
//  709     /* Enable the TIM Main Output */
//  710     TIMx->BDTR |= TIM_BDTR_MOE;
        MOVS     R1,#+68
        LDRH     R1,[R0, R1]
        MOVS     R2,#+128
        LSLS     R2,R2,#+8        ;; #+32768
        ORRS     R2,R2,R1
        ADDS     R0,R0,#+68
        STRH     R2,[R0, #+0]
        BX       LR
//  711   }
//  712   else
//  713   {
//  714     /* Disable the TIM Main Output */
//  715     TIMx->BDTR &= (uint16_t)(~((uint16_t)TIM_BDTR_MOE));
??TIM_CtrlPWMOutputs_0:
        MOVS     R1,#+68
        LDRH     R1,[R0, R1]
        LSLS     R1,R1,#+17
        LSRS     R1,R1,#+17
        ADDS     R0,R0,#+68
        STRH     R1,[R0, #+0]
//  716   }  
//  717 }
        BX       LR               ;; return
//  718 
//  719 /**
//  720   * @}
//  721   */
//  722 
//  723 /** @defgroup TIM_Group3 Output Compare management functions
//  724  *  @brief    Output Compare management functions 
//  725  *
//  726 @verbatim
//  727  ===============================================================================
//  728                 ##### Output Compare management functions #####
//  729  ===============================================================================
//  730         *** TIM Driver: how to use it in Output Compare Mode ***
//  731  ===============================================================================
//  732     [..] To use the Timer in Output Compare mode, the following steps are mandatory:
//  733          (#) Enable TIM clock using 
//  734              RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function.
//  735          (#) Configure the TIM pins by configuring the corresponding GPIO pins
//  736          (#) Configure the Time base unit as described in the first part of this 
//  737              driver, if needed, else the Timer will run with the default 
//  738              configuration:
//  739              (++) Autoreload value = 0xFFFF.
//  740              (++) Prescaler value = 0x0000.
//  741              (++) Counter mode = Up counting.
//  742              (++) Clock Division = TIM_CKD_DIV1.
//  743          (#) Fill the TIM_OCInitStruct with the desired parameters including:
//  744              (++) The TIM Output Compare mode: TIM_OCMode.
//  745              (++) TIM Output State: TIM_OutputState.
//  746              (++) TIM Pulse value: TIM_Pulse.
//  747              (++) TIM Output Compare Polarity : TIM_OCPolarity.
//  748          (#) Call TIM_OCxInit(TIMx, &TIM_OCInitStruct) to configure the desired 
//  749              channel with the corresponding configuration.
//  750          (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
//  751     [..]
//  752         (@) All other functions can be used separately to modify, if needed,
//  753           a specific feature of the Timer.
//  754         (@) In case of PWM mode, this function is mandatory:
//  755             TIM_OCxPreloadConfig(TIMx, TIM_OCPreload_ENABLE).
//  756         (@) If the corresponding interrupt or DMA request are needed, the user should:
//  757             (#@) Enable the NVIC (or the DMA) to use the TIM interrupts (or DMA requests).
//  758             (#@) Enable the corresponding interrupt (or DMA request) using the function
//  759                  TIM_ITConfig(TIMx, TIM_IT_CCx) (or TIM_DMA_Cmd(TIMx, TIM_DMA_CCx)).
//  760 
//  761 @endverbatim
//  762   * @{
//  763   */
//  764 
//  765 /**
//  766   * @brief  Initializes the TIMx Channel1 according to the specified
//  767   *         parameters in the TIM_OCInitStruct.
//  768   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 and 17 to select the TIM peripheral.
//  769   * @note   TIM2 is not applicable for STM32F030 devices.  
//  770   * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
//  771   *         that contains the configuration information for the specified TIM 
//  772   *         peripheral.
//  773   * @retval None
//  774   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  775 void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
//  776 {
TIM_OC1Init:
        PUSH     {R3-R6}
//  777   uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
//  778    
//  779   /* Check the parameters */
//  780   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
//  781   assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
//  782   assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
//  783   assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   
//  784  /* Disable the Channel 1: Reset the CC1E Bit */
//  785   TIMx->CCER &= (uint16_t)(~(uint16_t)TIM_CCER_CC1E);
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable18_1  ;; 0xfffe
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+32]
//  786   /* Get the TIMx CCER register value */
//  787   tmpccer = TIMx->CCER;
        LDRH     R4,[R0, #+32]
//  788   /* Get the TIMx CR2 register value */
//  789   tmpcr2 =  TIMx->CR2;
        LDRH     R2,[R0, #+4]
//  790   
//  791   /* Get the TIMx CCMR1 register value */
//  792   tmpccmrx = TIMx->CCMR1;
//  793     
//  794   /* Reset the Output Compare Mode Bits */
//  795   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR1_OC1M));
//  796   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR1_CC1S));
//  797 
//  798   /* Select the Output Compare Mode */
//  799   tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
        LDRH     R3,[R0, #+24]
        LDR      R5,??DataTable18_2  ;; 0xff8c
        ANDS     R5,R5,R3
        LDRH     R3,[R1, #+0]
        ORRS     R3,R3,R5
//  800   
//  801   /* Reset the Output Polarity level */
//  802   tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC1P));
//  803   /* Set the Output Compare Polarity */
//  804   tmpccer |= TIM_OCInitStruct->TIM_OCPolarity;
//  805   
//  806   /* Set the Output State */
//  807   tmpccer |= TIM_OCInitStruct->TIM_OutputState;
        LDR      R5,??DataTable18  ;; 0xfffd
        ANDS     R5,R5,R4
        LDRH     R6,[R1, #+12]
        ORRS     R6,R6,R5
        LDRH     R4,[R1, #+2]
        ORRS     R4,R4,R6
//  808     
//  809   if((TIMx == TIM1) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
        LDR      R5,??DataTable19_1  ;; 0x40012c00
        CMP      R0,R5
        BEQ      ??TIM_OC1Init_0
        LDR      R5,??DataTable8_4  ;; 0x40014000
        CMP      R0,R5
        BEQ      ??TIM_OC1Init_0
        LDR      R5,??DataTable8_5  ;; 0x40014400
        CMP      R0,R5
        BEQ      ??TIM_OC1Init_0
        LDR      R5,??DataTable11  ;; 0x40014800
        CMP      R0,R5
        BNE      ??TIM_OC1Init_1
//  810   {
//  811     assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
//  812     assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
//  813     assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
//  814     assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
//  815     
//  816     /* Reset the Output N Polarity level */
//  817     tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC1NP));
//  818     /* Set the Output N Polarity */
//  819     tmpccer |= TIM_OCInitStruct->TIM_OCNPolarity;
//  820     
//  821     /* Reset the Output N State */
//  822     tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC1NE));    
//  823     /* Set the Output N State */
//  824     tmpccer |= TIM_OCInitStruct->TIM_OutputNState;
??TIM_OC1Init_0:
        LDR      R5,??DataTable21  ;; 0xfff7
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+14]
        ORRS     R4,R4,R5
        LDR      R5,??DataTable19  ;; 0xfffb
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+4]
        ORRS     R4,R4,R5
//  825     
//  826     /* Reset the Ouput Compare and Output Compare N IDLE State */
//  827     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS1));
//  828     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS1N));
//  829     
//  830     /* Set the Output Idle state */
//  831     tmpcr2 |= TIM_OCInitStruct->TIM_OCIdleState;
//  832     /* Set the Output N Idle state */
//  833     tmpcr2 |= TIM_OCInitStruct->TIM_OCNIdleState;
        LDR      R5,??DataTable9_1  ;; 0xfcff
        ANDS     R5,R5,R2
        LDRH     R6,[R1, #+16]
        ORRS     R6,R6,R5
        LDRH     R2,[R1, #+18]
        ORRS     R2,R2,R6
//  834   }
//  835   /* Write to TIMx CR2 */
//  836   TIMx->CR2 = tmpcr2;
??TIM_OC1Init_1:
        STRH     R2,[R0, #+4]
//  837   
//  838   /* Write to TIMx CCMR1 */
//  839   TIMx->CCMR1 = tmpccmrx;
        STRH     R3,[R0, #+24]
//  840 
//  841   /* Set the Capture Compare Register value */
//  842   TIMx->CCR1 = TIM_OCInitStruct->TIM_Pulse; 
        LDR      R1,[R1, #+8]
        STR      R1,[R0, #+52]
//  843  
//  844   /* Write to TIMx CCER */
//  845   TIMx->CCER = tmpccer;
        STRH     R4,[R0, #+32]
//  846 }
        POP      {R0,R4-R6}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8:
        DC32     0x40000400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_1:
        DC32     0x40001000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_2:
        DC32     0x40001400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_3:
        DC32     0x40002000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_4:
        DC32     0x40014000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_5:
        DC32     0x40014400
//  847 
//  848 /**
//  849   * @brief  Initializes the TIMx Channel2 according to the specified
//  850   *         parameters in the TIM_OCInitStruct.
//  851   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
//  852   * @note   TIM2 is not applicable for STM32F030 devices.  
//  853   * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
//  854   *         that contains the configuration information for the specified TIM 
//  855   *         peripheral.
//  856   * @retval None
//  857   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  858 void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
//  859 {
TIM_OC2Init:
        PUSH     {R3-R6}
//  860   uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
//  861    
//  862   /* Check the parameters */
//  863   assert_param(IS_TIM_LIST6_PERIPH(TIMx)); 
//  864   assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
//  865   assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
//  866   assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   
//  867    /* Disable the Channel 2: Reset the CC2E Bit */
//  868   TIMx->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC2E));
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable26  ;; 0xffef
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+32]
//  869   
//  870   /* Get the TIMx CCER register value */  
//  871   tmpccer = TIMx->CCER;
        LDRH     R4,[R0, #+32]
//  872   /* Get the TIMx CR2 register value */
//  873   tmpcr2 =  TIMx->CR2;
        LDRH     R2,[R0, #+4]
//  874   
//  875   /* Get the TIMx CCMR1 register value */
//  876   tmpccmrx = TIMx->CCMR1;
//  877     
//  878   /* Reset the Output Compare mode and Capture/Compare selection Bits */
//  879   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR1_OC2M));
//  880   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR1_CC2S));
//  881   
//  882   /* Select the Output Compare Mode */
//  883   tmpccmrx |= (uint16_t)(TIM_OCInitStruct->TIM_OCMode << 8);
        LDRH     R3,[R0, #+24]
        LDR      R5,??DataTable27  ;; 0x8cff
        ANDS     R5,R5,R3
        LDRH     R3,[R1, #+0]
        LSLS     R3,R3,#+8
        ORRS     R3,R3,R5
//  884   
//  885   /* Reset the Output Polarity level */
//  886   tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC2P));
//  887   /* Set the Output Compare Polarity */
//  888   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 4);
//  889   
//  890   /* Set the Output State */
//  891   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 4);
        LDR      R5,??DataTable28  ;; 0xffdf
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+12]
        LDRH     R6,[R1, #+2]
        ORRS     R6,R6,R4
        LSLS     R4,R6,#+4
        ORRS     R4,R4,R5
//  892     
//  893   if((TIMx == TIM1) || (TIMx == TIM15))
        LDR      R5,??DataTable19_1  ;; 0x40012c00
        CMP      R0,R5
        BEQ      ??TIM_OC2Init_0
        LDR      R6,??DataTable29  ;; 0x40014000
        CMP      R0,R6
        BNE      ??TIM_OC2Init_1
//  894   {
//  895     /* Check the parameters */
//  896     assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
//  897     
//  898     /* Reset the Ouput Compare State */
//  899     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS2));
//  900     
//  901     /* Set the Output Idle state */
//  902     tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 2);
??TIM_OC2Init_0:
        LDR      R6,??DataTable29_1  ;; 0xfbff
        ANDS     R6,R6,R2
        LDRH     R2,[R1, #+16]
        LSLS     R2,R2,#+2
        ORRS     R2,R2,R6
//  903     
//  904     if (TIMx == TIM1)
        CMP      R0,R5
        BNE      ??TIM_OC2Init_1
//  905     {    
//  906       /* Check the parameters */
//  907       assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
//  908       assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
//  909       assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
//  910       
//  911       /* Reset the Output N Polarity level */
//  912       tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC2NP));
//  913       /* Set the Output N Polarity */
//  914       tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCNPolarity << 4);
//  915       
//  916       /* Reset the Output N State */
//  917       tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC2NE));    
//  918       /* Set the Output N State */
//  919       tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputNState << 4);
        LDR      R5,??DataTable20  ;; 0xff7f
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+14]
        LSLS     R4,R4,#+4
        ORRS     R4,R4,R5
        LDR      R5,??DataTable32  ;; 0xffbf
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+4]
        LSLS     R4,R4,#+4
        ORRS     R4,R4,R5
//  920       
//  921       /* Reset the Output Compare N IDLE State */
//  922       tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS2N));
//  923       
//  924       /* Set the Output N Idle state */
//  925       tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCNIdleState << 2);
        LDR      R5,??DataTable33  ;; 0xf7ff
        ANDS     R5,R5,R2
        LDRH     R2,[R1, #+18]
        LSLS     R2,R2,#+2
        ORRS     R2,R2,R5
//  926     }
//  927   }
//  928   /* Write to TIMx CR2 */
//  929   TIMx->CR2 = tmpcr2;
??TIM_OC2Init_1:
        STRH     R2,[R0, #+4]
//  930   
//  931   /* Write to TIMx CCMR1 */
//  932   TIMx->CCMR1 = tmpccmrx;
        STRH     R3,[R0, #+24]
//  933 
//  934   /* Set the Capture Compare Register value */
//  935   TIMx->CCR2 = TIM_OCInitStruct->TIM_Pulse;
        LDR      R1,[R1, #+8]
        STR      R1,[R0, #+56]
//  936   
//  937   /* Write to TIMx CCER */
//  938   TIMx->CCER = tmpccer;
        STRH     R4,[R0, #+32]
//  939 }
        POP      {R0,R4-R6}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable9:
        DC32     0xff8f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable9_1:
        DC32     0xfcff
//  940 
//  941 /**
//  942   * @brief  Initializes the TIMx Channel3 according to the specified
//  943   *         parameters in the TIM_OCInitStruct.
//  944   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
//  945   * @note   TIM2 is not applicable for STM32F030 devices.  
//  946   * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
//  947   *         that contains the configuration information for the specified TIM 
//  948   *         peripheral.
//  949   * @retval None
//  950   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  951 void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
//  952 {
TIM_OC3Init:
        PUSH     {R3-R6}
//  953   uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
//  954    
//  955   /* Check the parameters */
//  956   assert_param(IS_TIM_LIST3_PERIPH(TIMx)); 
//  957   assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
//  958   assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
//  959   assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   
//  960   /* Disable the Channel 2: Reset the CC2E Bit */
//  961   TIMx->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC3E));
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable32_1  ;; 0xfeff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+32]
//  962   
//  963   /* Get the TIMx CCER register value */
//  964   tmpccer = TIMx->CCER;
        LDRH     R5,[R0, #+32]
//  965   /* Get the TIMx CR2 register value */
//  966   tmpcr2 =  TIMx->CR2;
        LDRH     R2,[R0, #+4]
//  967   
//  968   /* Get the TIMx CCMR2 register value */
//  969   tmpccmrx = TIMx->CCMR2;
//  970     
//  971   /* Reset the Output Compare mode and Capture/Compare selection Bits */
//  972   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR2_OC3M));
//  973   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR2_CC3S));  
//  974   /* Select the Output Compare Mode */
//  975   tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
        LDRH     R3,[R0, #+28]
        LDR      R4,??DataTable18_2  ;; 0xff8c
        ANDS     R4,R4,R3
        LDRH     R3,[R1, #+0]
        ORRS     R3,R3,R4
//  976   
//  977   /* Reset the Output Polarity level */
//  978   tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC3P));
//  979   /* Set the Output Compare Polarity */
//  980   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 8);
//  981   
//  982   /* Set the Output State */
//  983   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 8);
        LDR      R4,??DataTable33_1  ;; 0xfdff
        ANDS     R4,R4,R5
        LDRH     R5,[R1, #+12]
        LDRH     R6,[R1, #+2]
        ORRS     R6,R6,R5
        LSLS     R5,R6,#+8
        ORRS     R5,R5,R4
//  984     
//  985   if(TIMx == TIM1)
        LDR      R4,??DataTable19_1  ;; 0x40012c00
        CMP      R0,R4
        BNE      ??TIM_OC3Init_0
//  986   {
//  987     assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
//  988     assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
//  989     assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
//  990     assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
//  991     
//  992     /* Reset the Output N Polarity level */
//  993     tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC3NP));
//  994     /* Set the Output N Polarity */
//  995     tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCNPolarity << 8);
//  996     /* Reset the Output N State */
//  997     tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC3NE));
//  998     
//  999     /* Set the Output N State */
// 1000     tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputNState << 8);
        LDR      R4,??DataTable33  ;; 0xf7ff
        ANDS     R4,R4,R5
        LDRH     R5,[R1, #+14]
        LSLS     R5,R5,#+8
        ORRS     R5,R5,R4
        LDR      R4,??DataTable29_1  ;; 0xfbff
        ANDS     R4,R4,R5
        LDRH     R5,[R1, #+4]
        LSLS     R5,R5,#+8
        ORRS     R5,R5,R4
// 1001     /* Reset the Ouput Compare and Output Compare N IDLE State */
// 1002     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS3));
// 1003     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS3N));
// 1004     /* Set the Output Idle state */
// 1005     tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 4);
// 1006     /* Set the Output N Idle state */
// 1007     tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCNIdleState << 4);
        LDR      R4,??DataTable33_2  ;; 0xcfff
        ANDS     R4,R4,R2
        LDRH     R2,[R1, #+16]
        LDRH     R6,[R1, #+18]
        ORRS     R6,R6,R2
        LSLS     R2,R6,#+4
        ORRS     R2,R2,R4
// 1008   }
// 1009   /* Write to TIMx CR2 */
// 1010   TIMx->CR2 = tmpcr2;
??TIM_OC3Init_0:
        STRH     R2,[R0, #+4]
// 1011   
// 1012   /* Write to TIMx CCMR2 */
// 1013   TIMx->CCMR2 = tmpccmrx;
        STRH     R3,[R0, #+28]
// 1014 
// 1015   /* Set the Capture Compare Register value */
// 1016   TIMx->CCR3 = TIM_OCInitStruct->TIM_Pulse;
        LDR      R1,[R1, #+8]
        STR      R1,[R0, #+60]
// 1017   
// 1018   /* Write to TIMx CCER */
// 1019   TIMx->CCER = tmpccer;
        STRH     R5,[R0, #+32]
// 1020 }
        POP      {R0,R4-R6}
        BX       LR               ;; return
// 1021 
// 1022 /**
// 1023   * @brief  Initializes the TIMx Channel4 according to the specified
// 1024   *         parameters in the TIM_OCInitStruct.
// 1025   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1026   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1027   * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
// 1028   *         that contains the configuration information for the specified TIM 
// 1029   *         peripheral.
// 1030   * @retval None
// 1031   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1032 void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
// 1033 {
TIM_OC4Init:
        PUSH     {R3-R6}
// 1034   uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
// 1035    
// 1036   /* Check the parameters */
// 1037   assert_param(IS_TIM_LIST3_PERIPH(TIMx)); 
// 1038   assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
// 1039   assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
// 1040   assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   
// 1041   /* Disable the Channel 2: Reset the CC4E Bit */
// 1042   TIMx->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC4E));
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable33_3  ;; 0xefff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+32]
// 1043   
// 1044   /* Get the TIMx CCER register value */
// 1045   tmpccer = TIMx->CCER;
        LDRH     R3,[R0, #+32]
// 1046   /* Get the TIMx CR2 register value */
// 1047   tmpcr2 =  TIMx->CR2;
        LDRH     R2,[R0, #+4]
// 1048   
// 1049   /* Get the TIMx CCMR2 register value */
// 1050   tmpccmrx = TIMx->CCMR2;
// 1051     
// 1052   /* Reset the Output Compare mode and Capture/Compare selection Bits */
// 1053   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR2_OC4M));
// 1054   tmpccmrx &= (uint16_t)(~((uint16_t)TIM_CCMR2_CC4S));
// 1055   
// 1056   /* Select the Output Compare Mode */
// 1057   tmpccmrx |= (uint16_t)(TIM_OCInitStruct->TIM_OCMode << 8);
        LDRH     R4,[R0, #+28]
        LDR      R5,??DataTable27  ;; 0x8cff
        ANDS     R5,R5,R4
        LDRH     R4,[R1, #+0]
        LSLS     R4,R4,#+8
        ORRS     R4,R4,R5
// 1058   
// 1059   /* Reset the Output Polarity level */
// 1060   tmpccer &= (uint16_t)(~((uint16_t)TIM_CCER_CC4P));
// 1061   /* Set the Output Compare Polarity */
// 1062   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OCPolarity << 12);
// 1063   
// 1064   /* Set the Output State */
// 1065   tmpccer |= (uint16_t)(TIM_OCInitStruct->TIM_OutputState << 12);
        LDR      R5,??DataTable33_4  ;; 0xdfff
        ANDS     R5,R5,R3
        LDRH     R3,[R1, #+12]
        LDRH     R6,[R1, #+2]
        ORRS     R6,R6,R3
        LSLS     R3,R6,#+12
        ORRS     R3,R3,R5
// 1066     
// 1067   if(TIMx == TIM1)
        LDR      R5,??DataTable19_1  ;; 0x40012c00
        CMP      R0,R5
        BNE      ??TIM_OC4Init_0
// 1068   {
// 1069     assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
// 1070     /* Reset the Ouput Compare IDLE State */
// 1071     tmpcr2 &= (uint16_t)(~((uint16_t)TIM_CR2_OIS4));
// 1072     /* Set the Output Idle state */
// 1073     tmpcr2 |= (uint16_t)(TIM_OCInitStruct->TIM_OCIdleState << 6);
        LDR      R5,??DataTable33_5  ;; 0xbfff
        ANDS     R5,R5,R2
        LDRH     R2,[R1, #+16]
        LSLS     R2,R2,#+6
        ORRS     R2,R2,R5
// 1074   }
// 1075   /* Write to TIMx CR2 */
// 1076   TIMx->CR2 = tmpcr2;
??TIM_OC4Init_0:
        STRH     R2,[R0, #+4]
// 1077   
// 1078   /* Write to TIMx CCMR2 */  
// 1079   TIMx->CCMR2 = tmpccmrx;
        STRH     R4,[R0, #+28]
// 1080 
// 1081   /* Set the Capture Compare Register value */
// 1082   TIMx->CCR4 = TIM_OCInitStruct->TIM_Pulse;
        LDR      R1,[R1, #+8]
        STR      R1,[R0, #+64]
// 1083   
// 1084   /* Write to TIMx CCER */
// 1085   TIMx->CCER = tmpccer;
        STRH     R3,[R0, #+32]
// 1086 }
        POP      {R0,R4-R6}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable11:
        DC32     0x40014800
// 1087 
// 1088 /**
// 1089   * @brief  Fills each TIM_OCInitStruct member with its default value.
// 1090   * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure which will
// 1091   *         be initialized.
// 1092   * @retval None
// 1093   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1094 void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct)
// 1095 {
// 1096   /* Set the default configuration */
// 1097   TIM_OCInitStruct->TIM_OCMode = TIM_OCMode_Timing;
TIM_OCStructInit:
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1098   TIM_OCInitStruct->TIM_OutputState = TIM_OutputState_Disable;
        STRH     R1,[R0, #+2]
// 1099   TIM_OCInitStruct->TIM_OutputNState = TIM_OutputNState_Disable;
        STRH     R1,[R0, #+4]
// 1100   TIM_OCInitStruct->TIM_Pulse = 0x0000000;
        STR      R1,[R0, #+8]
// 1101   TIM_OCInitStruct->TIM_OCPolarity = TIM_OCPolarity_High;
        STRH     R1,[R0, #+12]
// 1102   TIM_OCInitStruct->TIM_OCNPolarity = TIM_OCPolarity_High;
        STRH     R1,[R0, #+14]
// 1103   TIM_OCInitStruct->TIM_OCIdleState = TIM_OCIdleState_Reset;
        STRH     R1,[R0, #+16]
// 1104   TIM_OCInitStruct->TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        STRH     R1,[R0, #+18]
// 1105 }
        BX       LR               ;; return
// 1106 
// 1107 /**
// 1108   * @brief  Selects the TIM Output Compare Mode.
// 1109   * @note   This function disables the selected channel before changing the Output
// 1110   *         Compare Mode.
// 1111   *         User has to enable this channel using TIM_CCxCmd and TIM_CCxNCmd functions.
// 1112   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1113   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1114   * @param  TIM_Channel: specifies the TIM Channel
// 1115   *          This parameter can be one of the following values:
// 1116   *            @arg TIM_Channel_1: TIM Channel 1
// 1117   *            @arg TIM_Channel_2: TIM Channel 2
// 1118   *            @arg TIM_Channel_3: TIM Channel 3
// 1119   *            @arg TIM_Channel_4: TIM Channel 4
// 1120   * @param  TIM_OCMode: specifies the TIM Output Compare Mode.
// 1121   *          This parameter can be one of the following values:
// 1122   *            @arg TIM_OCMode_Timing
// 1123   *            @arg TIM_OCMode_Active
// 1124   *            @arg TIM_OCMode_Toggle
// 1125   *            @arg TIM_OCMode_PWM1
// 1126   *            @arg TIM_OCMode_PWM2
// 1127   *            @arg TIM_ForcedAction_Active
// 1128   *            @arg TIM_ForcedAction_InActive
// 1129   * @retval None
// 1130   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1131 void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode)
// 1132 {
TIM_SelectOCxM:
        PUSH     {R3-R5}
// 1133   uint32_t tmp = 0;
// 1134   uint16_t tmp1 = 0;
// 1135 
// 1136   /* Check the parameters */
// 1137   assert_param(IS_TIM_LIST4_PERIPH(TIMx));  
// 1138   assert_param(IS_TIM_OCM(TIM_OCMode));
// 1139   
// 1140   tmp = (uint32_t) TIMx;
// 1141   tmp += CCMR_OFFSET;
        MOVS     R3,R0
        ADDS     R3,R3,#+24
// 1142 
// 1143   tmp1 = CCER_CCE_SET << (uint16_t)TIM_Channel;
// 1144 
// 1145   /* Disable the Channel: Reset the CCxE Bit */
// 1146   TIMx->CCER &= (uint16_t) ~tmp1;
        LDRH     R4,[R0, #+32]
        MOVS     R5,#+1
        LSLS     R5,R5,R1
        BICS     R4,R4,R5
        STRH     R4,[R0, #+32]
// 1147 
// 1148   if((TIM_Channel == TIM_Channel_1) ||(TIM_Channel == TIM_Channel_3))
        CMP      R1,#+0
        BEQ      ??TIM_SelectOCxM_0
        CMP      R1,#+8
        BNE      ??TIM_SelectOCxM_1
// 1149   {
// 1150     tmp += (TIM_Channel>>1);
??TIM_SelectOCxM_0:
        LSRS     R0,R1,#+1
        ADDS     R0,R3,R0
// 1151 
// 1152     /* Reset the OCxM bits in the CCMRx register */
// 1153     *(__IO uint32_t *) tmp &= (uint32_t)~((uint32_t)TIM_CCMR1_OC1M);
        LDR      R1,[R0, #+0]
        MOVS     R3,#+112
        BICS     R1,R1,R3
        STR      R1,[R0, #+0]
// 1154    
// 1155     /* Configure the OCxM bits in the CCMRx register */
// 1156     *(__IO uint32_t *) tmp |= TIM_OCMode;
        LDR      R1,[R0, #+0]
        B        ??TIM_SelectOCxM_2
// 1157   }
// 1158   else
// 1159   {
// 1160     tmp += (uint16_t)(TIM_Channel - (uint16_t)4)>> (uint16_t)1;
??TIM_SelectOCxM_1:
        SUBS     R0,R1,#+4
        LSLS     R0,R0,#+16
        LSRS     R0,R0,#+17
        ADDS     R0,R3,R0
// 1161 
// 1162     /* Reset the OCxM bits in the CCMRx register */
// 1163     *(__IO uint32_t *) tmp &= (uint32_t)~((uint32_t)TIM_CCMR1_OC2M);
        LDR      R1,[R0, #+0]
        LDR      R3,??DataTable34  ;; 0xffff8fff
        ANDS     R3,R3,R1
        STR      R3,[R0, #+0]
// 1164     
// 1165     /* Configure the OCxM bits in the CCMRx register */
// 1166     *(__IO uint32_t *) tmp |= (uint16_t)(TIM_OCMode << 8);
        LDR      R1,[R0, #+0]
        LSLS     R2,R2,#+24
        LSRS     R2,R2,#+16
??TIM_SelectOCxM_2:
        ORRS     R1,R1,R2
        STR      R1,[R0, #+0]
// 1167   }
// 1168 }
        POP      {R0,R4,R5}
        BX       LR               ;; return
// 1169 
// 1170 /**
// 1171   * @brief  Sets the TIMx Capture Compare1 Register value
// 1172   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1173   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1174   * @param  Compare1: specifies the Capture Compare1 register new value.
// 1175   * @retval None
// 1176   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1177 void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1)
// 1178 {
// 1179   /* Check the parameters */
// 1180   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1181   
// 1182   /* Set the Capture Compare1 Register value */
// 1183   TIMx->CCR1 = Compare1;
TIM_SetCompare1:
        STR      R1,[R0, #+52]
// 1184 }
        BX       LR               ;; return
// 1185 
// 1186 /**
// 1187   * @brief  Sets the TIMx Capture Compare2 Register value
// 1188   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 1189   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1190   * @param  Compare2: specifies the Capture Compare2 register new value.
// 1191   * @retval None
// 1192   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1193 void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2)
// 1194 {
// 1195   /* Check the parameters */
// 1196   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1197   
// 1198   /* Set the Capture Compare2 Register value */
// 1199   TIMx->CCR2 = Compare2;
TIM_SetCompare2:
        STR      R1,[R0, #+56]
// 1200 }
        BX       LR               ;; return
// 1201 
// 1202 /**
// 1203   * @brief  Sets the TIMx Capture Compare3 Register value
// 1204   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1205   * @param  Compare3: specifies the Capture Compare3 register new value.
// 1206   * @retval None
// 1207   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1208 void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3)
// 1209 {
// 1210   /* Check the parameters */
// 1211   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1212   
// 1213   /* Set the Capture Compare3 Register value */
// 1214   TIMx->CCR3 = Compare3;
TIM_SetCompare3:
        STR      R1,[R0, #+60]
// 1215 }
        BX       LR               ;; return
// 1216 
// 1217 /**
// 1218   * @brief  Sets the TIMx Capture Compare4 Register value
// 1219   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1220   * @note   TIM2 is not applicable for STM32F030 devices.    
// 1221   * @param  Compare4: specifies the Capture Compare4 register new value.
// 1222   * @retval None
// 1223   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1224 void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4)
// 1225 {
// 1226   /* Check the parameters */
// 1227   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1228   
// 1229   /* Set the Capture Compare4 Register value */
// 1230   TIMx->CCR4 = Compare4;
TIM_SetCompare4:
        STR      R1,[R0, #+64]
// 1231 }
        BX       LR               ;; return
// 1232 
// 1233 /**
// 1234   * @brief  Forces the TIMx output 1 waveform to active or inactive level.
// 1235   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1236   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1237   * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
// 1238   *          This parameter can be one of the following values:
// 1239   *            @arg TIM_ForcedAction_Active: Force active level on OC1REF
// 1240   *            @arg TIM_ForcedAction_InActive: Force inactive level on OC1REF.
// 1241   * @retval None
// 1242   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1243 void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
// 1244 {
// 1245   uint16_t tmpccmr1 = 0;
// 1246   /* Check the parameters */
// 1247   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1248   assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));
// 1249   tmpccmr1 = TIMx->CCMR1;
// 1250   /* Reset the OC1M Bits */
// 1251   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC1M);
// 1252   /* Configure The Forced output Mode */
// 1253   tmpccmr1 |= TIM_ForcedAction;
// 1254   /* Write to TIMx CCMR1 register */
// 1255   TIMx->CCMR1 = tmpccmr1;
TIM_ForcedOC1Config:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable34_1  ;; 0xff8f
        ANDS     R3,R3,R2
        B.N      ??Subroutine3_0
// 1256 }
// 1257  
// 1258 /**
// 1259   * @brief  Forces the TIMx output 2 waveform to active or inactive level.
// 1260   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 1261   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1262   * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
// 1263   *          This parameter can be one of the following values:
// 1264   *            @arg TIM_ForcedAction_Active: Force active level on OC2REF
// 1265   *            @arg TIM_ForcedAction_InActive: Force inactive level on OC2REF.
// 1266   * @retval None
// 1267   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1268 void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
// 1269 {
// 1270   uint16_t tmpccmr1 = 0;
// 1271   
// 1272   /* Check the parameters */
// 1273   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1274   assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));
// 1275   
// 1276   tmpccmr1 = TIMx->CCMR1;
// 1277   /* Reset the OC2M Bits */
// 1278   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC2M);
// 1279   /* Configure The Forced output Mode */
// 1280   tmpccmr1 |= (uint16_t)(TIM_ForcedAction << 8);
// 1281   /* Write to TIMx CCMR1 register */
// 1282   TIMx->CCMR1 = tmpccmr1;
TIM_ForcedOC2Config:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable34_2  ;; 0x8fff
        B.N      ?Subroutine3
// 1283 }
// 1284 
// 1285 /**
// 1286   * @brief  Forces the TIMx output 3 waveform to active or inactive level.
// 1287   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1288   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1289   * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
// 1290   *          This parameter can be one of the following values:
// 1291   *            @arg TIM_ForcedAction_Active: Force active level on OC3REF
// 1292   *            @arg TIM_ForcedAction_InActive: Force inactive level on OC3REF.
// 1293   * @retval None
// 1294   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1295 void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
// 1296 {
// 1297   uint16_t tmpccmr2 = 0;
// 1298   
// 1299   /* Check the parameters */
// 1300   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1301   assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));
// 1302   
// 1303   tmpccmr2 = TIMx->CCMR2;
// 1304   /* Reset the OC1M Bits */
// 1305   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC3M);
// 1306   /* Configure The Forced output Mode */
// 1307   tmpccmr2 |= TIM_ForcedAction;
// 1308   /* Write to TIMx CCMR2 register */
// 1309   TIMx->CCMR2 = tmpccmr2;
TIM_ForcedOC3Config:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable34_1  ;; 0xff8f
        ANDS     R3,R3,R2
        B.N      ??Subroutine4_0
// 1310 }
// 1311 
// 1312 /**
// 1313   * @brief  Forces the TIMx output 4 waveform to active or inactive level.
// 1314   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1315   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1316   * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
// 1317   *          This parameter can be one of the following values:
// 1318   *            @arg TIM_ForcedAction_Active: Force active level on OC4REF
// 1319   *            @arg TIM_ForcedAction_InActive: Force inactive level on OC4REF.
// 1320   * @retval None
// 1321   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1322 void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction)
// 1323 {
// 1324   uint16_t tmpccmr2 = 0;
// 1325   /* Check the parameters */
// 1326   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1327   assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));
// 1328   
// 1329   tmpccmr2 = TIMx->CCMR2;
// 1330   /* Reset the OC2M Bits */
// 1331   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC4M);
// 1332   /* Configure The Forced output Mode */
// 1333   tmpccmr2 |= (uint16_t)(TIM_ForcedAction << 8);
// 1334   /* Write to TIMx CCMR2 register */
// 1335   TIMx->CCMR2 = tmpccmr2;
TIM_ForcedOC4Config:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable34_2  ;; 0x8fff
        B.N      ?Subroutine4
// 1336 }
// 1337 
// 1338 /**
// 1339   * @brief  Sets or Resets the TIM peripheral Capture Compare Preload Control bit.
// 1340   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIMx peripheral
// 1341   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1342   * @param  NewState: new state of the Capture Compare Preload Control bit
// 1343   *          This parameter can be: ENABLE or DISABLE.
// 1344   * @retval None
// 1345   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1346 void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState)
// 1347 { 
// 1348   /* Check the parameters */
// 1349   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1350   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1351   if (NewState != DISABLE)
TIM_CCPreloadControl:
        CMP      R1,#+0
        LDRH     R1,[R0, #+4]
        BEQ      ??TIM_CCPreloadControl_0
// 1352   {
// 1353     /* Set the CCPC Bit */
// 1354     TIMx->CR2 |= TIM_CR2_CCPC;
        MOVS     R2,#+1
        ORRS     R2,R2,R1
        B        ??TIM_CCPreloadControl_1
// 1355   }
// 1356   else
// 1357   {
// 1358     /* Reset the CCPC Bit */
// 1359     TIMx->CR2 &= (uint16_t)~((uint16_t)TIM_CR2_CCPC);
??TIM_CCPreloadControl_0:
        LDR      R2,??DataTable18_1  ;; 0xfffe
        ANDS     R2,R2,R1
??TIM_CCPreloadControl_1:
        STRH     R2,[R0, #+4]
// 1360   }
// 1361 }
        BX       LR               ;; return
// 1362 
// 1363 
// 1364 /**
// 1365   * @brief  Enables or disables the TIMx peripheral Preload register on CCR1.
// 1366   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 and 17 to select the TIM peripheral.
// 1367   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1368   * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
// 1369   *          This parameter can be one of the following values:
// 1370   *            @arg TIM_OCPreload_Enable
// 1371   *            @arg TIM_OCPreload_Disable
// 1372   * @retval None
// 1373   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1374 void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
// 1375 {
// 1376   uint16_t tmpccmr1 = 0;
// 1377   /* Check the parameters */
// 1378   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1379   assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));
// 1380   
// 1381   tmpccmr1 = TIMx->CCMR1;
// 1382   /* Reset the OC1PE Bit */
// 1383   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC1PE);
// 1384   /* Enable or Disable the Output Compare Preload feature */
// 1385   tmpccmr1 |= TIM_OCPreload;
// 1386   /* Write to TIMx CCMR1 register */
// 1387   TIMx->CCMR1 = tmpccmr1;
TIM_OC1PreloadConfig:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable23  ;; 0xfff7
        ANDS     R3,R3,R2
        B.N      ??Subroutine3_0
// 1388 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18:
        DC32     0xfffd

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_1:
        DC32     0xfffe

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable18_2:
        DC32     0xff8c
// 1389 
// 1390 /**
// 1391   * @brief  Enables or disables the TIMx peripheral Preload register on CCR2.
// 1392   * @param  TIMx: where x can be 1, 2, 3 and 15 to select the TIM peripheral.
// 1393   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1394   * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
// 1395   *          This parameter can be one of the following values:
// 1396   *            @arg TIM_OCPreload_Enable
// 1397   *            @arg TIM_OCPreload_Disable
// 1398   * @retval None
// 1399   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1400 void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
// 1401 {
// 1402   uint16_t tmpccmr1 = 0;
// 1403   /* Check the parameters */
// 1404   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1405   assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));
// 1406   
// 1407   tmpccmr1 = TIMx->CCMR1;
// 1408   /* Reset the OC2PE Bit */
// 1409   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC2PE);
// 1410   /* Enable or Disable the Output Compare Preload feature */
// 1411   tmpccmr1 |= (uint16_t)(TIM_OCPreload << 8);
// 1412   /* Write to TIMx CCMR1 register */
// 1413   TIMx->CCMR1 = tmpccmr1;
TIM_OC2PreloadConfig:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable33  ;; 0xf7ff
        REQUIRE ?Subroutine3
        ;; // Fall through to label ?Subroutine3
// 1414 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine3:
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+8
??Subroutine3_0:
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+24]
        BX       LR               ;; return
// 1415 
// 1416 /**
// 1417   * @brief  Enables or disables the TIMx peripheral Preload register on CCR3.
// 1418   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1419   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1420   * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
// 1421   *          This parameter can be one of the following values:
// 1422   *            @arg TIM_OCPreload_Enable
// 1423   *            @arg TIM_OCPreload_Disable
// 1424   * @retval None
// 1425   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1426 void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
// 1427 {
// 1428   uint16_t tmpccmr2 = 0;
// 1429   
// 1430   /* Check the parameters */
// 1431   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1432   assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));
// 1433   
// 1434   tmpccmr2 = TIMx->CCMR2;
// 1435   /* Reset the OC3PE Bit */
// 1436   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC3PE);
// 1437   /* Enable or Disable the Output Compare Preload feature */
// 1438   tmpccmr2 |= TIM_OCPreload;
// 1439   /* Write to TIMx CCMR2 register */
// 1440   TIMx->CCMR2 = tmpccmr2;
TIM_OC3PreloadConfig:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable23  ;; 0xfff7
        ANDS     R3,R3,R2
        B.N      ??Subroutine4_0
// 1441 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19:
        DC32     0xfffb

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable19_1:
        DC32     0x40012c00
// 1442 
// 1443 /**
// 1444   * @brief  Enables or disables the TIMx peripheral Preload register on CCR4.
// 1445   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1446   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1447   * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
// 1448   *          This parameter can be one of the following values:
// 1449   *            @arg TIM_OCPreload_Enable
// 1450   *            @arg TIM_OCPreload_Disable
// 1451   * @retval None
// 1452   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1453 void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
// 1454 {
// 1455   uint16_t tmpccmr2 = 0;
// 1456   
// 1457   /* Check the parameters */
// 1458   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1459   assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));
// 1460   
// 1461   tmpccmr2 = TIMx->CCMR2;
// 1462   /* Reset the OC4PE Bit */
// 1463   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC4PE);
// 1464   /* Enable or Disable the Output Compare Preload feature */
// 1465   tmpccmr2 |= (uint16_t)(TIM_OCPreload << 8);
// 1466   /* Write to TIMx CCMR2 register */
// 1467   TIMx->CCMR2 = tmpccmr2;
TIM_OC4PreloadConfig:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable33  ;; 0xf7ff
        REQUIRE ?Subroutine4
        ;; // Fall through to label ?Subroutine4
// 1468 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine4:
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+8
??Subroutine4_0:
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+28]
        BX       LR               ;; return
// 1469 
// 1470 /**
// 1471   * @brief  Configures the TIMx Output Compare 1 Fast feature.
// 1472   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1473   * @note   TIM2 is not applicable for STM32F030 devices.
// 1474   * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
// 1475   *          This parameter can be one of the following values:
// 1476   *            @arg TIM_OCFast_Enable: TIM output compare fast enable
// 1477   *            @arg TIM_OCFast_Disable: TIM output compare fast disable
// 1478   * @retval None
// 1479   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1480 void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
// 1481 {
// 1482   uint16_t tmpccmr1 = 0;
// 1483   
// 1484   /* Check the parameters */
// 1485   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1486   assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));
// 1487   
// 1488   /* Get the TIMx CCMR1 register value */
// 1489   tmpccmr1 = TIMx->CCMR1;
// 1490   /* Reset the OC1FE Bit */
// 1491   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC1FE);
// 1492   /* Enable or Disable the Output Compare Fast Bit */
// 1493   tmpccmr1 |= TIM_OCFast;
// 1494   /* Write to TIMx CCMR1 */
// 1495   TIMx->CCMR1 = tmpccmr1;
TIM_OC1FastConfig:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable23_1  ;; 0xfffb
        ANDS     R3,R3,R2
        B.N      ??Subroutine3_0
// 1496 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable20:
        DC32     0xff7f
// 1497 
// 1498 /**
// 1499   * @brief  Configures the TIMx Output Compare 2 Fast feature.
// 1500   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 1501   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1502   * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
// 1503   *          This parameter can be one of the following values:
// 1504   *            @arg TIM_OCFast_Enable: TIM output compare fast enable
// 1505   *            @arg TIM_OCFast_Disable: TIM output compare fast disable
// 1506   * @retval None
// 1507   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1508 void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
// 1509 {
// 1510   uint16_t tmpccmr1 = 0;
// 1511   
// 1512   /* Check the parameters */
// 1513   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1514   assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));
// 1515   
// 1516   /* Get the TIMx CCMR1 register value */
// 1517   tmpccmr1 = TIMx->CCMR1;
// 1518   /* Reset the OC2FE Bit */
// 1519   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC2FE);
// 1520   /* Enable or Disable the Output Compare Fast Bit */
// 1521   tmpccmr1 |= (uint16_t)(TIM_OCFast << 8);
// 1522   /* Write to TIMx CCMR1 */
// 1523   TIMx->CCMR1 = tmpccmr1;
TIM_OC2FastConfig:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable29_1  ;; 0xfbff
        B.N      ?Subroutine3
// 1524 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21:
        DC32     0xfff7
// 1525 
// 1526 /**
// 1527   * @brief  Configures the TIMx Output Compare 3 Fast feature.
// 1528   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1529   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1530   * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
// 1531   *          This parameter can be one of the following values:
// 1532   *            @arg TIM_OCFast_Enable: TIM output compare fast enable
// 1533   *            @arg TIM_OCFast_Disable: TIM output compare fast disable
// 1534   * @retval None
// 1535   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1536 void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
// 1537 {
// 1538   uint16_t tmpccmr2 = 0;
// 1539   
// 1540   /* Check the parameters */
// 1541   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1542   assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));
// 1543   
// 1544   /* Get the TIMx CCMR2 register value */
// 1545   tmpccmr2 = TIMx->CCMR2;
// 1546   /* Reset the OC3FE Bit */
// 1547   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC3FE);
// 1548   /* Enable or Disable the Output Compare Fast Bit */
// 1549   tmpccmr2 |= TIM_OCFast;
// 1550   /* Write to TIMx CCMR2 */
// 1551   TIMx->CCMR2 = tmpccmr2;
TIM_OC3FastConfig:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable23_1  ;; 0xfffb
        ANDS     R3,R3,R2
        B.N      ??Subroutine4_0
// 1552 }
// 1553 
// 1554 /**
// 1555   * @brief  Configures the TIMx Output Compare 4 Fast feature.
// 1556   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1557   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1558   * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
// 1559   *          This parameter can be one of the following values:
// 1560   *            @arg TIM_OCFast_Enable: TIM output compare fast enable
// 1561   *            @arg TIM_OCFast_Disable: TIM output compare fast disable
// 1562   * @retval None
// 1563   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1564 void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast)
// 1565 {
// 1566   uint16_t tmpccmr2 = 0;
// 1567   
// 1568   /* Check the parameters */
// 1569   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1570   assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));
// 1571   
// 1572   /* Get the TIMx CCMR2 register value */
// 1573   tmpccmr2 = TIMx->CCMR2;
// 1574   /* Reset the OC4FE Bit */
// 1575   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC4FE);
// 1576   /* Enable or Disable the Output Compare Fast Bit */
// 1577   tmpccmr2 |= (uint16_t)(TIM_OCFast << 8);
// 1578   /* Write to TIMx CCMR2 */
// 1579   TIMx->CCMR2 = tmpccmr2;
TIM_OC4FastConfig:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable29_1  ;; 0xfbff
        B.N      ?Subroutine4
// 1580 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23:
        DC32     0xfff7

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_1:
        DC32     0xfffb
// 1581 
// 1582 /**
// 1583   * @brief  Clears or safeguards the OCREF1 signal on an external event
// 1584   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1585   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1586   * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
// 1587   *          This parameter can be one of the following values:
// 1588   *            @arg TIM_OCClear_Enable: TIM Output clear enable
// 1589   *            @arg TIM_OCClear_Disable: TIM Output clear disable
// 1590   * @retval None
// 1591   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1592 void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
// 1593 {
// 1594   uint16_t tmpccmr1 = 0;
// 1595   
// 1596   /* Check the parameters */
// 1597   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1598   assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));
// 1599   
// 1600   tmpccmr1 = TIMx->CCMR1;
// 1601   /* Reset the OC1CE Bit */
// 1602   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC1CE);
// 1603   /* Enable or Disable the Output Compare Clear Bit */
// 1604   tmpccmr1 |= TIM_OCClear;
// 1605   /* Write to TIMx CCMR1 register */
// 1606   TIMx->CCMR1 = tmpccmr1;
TIM_ClearOC1Ref:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable31  ;; 0xff7f
        ANDS     R3,R3,R2
        B.N      ??Subroutine3_0
// 1607 }
// 1608 
// 1609 /**
// 1610   * @brief  Clears or safeguards the OCREF2 signal on an external event
// 1611   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 1612   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1613   * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
// 1614   *          This parameter can be one of the following values:
// 1615   *            @arg TIM_OCClear_Enable: TIM Output clear enable
// 1616   *            @arg TIM_OCClear_Disable: TIM Output clear disable
// 1617   * @retval None
// 1618   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1619 void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
// 1620 {
// 1621   uint16_t tmpccmr1 = 0;
// 1622   
// 1623   /* Check the parameters */
// 1624   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1625   assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));
// 1626   
// 1627   tmpccmr1 = TIMx->CCMR1;
// 1628   /* Reset the OC2CE Bit */
// 1629   tmpccmr1 &= (uint16_t)~((uint16_t)TIM_CCMR1_OC2CE);
// 1630   /* Enable or Disable the Output Compare Clear Bit */
// 1631   tmpccmr1 |= (uint16_t)(TIM_OCClear << 8);
// 1632   /* Write to TIMx CCMR1 register */
// 1633   TIMx->CCMR1 = tmpccmr1;
TIM_ClearOC2Ref:
        LDRH     R2,[R0, #+24]
        LSLS     R2,R2,#+17
        LSRS     R2,R2,#+17
        LSLS     R1,R1,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+24]
// 1634 }
        BX       LR               ;; return
// 1635 
// 1636 /**
// 1637   * @brief  Clears or safeguards the OCREF3 signal on an external event
// 1638   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1639   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1640   * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
// 1641   *          This parameter can be one of the following values:
// 1642   *            @arg TIM_OCClear_Enable: TIM Output clear enable
// 1643   *            @arg TIM_OCClear_Disable: TIM Output clear disable
// 1644   * @retval None
// 1645   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1646 void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
// 1647 {
// 1648   uint16_t tmpccmr2 = 0;
// 1649   
// 1650   /* Check the parameters */
// 1651   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1652   assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));
// 1653   
// 1654   tmpccmr2 = TIMx->CCMR2;
// 1655   /* Reset the OC3CE Bit */
// 1656   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC3CE);
// 1657   /* Enable or Disable the Output Compare Clear Bit */
// 1658   tmpccmr2 |= TIM_OCClear;
// 1659   /* Write to TIMx CCMR2 register */
// 1660   TIMx->CCMR2 = tmpccmr2;
TIM_ClearOC3Ref:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable31  ;; 0xff7f
        ANDS     R3,R3,R2
        B.N      ??Subroutine4_0
// 1661 }
// 1662 
// 1663 /**
// 1664   * @brief  Clears or safeguards the OCREF4 signal on an external event
// 1665   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1666   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1667   * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
// 1668   *          This parameter can be one of the following values:
// 1669   *            @arg TIM_OCClear_Enable: TIM Output clear enable
// 1670   *            @arg TIM_OCClear_Disable: TIM Output clear disable
// 1671   * @retval None
// 1672   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1673 void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear)
// 1674 {
// 1675   uint16_t tmpccmr2 = 0;
// 1676   
// 1677   /* Check the parameters */
// 1678   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1679   assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));
// 1680   
// 1681   tmpccmr2 = TIMx->CCMR2;
// 1682   /* Reset the OC4CE Bit */
// 1683   tmpccmr2 &= (uint16_t)~((uint16_t)TIM_CCMR2_OC4CE);
// 1684   /* Enable or Disable the Output Compare Clear Bit */
// 1685   tmpccmr2 |= (uint16_t)(TIM_OCClear << 8);
// 1686   /* Write to TIMx CCMR2 register */
// 1687   TIMx->CCMR2 = tmpccmr2;
TIM_ClearOC4Ref:
        LDRH     R2,[R0, #+28]
        LSLS     R2,R2,#+17
        LSRS     R2,R2,#+17
        LSLS     R1,R1,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+28]
// 1688 }
        BX       LR               ;; return
// 1689 
// 1690 /**
// 1691   * @brief  Configures the TIMx channel 1 polarity.
// 1692   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1693   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1694   * @param  TIM_OCPolarity: specifies the OC1 Polarity
// 1695   *          This parmeter can be one of the following values:
// 1696   *            @arg TIM_OCPolarity_High: Output Compare active high
// 1697   *            @arg TIM_OCPolarity_Low: Output Compare active low
// 1698   * @retval None
// 1699   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1700 void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
// 1701 {
// 1702   uint16_t tmpccer = 0;
// 1703   
// 1704   /* Check the parameters */
// 1705   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 1706   assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));
// 1707   
// 1708   tmpccer = TIMx->CCER;
// 1709   /* Set or Reset the CC1P Bit */
// 1710   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC1P);
// 1711   tmpccer |= TIM_OCPolarity;
// 1712   /* Write to TIMx CCER register */
// 1713   TIMx->CCER = tmpccer;
TIM_OC1PolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable38  ;; 0xfffd
        ANDS     R3,R3,R2
        B.N      ??Subroutine5_0
// 1714 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable26:
        DC32     0xffef
// 1715 
// 1716 /**
// 1717   * @brief  Configures the TIMx Channel 1N polarity.
// 1718   * @param  TIMx: where x can be 1, 15, 16 or 17 to select the TIM peripheral.
// 1719   * @param  TIM_OCNPolarity: specifies the OC1N Polarity
// 1720   *          This parmeter can be one of the following values:
// 1721   *            @arg TIM_OCNPolarity_High: Output Compare active high
// 1722   *            @arg TIM_OCNPolarity_Low: Output Compare active low
// 1723   * @retval None
// 1724   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1725 void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
// 1726 {
// 1727   uint16_t tmpccer = 0;
// 1728   /* Check the parameters */
// 1729   assert_param(IS_TIM_LIST2_PERIPH(TIMx));
// 1730   assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
// 1731    
// 1732   tmpccer = TIMx->CCER;
// 1733   /* Set or Reset the CC1NP Bit */
// 1734   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC1NP);
// 1735   tmpccer |= TIM_OCNPolarity;
// 1736   /* Write to TIMx CCER register */
// 1737   TIMx->CCER = tmpccer;
TIM_OC1NPolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable38_1  ;; 0xfff7
        ANDS     R3,R3,R2
        B.N      ??Subroutine5_0
// 1738 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27:
        DC32     0x8cff
// 1739 
// 1740 /**
// 1741   * @brief  Configures the TIMx channel 2 polarity.
// 1742   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 1743   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1744   * @param  TIM_OCPolarity: specifies the OC2 Polarity
// 1745   *          This parmeter can be one of the following values:
// 1746   *            @arg TIM_OCPolarity_High: Output Compare active high
// 1747   *            @arg TIM_OCPolarity_Low: Output Compare active low
// 1748   * @retval None
// 1749   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1750 void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
// 1751 {
// 1752   uint16_t tmpccer = 0;
// 1753   
// 1754   /* Check the parameters */
// 1755   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 1756   assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));
// 1757   
// 1758   tmpccer = TIMx->CCER;
// 1759   /* Set or Reset the CC2P Bit */
// 1760   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC2P);
// 1761   tmpccer |= (uint16_t)(TIM_OCPolarity << 4);
// 1762   /* Write to TIMx CCER register */
// 1763   TIMx->CCER = tmpccer;
TIM_OC2PolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable28  ;; 0xffdf
        B.N      ?Subroutine5
// 1764 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable28:
        DC32     0xffdf
// 1765 
// 1766 /**
// 1767   * @brief  Configures the TIMx Channel 2N polarity.
// 1768   * @param  TIMx: where x can be 1 to select the TIM peripheral.
// 1769   * @param  TIM_OCNPolarity: specifies the OC2N Polarity
// 1770   *          This parmeter can be one of the following values:
// 1771   *            @arg TIM_OCNPolarity_High: Output Compare active high
// 1772   *            @arg TIM_OCNPolarity_Low: Output Compare active low
// 1773   * @retval None
// 1774   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1775 void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
// 1776 {
// 1777   uint16_t tmpccer = 0;
// 1778   /* Check the parameters */
// 1779   assert_param(IS_TIM_LIST1_PERIPH(TIMx));
// 1780   assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
// 1781   
// 1782   tmpccer = TIMx->CCER;
// 1783   /* Set or Reset the CC2NP Bit */
// 1784   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC2NP);
// 1785   tmpccer |= (uint16_t)(TIM_OCNPolarity << 4);
// 1786   /* Write to TIMx CCER register */
// 1787   TIMx->CCER = tmpccer;
TIM_OC2NPolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable31  ;; 0xff7f
        REQUIRE ?Subroutine5
        ;; // Fall through to label ?Subroutine5
// 1788 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine5:
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+4
??Subroutine5_0:
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+32]
        BX       LR               ;; return
// 1789 
// 1790 /**
// 1791   * @brief  Configures the TIMx channel 3 polarity.
// 1792   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1793   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1794   * @param  TIM_OCPolarity: specifies the OC3 Polarity
// 1795   *          This parmeter can be one of the following values:
// 1796   *            @arg TIM_OCPolarity_High: Output Compare active high
// 1797   *            @arg TIM_OCPolarity_Low: Output Compare active low
// 1798   * @retval None
// 1799   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1800 void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
// 1801 {
// 1802   uint16_t tmpccer = 0;
// 1803   
// 1804   /* Check the parameters */
// 1805   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1806   assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));
// 1807   
// 1808   tmpccer = TIMx->CCER;
// 1809   /* Set or Reset the CC3P Bit */
// 1810   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC3P);
// 1811   tmpccer |= (uint16_t)(TIM_OCPolarity << 8);
// 1812   /* Write to TIMx CCER register */
// 1813   TIMx->CCER = tmpccer;
TIM_OC3PolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable33_1  ;; 0xfdff
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+8
        B.N      ??Subroutine5_0
// 1814 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable29:
        DC32     0x40014000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable29_1:
        DC32     0xfbff
// 1815 
// 1816 /**
// 1817   * @brief  Configures the TIMx Channel 3N polarity.
// 1818   * @param  TIMx: where x can be 1 to select the TIM peripheral.
// 1819   * @param  TIM_OCNPolarity: specifies the OC3N Polarity
// 1820   *          This parmeter can be one of the following values:
// 1821   *            @arg TIM_OCNPolarity_High: Output Compare active high
// 1822   *            @arg TIM_OCNPolarity_Low: Output Compare active low
// 1823   * @retval None
// 1824   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1825 void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity)
// 1826 {
// 1827   uint16_t tmpccer = 0;
// 1828  
// 1829   /* Check the parameters */
// 1830   assert_param(IS_TIM_LIST1_PERIPH(TIMx));
// 1831   assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
// 1832     
// 1833   tmpccer = TIMx->CCER;
// 1834   /* Set or Reset the CC3NP Bit */
// 1835   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC3NP);
// 1836   tmpccer |= (uint16_t)(TIM_OCNPolarity << 8);
// 1837   /* Write to TIMx CCER register */
// 1838   TIMx->CCER = tmpccer;
TIM_OC3NPolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable33  ;; 0xf7ff
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+8
        B.N      ??Subroutine5_0
// 1839 }
// 1840 
// 1841 /**
// 1842   * @brief  Configures the TIMx channel 4 polarity.
// 1843   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1844   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1845   * @param  TIM_OCPolarity: specifies the OC4 Polarity
// 1846   *          This parmeter can be one of the following values:
// 1847   *            @arg TIM_OCPolarity_High: Output Compare active high
// 1848   *            @arg TIM_OCPolarity_Low: Output Compare active low
// 1849   * @retval None
// 1850   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1851 void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
// 1852 {
// 1853   uint16_t tmpccer = 0;
// 1854   
// 1855   /* Check the parameters */
// 1856   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1857   assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));
// 1858   
// 1859   tmpccer = TIMx->CCER;
// 1860   /* Set or Reset the CC4P Bit */
// 1861   tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC4P);
// 1862   tmpccer |= (uint16_t)(TIM_OCPolarity << 12);
// 1863   /* Write to TIMx CCER register */
// 1864   TIMx->CCER = tmpccer;
TIM_OC4PolarityConfig:
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable33_4  ;; 0xdfff
        ANDS     R3,R3,R2
        LSLS     R1,R1,#+12
        B.N      ??Subroutine5_0
// 1865 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable31:
        DC32     0xff7f
// 1866 
// 1867 /**
// 1868   * @brief  Selects the OCReference Clear source.
// 1869   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 1870   * @note   TIM2 is not applicable for STM32F030 devices.  
// 1871   * @param  TIM_OCReferenceClear: specifies the OCReference Clear source.
// 1872   *          This parameter can be one of the following values:
// 1873   *            @arg TIM_OCReferenceClear_ETRF: The internal OCreference clear input is connected to ETRF.
// 1874   *            @arg TIM_OCReferenceClear_OCREFCLR: The internal OCreference clear input is connected to OCREF_CLR input.  
// 1875   * @retval None
// 1876   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1877 void TIM_SelectOCREFClear(TIM_TypeDef* TIMx, uint16_t TIM_OCReferenceClear)
// 1878 {
// 1879   /* Check the parameters */
// 1880   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 1881   assert_param(TIM_OCREFERENCECECLEAR_SOURCE(TIM_OCReferenceClear));
// 1882 
// 1883   /* Set the TIM_OCReferenceClear source */
// 1884   TIMx->SMCR &=  (uint16_t)~((uint16_t)TIM_SMCR_OCCS);
TIM_SelectOCREFClear:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable38_1  ;; 0xfff7
        B.N      ?Subroutine2
// 1885   TIMx->SMCR |=  TIM_OCReferenceClear;
// 1886 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable32:
        DC32     0xffbf

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable32_1:
        DC32     0xfeff
// 1887 
// 1888 /**
// 1889   * @brief  Enables or disables the TIM Capture Compare Channel x.
// 1890   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 1891   * @note   TIM2 is not applicable for STM32F030 devices.
// 1892   * @param  TIM_Channel: specifies the TIM Channel
// 1893   *          This parameter can be one of the following values:
// 1894   *            @arg TIM_Channel_1: TIM Channel 1
// 1895   *            @arg TIM_Channel_2: TIM Channel 2
// 1896   *            @arg TIM_Channel_3: TIM Channel 3
// 1897   *            @arg TIM_Channel_4: TIM Channel 4
// 1898   * @param  TIM_CCx: specifies the TIM Channel CCxE bit new state.
// 1899   *          This parameter can be: TIM_CCx_Enable or TIM_CCx_Disable. 
// 1900   * @retval None
// 1901   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1902 void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx)
// 1903 {
TIM_CCxCmd:
        PUSH     {R4}
// 1904   uint16_t tmp = 0;
// 1905 
// 1906   /* Check the parameters */
// 1907   assert_param(IS_TIM_LIST4_PERIPH(TIMx)); 
// 1908   assert_param(IS_TIM_CCX(TIM_CCx));
// 1909 
// 1910   tmp = CCER_CCE_SET << TIM_Channel;
// 1911 
// 1912   /* Reset the CCxE Bit */
// 1913   TIMx->CCER &= (uint16_t)~ tmp;
        LDRH     R3,[R0, #+32]
        MOVS     R4,#+1
        B.N      ?Subroutine0
// 1914 
// 1915   /* Set or reset the CCxE Bit */ 
// 1916   TIMx->CCER |=  (uint16_t)(TIM_CCx << TIM_Channel);
// 1917 }
// 1918 
// 1919 /**
// 1920   * @brief  Enables or disables the TIM Capture Compare Channel xN.
// 1921   * @param  TIMx: where x can be 1, 15, 16 or 17 to select the TIM peripheral.
// 1922   * @param  TIM_Channel: specifies the TIM Channel
// 1923   *          This parmeter can be one of the following values:
// 1924   *            @arg TIM_Channel_1: TIM Channel 1
// 1925   *            @arg TIM_Channel_2: TIM Channel 2
// 1926   *            @arg TIM_Channel_3: TIM Channel 3
// 1927   * @param  TIM_CCxN: specifies the TIM Channel CCxNE bit new state.
// 1928   *          This parameter can be: TIM_CCxN_Enable or TIM_CCxN_Disable. 
// 1929   * @retval None
// 1930   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1931 void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN)
// 1932 {
TIM_CCxNCmd:
        PUSH     {R4}
// 1933   uint16_t tmp = 0;
// 1934 
// 1935   /* Check the parameters */
// 1936   assert_param(IS_TIM_LIST2_PERIPH(TIMx));
// 1937   assert_param(IS_TIM_COMPLEMENTARY_CHANNEL(TIM_Channel));
// 1938   assert_param(IS_TIM_CCXN(TIM_CCxN));
// 1939 
// 1940   tmp = CCER_CCNE_SET << TIM_Channel;
// 1941 
// 1942   /* Reset the CCxNE Bit */
// 1943   TIMx->CCER &= (uint16_t) ~tmp;
        LDRH     R3,[R0, #+32]
        MOVS     R4,#+4
        REQUIRE ?Subroutine0
        ;; // Fall through to label ?Subroutine0
// 1944 
// 1945   /* Set or reset the CCxNE Bit */ 
// 1946   TIMx->CCER |=  (uint16_t)(TIM_CCxN << TIM_Channel);
// 1947 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine0:
        LSLS     R4,R4,R1
        BICS     R3,R3,R4
        STRH     R3,[R0, #+32]
        LDRH     R3,[R0, #+32]
        LSLS     R2,R2,R1
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
        POP      {R4}
        BX       LR               ;; return
// 1948 
// 1949 /**
// 1950   * @brief  Selects the TIM peripheral Commutation event.
// 1951   * @param  TIMx: where x can be  1, 15, 16 or 17 to select the TIMx peripheral
// 1952   * @param  NewState: new state of the Commutation event.
// 1953   *          This parameter can be: ENABLE or DISABLE.
// 1954   * @retval None
// 1955   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1956 void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState)
// 1957 {
// 1958   /* Check the parameters */
// 1959   assert_param(IS_TIM_LIST2_PERIPH(TIMx));
// 1960   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1961   if (NewState != DISABLE)
TIM_SelectCOM:
        CMP      R1,#+0
        LDRH     R1,[R0, #+4]
        BEQ      ??TIM_SelectCOM_0
// 1962   {
// 1963     /* Set the COM Bit */
// 1964     TIMx->CR2 |= TIM_CR2_CCUS;
        MOVS     R2,#+4
        ORRS     R2,R2,R1
        B        ??TIM_SelectCOM_1
// 1965   }
// 1966   else
// 1967   {
// 1968     /* Reset the COM Bit */
// 1969     TIMx->CR2 &= (uint16_t)~((uint16_t)TIM_CR2_CCUS);
??TIM_SelectCOM_0:
        LDR      R2,??DataTable41  ;; 0xfffb
        ANDS     R2,R2,R1
??TIM_SelectCOM_1:
        STRH     R2,[R0, #+4]
// 1970   }
// 1971 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33:
        DC32     0xf7ff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33_1:
        DC32     0xfdff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33_2:
        DC32     0xcfff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33_3:
        DC32     0xefff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33_4:
        DC32     0xdfff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33_5:
        DC32     0xbfff
// 1972 
// 1973 /**
// 1974   * @}
// 1975   */
// 1976 
// 1977 /** @defgroup TIM_Group4 Input Capture management functions
// 1978  *  @brief    Input Capture management functions 
// 1979  *
// 1980 @verbatim
// 1981  ===============================================================================
// 1982                ##### Input Capture management functions #####
// 1983  ===============================================================================
// 1984    
// 1985           *** TIM Driver: how to use it in Input Capture Mode ***
// 1986  ===============================================================================
// 1987     [..] To use the Timer in Input Capture mode, the following steps are mandatory:
// 1988          (#) Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) 
// 1989              function.
// 1990          (#) Configure the TIM pins by configuring the corresponding GPIO pins.
// 1991          (#) Configure the Time base unit as described in the first part of this 
// 1992              driver, if needed, else the Timer will run with the default configuration:
// 1993              (++) Autoreload value = 0xFFFF.
// 1994              (++) Prescaler value = 0x0000.
// 1995              (++) Counter mode = Up counting.
// 1996              (++) Clock Division = TIM_CKD_DIV1.
// 1997          (#) Fill the TIM_ICInitStruct with the desired parameters including:
// 1998              (++) TIM Channel: TIM_Channel.
// 1999              (++) TIM Input Capture polarity: TIM_ICPolarity.
// 2000              (++) TIM Input Capture selection: TIM_ICSelection.
// 2001              (++) TIM Input Capture Prescaler: TIM_ICPrescaler.
// 2002              (++) TIM Input CApture filter value: TIM_ICFilter.
// 2003          (#) Call TIM_ICInit(TIMx, &TIM_ICInitStruct) to configure the desired 
// 2004              channel with the corresponding configuration and to measure only 
// 2005              frequency or duty cycle of the input signal,or, Call 
// 2006              TIM_PWMIConfig(TIMx, &TIM_ICInitStruct) to configure the desired 
// 2007              channels with the corresponding configuration and to measure the 
// 2008              frequency and the duty cycle of the input signal.
// 2009          (#) Enable the NVIC or the DMA to read the measured frequency.
// 2010          (#) Enable the corresponding interrupt (or DMA request) to read 
// 2011              the Captured value, using the function TIM_ITConfig(TIMx, TIM_IT_CCx)
// 2012              (or TIM_DMA_Cmd(TIMx, TIM_DMA_CCx)).
// 2013          (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
// 2014          (#) Use TIM_GetCapturex(TIMx); to read the captured value.
// 2015     [..]
// 2016         (@) All other functions can be used separately to modify, if needed,
// 2017             a specific feature of the Timer. 
// 2018 
// 2019 @endverbatim
// 2020   * @{
// 2021   */
// 2022 
// 2023 /**
// 2024   * @brief  Initializes the TIM peripheral according to the specified
// 2025   *         parameters in the TIM_ICInitStruct.
// 2026   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 2027   * @note   TIM2 is not applicable for STM32F030 devices.
// 2028   * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure
// 2029   *         that contains the configuration information for the specified TIM 
// 2030   *         peripheral.
// 2031   * @retval None
// 2032   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2033 void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
// 2034 {
TIM_ICInit:
        PUSH     {R3-R7}
// 2035   /* Check the parameters */
// 2036   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 2037   assert_param(IS_TIM_CHANNEL(TIM_ICInitStruct->TIM_Channel));  
// 2038   assert_param(IS_TIM_IC_SELECTION(TIM_ICInitStruct->TIM_ICSelection));
// 2039   assert_param(IS_TIM_IC_PRESCALER(TIM_ICInitStruct->TIM_ICPrescaler));
// 2040   assert_param(IS_TIM_IC_FILTER(TIM_ICInitStruct->TIM_ICFilter));
// 2041   assert_param(IS_TIM_IC_POLARITY(TIM_ICInitStruct->TIM_ICPolarity));
// 2042 
// 2043   if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_1)
        LDRH     R3,[R1, #+2]
        LDRH     R4,[R1, #+4]
        LDRH     R2,[R1, #+8]
        LDRH     R5,[R1, #+0]
        CMP      R5,#+0
        BNE      ??TIM_ICInit_0
// 2044   {
// 2045     assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 2046     /* TI1 Configuration */
// 2047     TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
// 2048                TIM_ICInitStruct->TIM_ICSelection,
// 2049                TIM_ICInitStruct->TIM_ICFilter);
        LDRH     R5,[R0, #+32]
        LDR      R6,??DataTable39  ;; 0xfffe
        ANDS     R6,R6,R5
        STRH     R6,[R0, #+32]
        LDRH     R6,[R0, #+24]
        LDRH     R5,[R0, #+32]
        LDR      R7,??DataTable39_1  ;; 0xff0c
        ANDS     R7,R7,R6
        ORRS     R4,R4,R7
        LSLS     R2,R2,#+4
        ORRS     R2,R2,R4
        STRH     R2,[R0, #+24]
        LDR      R2,??DataTable39_2  ;; 0xfff5
        ANDS     R2,R2,R5
        ORRS     R3,R3,R2
        MOVS     R2,#+1
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
// 2050     /* Set the Input Capture Prescaler value */
// 2051     TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable41_1  ;; 0xfff3
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
        LDRH     R2,[R0, #+24]
        B.N      ??TIM_ICInit_1
// 2052   }
// 2053   else if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_2)
??TIM_ICInit_0:
        CMP      R5,#+4
        BNE      ??TIM_ICInit_2
// 2054   {
// 2055     assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2056     /* TI2 Configuration */
// 2057     TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
// 2058                TIM_ICInitStruct->TIM_ICSelection,
// 2059                TIM_ICInitStruct->TIM_ICFilter);
        LDRH     R5,[R0, #+32]
        LDR      R6,??DataTable41_2  ;; 0xffef
        ANDS     R6,R6,R5
        STRH     R6,[R0, #+32]
        LDRH     R7,[R0, #+24]
        LDRH     R5,[R0, #+32]
        LDR      R6,??DataTable42  ;; 0xcff
        ANDS     R6,R6,R7
        LSLS     R2,R2,#+12
        ORRS     R2,R2,R6
        LSLS     R4,R4,#+8
        ORRS     R4,R4,R2
        STRH     R4,[R0, #+24]
        LDR      R2,??DataTable42_1  ;; 0xff5f
        ANDS     R2,R2,R5
        LSLS     R3,R3,#+4
        ORRS     R3,R3,R2
        MOVS     R2,#+16
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
// 2060     /* Set the Input Capture Prescaler value */
// 2061     TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable42_2  ;; 0xf3ff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
        LDRH     R2,[R0, #+24]
        LSLS     R1,R1,#+8
??TIM_ICInit_1:
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+24]
// 2062   }
        B        ??TIM_ICInit_3
// 2063   else if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_3)
??TIM_ICInit_2:
        CMP      R5,#+8
        LDRH     R5,[R0, #+32]
        BNE      ??TIM_ICInit_4
// 2064   {
// 2065     assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2066     /* TI3 Configuration */
// 2067     TI3_Config(TIMx,  TIM_ICInitStruct->TIM_ICPolarity,
// 2068                TIM_ICInitStruct->TIM_ICSelection,
// 2069                TIM_ICInitStruct->TIM_ICFilter);
        LDR      R6,??DataTable42_3  ;; 0xfeff
        ANDS     R6,R6,R5
        STRH     R6,[R0, #+32]
        LDRH     R6,[R0, #+28]
        LDRH     R5,[R0, #+32]
        LDR      R7,??DataTable39_1  ;; 0xff0c
        ANDS     R7,R7,R6
        ORRS     R4,R4,R7
        LSLS     R2,R2,#+4
        ORRS     R2,R2,R4
        STRH     R2,[R0, #+28]
        LDR      R2,??DataTable42_4  ;; 0xf5ff
        ANDS     R2,R2,R5
        LSLS     R3,R3,#+8
        ORRS     R3,R3,R2
        MOVS     R2,#+128
        LSLS     R2,R2,#+1        ;; #+256
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
// 2070     /* Set the Input Capture Prescaler value */
// 2071     TIM_SetIC3Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable41_1  ;; 0xfff3
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+28]
        LDRH     R2,[R0, #+28]
        B        ??TIM_ICInit_5
// 2072   }
// 2073   else
// 2074   {
// 2075     assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2076     /* TI4 Configuration */
// 2077     TI4_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
// 2078                TIM_ICInitStruct->TIM_ICSelection,
// 2079                TIM_ICInitStruct->TIM_ICFilter);
??TIM_ICInit_4:
        LDR      R6,??DataTable42_5  ;; 0xefff
        ANDS     R6,R6,R5
        STRH     R6,[R0, #+32]
        LDRH     R7,[R0, #+28]
        LDRH     R5,[R0, #+32]
        LDR      R6,??DataTable42  ;; 0xcff
        ANDS     R6,R6,R7
        LSLS     R4,R4,#+8
        ORRS     R4,R4,R6
        LSLS     R2,R2,#+12
        ORRS     R2,R2,R4
        STRH     R2,[R0, #+28]
        LDR      R2,??DataTable42_6  ;; 0x5fff
        ANDS     R2,R2,R5
        LSLS     R3,R3,#+12
        ORRS     R3,R3,R2
        MOVS     R2,#+128
        LSLS     R2,R2,#+5        ;; #+4096
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
// 2080     /* Set the Input Capture Prescaler value */
// 2081     TIM_SetIC4Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable42_2  ;; 0xf3ff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+28]
        LDRH     R2,[R0, #+28]
        LSLS     R1,R1,#+8
??TIM_ICInit_5:
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+28]
// 2082   }
// 2083 }
??TIM_ICInit_3:
        POP      {R0,R4-R7}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34:
        DC32     0xffff8fff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34_1:
        DC32     0xff8f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34_2:
        DC32     0x8fff
// 2084 
// 2085 /**
// 2086   * @brief  Fills each TIM_ICInitStruct member with its default value.
// 2087   * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure which will
// 2088   *         be initialized.
// 2089   * @retval None
// 2090   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2091 void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct)
// 2092 {
// 2093   /* Set the default configuration */
// 2094   TIM_ICInitStruct->TIM_Channel = TIM_Channel_1;
TIM_ICStructInit:
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 2095   TIM_ICInitStruct->TIM_ICPolarity = TIM_ICPolarity_Rising;
        STRH     R1,[R0, #+2]
// 2096   TIM_ICInitStruct->TIM_ICSelection = TIM_ICSelection_DirectTI;
        MOVS     R2,#+1
        STRH     R2,[R0, #+4]
// 2097   TIM_ICInitStruct->TIM_ICPrescaler = TIM_ICPSC_DIV1;
        STRH     R1,[R0, #+6]
// 2098   TIM_ICInitStruct->TIM_ICFilter = 0x00;
        STRH     R1,[R0, #+8]
// 2099 }
        BX       LR               ;; return
// 2100 
// 2101 /**
// 2102   * @brief  Configures the TIM peripheral according to the specified
// 2103   *         parameters in the TIM_ICInitStruct to measure an external PWM signal.
// 2104   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 2105   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2106   * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure
// 2107   *         that contains the configuration information for the specified TIM 
// 2108   *         peripheral.
// 2109   * @retval None
// 2110   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2111 void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
// 2112 {
TIM_PWMIConfig:
        PUSH     {R4-R7}
        SUB      SP,SP,#+12
// 2113   uint16_t icoppositepolarity = TIM_ICPolarity_Rising;
        MOVS     R4,#+0
        MOVS     R5,#+1
// 2114   uint16_t icoppositeselection = TIM_ICSelection_DirectTI;
// 2115   /* Check the parameters */
// 2116   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2117   /* Select the Opposite Input Polarity */
// 2118   if (TIM_ICInitStruct->TIM_ICPolarity == TIM_ICPolarity_Rising)
        LDRH     R3,[R1, #+2]
        CMP      R3,#+0
        BNE      ??TIM_PWMIConfig_0
// 2119   {
// 2120     icoppositepolarity = TIM_ICPolarity_Falling;
        MOVS     R4,#+2
// 2121   }
// 2122   else
// 2123   {
// 2124     icoppositepolarity = TIM_ICPolarity_Rising;
// 2125   }
// 2126   /* Select the Opposite Input */
// 2127   if (TIM_ICInitStruct->TIM_ICSelection == TIM_ICSelection_DirectTI)
??TIM_PWMIConfig_0:
        LDRH     R2,[R1, #+4]
        CMP      R2,#+1
        BNE      ??TIM_PWMIConfig_1
// 2128   {
// 2129     icoppositeselection = TIM_ICSelection_IndirectTI;
        MOVS     R5,#+2
// 2130   }
// 2131   else
// 2132   {
// 2133     icoppositeselection = TIM_ICSelection_DirectTI;
// 2134   }
// 2135   if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_1)
??TIM_PWMIConfig_1:
        LDRH     R6,[R1, #+8]
        MOV      R7,SP
        STRH     R6,[R7, #+0]
        LDRH     R6,[R1, #+0]
        CMP      R6,#+0
        MOV      R6,SP
        BNE      ??TIM_PWMIConfig_2
// 2136   {
// 2137     /* TI1 Configuration */
// 2138     TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
// 2139                TIM_ICInitStruct->TIM_ICFilter);
        LDRH     R6,[R6, #+0]
        STRH     R2,[R7, #+2]
        MOV      R2,SP
        STRH     R3,[R2, #+4]
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable39  ;; 0xfffe
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+32]
        LDRH     R7,[R0, #+24]
        LDRH     R2,[R0, #+32]
        LDR      R3,??DataTable39_1  ;; 0xff0c
        ANDS     R3,R3,R7
        MOV      R7,SP
        LDRH     R7,[R7, #+2]
        ORRS     R7,R7,R3
        LSLS     R3,R6,#+4
        ORRS     R3,R3,R7
        STRH     R3,[R0, #+24]
        LDR      R3,??DataTable39_2  ;; 0xfff5
        ANDS     R3,R3,R2
        MOV      R2,SP
        LDRH     R2,[R2, #+4]
        ORRS     R2,R2,R3
        MOVS     R3,#+1
        ORRS     R3,R3,R2
        STRH     R3,[R0, #+32]
// 2140     /* Set the Input Capture Prescaler value */
// 2141     TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R2,[R1, #+6]
        LDRH     R3,[R0, #+24]
        LDR      R6,??DataTable41_1  ;; 0xfff3
        ANDS     R6,R6,R3
        STRH     R6,[R0, #+24]
        LDRH     R3,[R0, #+24]
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+24]
// 2142     /* TI2 Configuration */
// 2143     TI2_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
        LDRH     R2,[R1, #+8]
        LDRH     R3,[R0, #+32]
        LDR      R6,??DataTable41_2  ;; 0xffef
        ANDS     R6,R6,R3
        STRH     R6,[R0, #+32]
        LDRH     R7,[R0, #+24]
        LDRH     R3,[R0, #+32]
        LDR      R6,??DataTable42  ;; 0xcff
        ANDS     R6,R6,R7
        LSLS     R2,R2,#+12
        ORRS     R2,R2,R6
        LSLS     R5,R5,#+8
        ORRS     R5,R5,R2
        STRH     R5,[R0, #+24]
        LDR      R2,??DataTable42_1  ;; 0xff5f
        ANDS     R2,R2,R3
        LSLS     R3,R4,#+4
        ORRS     R3,R3,R2
        MOVS     R2,#+16
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+32]
// 2144     /* Set the Input Capture Prescaler value */
// 2145     TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable42_2  ;; 0xf3ff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
        LDRH     R2,[R0, #+24]
        LSLS     R1,R1,#+8
        B        ??TIM_PWMIConfig_3
// 2146   }
// 2147   else
// 2148   { 
// 2149     /* TI2 Configuration */
// 2150     TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
// 2151                TIM_ICInitStruct->TIM_ICFilter);
??TIM_PWMIConfig_2:
        LDRH     R7,[R7, #+0]
        STRH     R7,[R6, #+4]
        LDRH     R6,[R0, #+32]
        LDR      R7,??DataTable41_2  ;; 0xffef
        ANDS     R7,R7,R6
        STRH     R7,[R0, #+32]
        LDRH     R6,[R0, #+24]
        MOV      R7,SP
        STRH     R6,[R7, #+0]
        LDRH     R6,[R0, #+32]
        STRH     R6,[R7, #+2]
        MOV      R6,SP
        LDRH     R7,[R6, #+0]
        LDR      R6,??DataTable42  ;; 0xcff
        ANDS     R6,R6,R7
        MOV      R7,SP
        LDRH     R7,[R7, #+4]
        LSLS     R7,R7,#+12
        ORRS     R7,R7,R6
        LSLS     R2,R2,#+8
        ORRS     R2,R2,R7
        STRH     R2,[R0, #+24]
        MOV      R2,SP
        LDRH     R2,[R2, #+2]
        LDR      R6,??DataTable42_1  ;; 0xff5f
        ANDS     R6,R6,R2
        LSLS     R2,R3,#+4
        ORRS     R2,R2,R6
        MOVS     R3,#+16
        ORRS     R3,R3,R2
        STRH     R3,[R0, #+32]
// 2152     /* Set the Input Capture Prescaler value */
// 2153     TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R2,[R1, #+6]
        LDRH     R3,[R0, #+24]
        LDR      R6,??DataTable42_2  ;; 0xf3ff
        ANDS     R6,R6,R3
        STRH     R6,[R0, #+24]
        LDRH     R3,[R0, #+24]
        LSLS     R2,R2,#+8
        ORRS     R2,R2,R3
        STRH     R2,[R0, #+24]
// 2154     /* TI1 Configuration */
// 2155     TI1_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
        LDRH     R2,[R1, #+8]
        LDRH     R3,[R0, #+32]
        LDR      R6,??DataTable39  ;; 0xfffe
        ANDS     R6,R6,R3
        STRH     R6,[R0, #+32]
        LDRH     R6,[R0, #+24]
        LDRH     R3,[R0, #+32]
        LDR      R7,??DataTable39_1  ;; 0xff0c
        ANDS     R7,R7,R6
        ORRS     R5,R5,R7
        LSLS     R2,R2,#+4
        ORRS     R2,R2,R5
        STRH     R2,[R0, #+24]
        LDR      R2,??DataTable39_2  ;; 0xfff5
        ANDS     R2,R2,R3
        ORRS     R4,R4,R2
        MOVS     R2,#+1
        ORRS     R2,R2,R4
        STRH     R2,[R0, #+32]
// 2156     /* Set the Input Capture Prescaler value */
// 2157     TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
        LDRH     R1,[R1, #+6]
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable41_1  ;; 0xfff3
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
        LDRH     R2,[R0, #+24]
??TIM_PWMIConfig_3:
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+24]
// 2158   }
// 2159 }
        ADD      SP,SP,#+12
        POP      {R4-R7}
        BX       LR               ;; return
// 2160 
// 2161 /**
// 2162   * @brief  Gets the TIMx Input Capture 1 value.
// 2163   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 2164   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2165   * @retval Capture Compare 1 Register value.
// 2166   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2167 uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx)
// 2168 {
// 2169   /* Check the parameters */
// 2170   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 2171   
// 2172   /* Get the Capture 1 Register value */
// 2173   return TIMx->CCR1;
TIM_GetCapture1:
        LDR      R0,[R0, #+52]
        BX       LR               ;; return
// 2174 }
// 2175 
// 2176 /**
// 2177   * @brief  Gets the TIMx Input Capture 2 value.
// 2178   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 2179   * @retval Capture Compare 2 Register value.
// 2180   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2181 uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx)
// 2182 {
// 2183   /* Check the parameters */
// 2184   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2185   
// 2186   /* Get the Capture 2 Register value */
// 2187   return TIMx->CCR2;
TIM_GetCapture2:
        LDR      R0,[R0, #+56]
        BX       LR               ;; return
// 2188 }
// 2189 
// 2190 /**
// 2191   * @brief  Gets the TIMx Input Capture 3 value.
// 2192   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2193   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2194   * @retval Capture Compare 3 Register value.
// 2195   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2196 uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx)
// 2197 {
// 2198   /* Check the parameters */
// 2199   assert_param(IS_TIM_LIST3_PERIPH(TIMx)); 
// 2200   
// 2201   /* Get the Capture 3 Register value */
// 2202   return TIMx->CCR3;
TIM_GetCapture3:
        LDR      R0,[R0, #+60]
        BX       LR               ;; return
// 2203 }
// 2204 
// 2205 /**
// 2206   * @brief  Gets the TIMx Input Capture 4 value.
// 2207   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2208   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2209   * @retval Capture Compare 4 Register value.
// 2210   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2211 uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx)
// 2212 {
// 2213   /* Check the parameters */
// 2214   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2215   
// 2216   /* Get the Capture 4 Register value */
// 2217   return TIMx->CCR4;
TIM_GetCapture4:
        LDR      R0,[R0, #+64]
        BX       LR               ;; return
// 2218 }
// 2219 
// 2220 /**
// 2221   * @brief  Sets the TIMx Input Capture 1 prescaler.
// 2222   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 2223   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2224   * @param  TIM_ICPSC: specifies the Input Capture1 prescaler new value.
// 2225   *          This parameter can be one of the following values:
// 2226   *            @arg TIM_ICPSC_DIV1: no prescaler
// 2227   *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
// 2228   *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
// 2229   *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
// 2230   * @retval None
// 2231   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2232 void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
// 2233 {
// 2234   /* Check the parameters */
// 2235   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 2236   assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));
// 2237   
// 2238   /* Reset the IC1PSC Bits */
// 2239   TIMx->CCMR1 &= (uint16_t)~((uint16_t)TIM_CCMR1_IC1PSC);
TIM_SetIC1Prescaler:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable41_1  ;; 0xfff3
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
// 2240   /* Set the IC1PSC value */
// 2241   TIMx->CCMR1 |= TIM_ICPSC;
        LDRH     R2,[R0, #+24]
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+24]
// 2242 }
        BX       LR               ;; return
// 2243 
// 2244 /**
// 2245   * @brief  Sets the TIMx Input Capture 2 prescaler.
// 2246   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 2247   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2248   * @param  TIM_ICPSC: specifies the Input Capture2 prescaler new value.
// 2249   *          This parameter can be one of the following values:
// 2250   *            @arg TIM_ICPSC_DIV1: no prescaler
// 2251   *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
// 2252   *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
// 2253   *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
// 2254   * @retval None
// 2255   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2256 void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
// 2257 {
// 2258   /* Check the parameters */
// 2259   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2260   assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));
// 2261   
// 2262   /* Reset the IC2PSC Bits */
// 2263   TIMx->CCMR1 &= (uint16_t)~((uint16_t)TIM_CCMR1_IC2PSC);
TIM_SetIC2Prescaler:
        LDRH     R2,[R0, #+24]
        LDR      R3,??DataTable46  ;; 0xf3ff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+24]
// 2264   /* Set the IC2PSC value */
// 2265   TIMx->CCMR1 |= (uint16_t)(TIM_ICPSC << 8);
        LDRH     R2,[R0, #+24]
        LSLS     R1,R1,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+24]
// 2266 }
        BX       LR               ;; return
// 2267 
// 2268 /**
// 2269   * @brief  Sets the TIMx Input Capture 3 prescaler.
// 2270   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2271   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2272   * @param  TIM_ICPSC: specifies the Input Capture3 prescaler new value.
// 2273   *          This parameter can be one of the following values:
// 2274   *            @arg TIM_ICPSC_DIV1: no prescaler
// 2275   *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
// 2276   *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
// 2277   *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
// 2278   * @retval None
// 2279   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2280 void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
// 2281 {
// 2282   /* Check the parameters */
// 2283   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2284   assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));
// 2285   
// 2286   /* Reset the IC3PSC Bits */
// 2287   TIMx->CCMR2 &= (uint16_t)~((uint16_t)TIM_CCMR2_IC3PSC);
TIM_SetIC3Prescaler:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable41_1  ;; 0xfff3
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+28]
// 2288   /* Set the IC3PSC value */
// 2289   TIMx->CCMR2 |= TIM_ICPSC;
        LDRH     R2,[R0, #+28]
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+28]
// 2290 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable38:
        DC32     0xfffd

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable38_1:
        DC32     0xfff7
// 2291 
// 2292 /**
// 2293   * @brief  Sets the TIMx Input Capture 4 prescaler.
// 2294   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2295   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2296   * @param  TIM_ICPSC: specifies the Input Capture4 prescaler new value.
// 2297   *          This parameter can be one of the following values:
// 2298   *            @arg TIM_ICPSC_DIV1: no prescaler
// 2299   *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
// 2300   *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
// 2301   *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
// 2302   * @retval None
// 2303   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2304 void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC)
// 2305 {  
// 2306   /* Check the parameters */
// 2307   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2308   assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));
// 2309   
// 2310   /* Reset the IC4PSC Bits */
// 2311   TIMx->CCMR2 &= (uint16_t)~((uint16_t)TIM_CCMR2_IC4PSC);
TIM_SetIC4Prescaler:
        LDRH     R2,[R0, #+28]
        LDR      R3,??DataTable46  ;; 0xf3ff
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+28]
// 2312   /* Set the IC4PSC value */
// 2313   TIMx->CCMR2 |= (uint16_t)(TIM_ICPSC << 8);
        LDRH     R2,[R0, #+28]
        LSLS     R1,R1,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+28]
// 2314 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39:
        DC32     0xfffe

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_1:
        DC32     0xff0c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_2:
        DC32     0xfff5
// 2315 
// 2316 /**
// 2317   * @}
// 2318   */
// 2319 
// 2320 /** @defgroup TIM_Group5 Interrupts DMA and flags management functions
// 2321  *  @brief    Interrupts, DMA and flags management functions 
// 2322  *
// 2323 @verbatim
// 2324  ===============================================================================
// 2325           ##### Interrupts, DMA and flags management functions #####
// 2326  ===============================================================================
// 2327 
// 2328 @endverbatim
// 2329   * @{
// 2330   */
// 2331 
// 2332 /**
// 2333   * @brief  Enables or disables the specified TIM interrupts.
// 2334   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the TIMx peripheral.
// 2335   * @note   TIM7 is applicable only for STM32F072 devices
// 2336   * @note   TIM6 is not applivable for STM32F031 devices.
// 2337   * @note   TIM2 is not applicable for STM32F030 devices.
// 2338   * @param  TIM_IT: specifies the TIM interrupts sources to be enabled or disabled.
// 2339   *          This parameter can be any combination of the following values:
// 2340   *            @arg TIM_IT_Update: TIM update Interrupt source
// 2341   *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
// 2342   *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
// 2343   *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
// 2344   *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
// 2345   *            @arg TIM_IT_COM: TIM Commutation Interrupt source
// 2346   *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
// 2347   *            @arg TIM_IT_Break: TIM Break Interrupt source
// 2348   * 
// 2349   * @note   TIM6 and TIM7 can only generate an update interrupt.
// 2350   * @note   TIM15 can have only TIM_IT_Update, TIM_IT_CC1,TIM_IT_CC2 or TIM_IT_Trigger. 
// 2351   * @note   TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.   
// 2352   * @note   TIM_IT_Break is used only with TIM1 and TIM15. 
// 2353   * @note   TIM_IT_COM is used only with TIM1, TIM15, TIM16 and TIM17.
// 2354   *       
// 2355   * @param  NewState: new state of the TIM interrupts.
// 2356   *          This parameter can be: ENABLE or DISABLE.
// 2357   * @retval None
// 2358   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2359 void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
// 2360 {  
// 2361   /* Check the parameters */
// 2362   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2363   assert_param(IS_TIM_IT(TIM_IT));
// 2364   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 2365   
// 2366   if (NewState != DISABLE)
TIM_ITConfig:
        CMP      R2,#+0
        LDRH     R2,[R0, #+12]
        BEQ      ??TIM_ITConfig_0
// 2367   {
// 2368     /* Enable the Interrupt sources */
// 2369     TIMx->DIER |= TIM_IT;
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+12]
        BX       LR
// 2370   }
// 2371   else
// 2372   {
// 2373     /* Disable the Interrupt sources */
// 2374     TIMx->DIER &= (uint16_t)~TIM_IT;
??TIM_ITConfig_0:
        BICS     R2,R2,R1
        STRH     R2,[R0, #+12]
// 2375   }
// 2376 }
        BX       LR               ;; return
// 2377 
// 2378 /**
// 2379   * @brief  Configures the TIMx event to be generate by software.
// 2380   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the 
// 2381   *         TIM peripheral.
// 2382   * @note   TIM7 is applicable only for STM32F072 devices
// 2383   * @note   TIM6 is not applivable for STM32F031 devices.
// 2384   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2385   * @param  TIM_EventSource: specifies the event source.
// 2386   *          This parameter can be one or more of the following values:  
// 2387   *            @arg TIM_EventSource_Update: Timer update Event source
// 2388   *            @arg TIM_EventSource_CC1: Timer Capture Compare 1 Event source
// 2389   *            @arg TIM_EventSource_CC2: Timer Capture Compare 2 Event source
// 2390   *            @arg TIM_EventSource_CC3: Timer Capture Compare 3 Event source
// 2391   *            @arg TIM_EventSource_CC4: Timer Capture Compare 4 Event source
// 2392   *            @arg TIM_EventSource_COM: Timer COM event source  
// 2393   *            @arg TIM_EventSource_Trigger: Timer Trigger Event source
// 2394   *            @arg TIM_EventSource_Break: Timer Break event source
// 2395   *
// 2396   * @note   TIM6 and TIM7 can only generate an update event.  
// 2397   * @note   TIM_EventSource_COM and TIM_EventSource_Break are used only with TIM1.
// 2398   *             
// 2399   * @retval None
// 2400   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2401 void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource)
// 2402 { 
// 2403   /* Check the parameters */
// 2404   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2405   assert_param(IS_TIM_EVENT_SOURCE(TIM_EventSource)); 
// 2406   /* Set the event sources */
// 2407   TIMx->EGR = TIM_EventSource;
TIM_GenerateEvent:
        STRH     R1,[R0, #+20]
// 2408 }
        BX       LR               ;; return
// 2409 
// 2410 /**
// 2411   * @brief  Checks whether the specified TIM flag is set or not.
// 2412   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the TIM peripheral.
// 2413   * @note   TIM7 is applicable only for STM32F072 devices
// 2414   * @note   TIM6 is not applivable for STM32F031 devices.
// 2415   * @note   TIM2 is not applicable for STM32F030 devices.
// 2416   * @param  TIM_FLAG: specifies the flag to check.
// 2417   *          This parameter can be one of the following values:
// 2418   *            @arg TIM_FLAG_Update: TIM update Flag
// 2419   *            @arg TIM_FLAG_CC1: TIM Capture Compare 1 Flag
// 2420   *            @arg TIM_FLAG_CC2: TIM Capture Compare 2 Flag
// 2421   *            @arg TIM_FLAG_CC3: TIM Capture Compare 3 Flag
// 2422   *            @arg TIM_FLAG_CC4: TIM Capture Compare 4 Flag
// 2423   *            @arg TIM_FLAG_COM: TIM Commutation Flag
// 2424   *            @arg TIM_FLAG_Trigger: TIM Trigger Flag
// 2425   *            @arg TIM_FLAG_Break: TIM Break Flag
// 2426   *            @arg TIM_FLAG_CC1OF: TIM Capture Compare 1 overcapture Flag
// 2427   *            @arg TIM_FLAG_CC2OF: TIM Capture Compare 2 overcapture Flag
// 2428   *            @arg TIM_FLAG_CC3OF: TIM Capture Compare 3 overcapture Flag
// 2429   *            @arg TIM_FLAG_CC4OF: TIM Capture Compare 4 overcapture Flag
// 2430   *
// 2431   * @note   TIM6 and TIM7 can have only one update flag. 
// 2432   * @note   TIM15 can have only TIM_FLAG_Update, TIM_FLAG_CC1, TIM_FLAG_CC2 or TIM_FLAG_Trigger.
// 2433   * @note   TIM14, TIM16 and TIM17 can have TIM_FLAG_Update or TIM_FLAG_CC1.   
// 2434   * @note   TIM_FLAG_Break is used only with TIM1 and TIM15. 
// 2435   * @note   TIM_FLAG_COM is used only with TIM1 TIM15, TIM16 and TIM17.
// 2436   *
// 2437   * @retval The new state of TIM_FLAG (SET or RESET).
// 2438   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2439 FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
// 2440 { 
TIM_GetFlagStatus:
        MOVS     R2,R0
// 2441   ITStatus bitstatus = RESET; 
        MOVS     R0,#+0
// 2442    
// 2443   /* Check the parameters */
// 2444   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2445   assert_param(IS_TIM_GET_FLAG(TIM_FLAG));
// 2446   
// 2447   if ((TIMx->SR & TIM_FLAG) != (uint16_t)RESET)
        LDRH     R2,[R2, #+16]
        TST      R2,R1
        BEQ      ??TIM_GetFlagStatus_0
// 2448   {
// 2449     bitstatus = SET;
        MOVS     R0,#+1
// 2450   }
// 2451   else
// 2452   {
// 2453     bitstatus = RESET;
// 2454   }
// 2455   return bitstatus;
??TIM_GetFlagStatus_0:
        BX       LR               ;; return
// 2456 }
// 2457 
// 2458 /**
// 2459   * @brief  Clears the TIMx's pending flags.
// 2460   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the TIM peripheral.
// 2461   * @note   TIM7 is applicable only for STM32F072 devices
// 2462   * @note   TIM6 is not applivable for STM32F031 devices.
// 2463   * @note   TIM2 is not applicable for STM32F030 devices.
// 2464   * @param  TIM_FLAG: specifies the flag bit to clear.
// 2465   *          This parameter can be any combination of the following values:
// 2466   *            @arg TIM_FLAG_Update: TIM update Flag
// 2467   *            @arg TIM_FLAG_CC1: TIM Capture Compare 1 Flag
// 2468   *            @arg TIM_FLAG_CC2: TIM Capture Compare 2 Flag
// 2469   *            @arg TIM_FLAG_CC3: TIM Capture Compare 3 Flag
// 2470   *            @arg TIM_FLAG_CC4: TIM Capture Compare 4 Flag
// 2471   *            @arg TIM_FLAG_COM: TIM Commutation Flag
// 2472   *            @arg TIM_FLAG_Trigger: TIM Trigger Flag
// 2473   *            @arg TIM_FLAG_Break: TIM Break Flag
// 2474   *            @arg TIM_FLAG_CC1OF: TIM Capture Compare 1 overcapture Flag
// 2475   *            @arg TIM_FLAG_CC2OF: TIM Capture Compare 2 overcapture Flag
// 2476   *            @arg TIM_FLAG_CC3OF: TIM Capture Compare 3 overcapture Flag
// 2477   *            @arg TIM_FLAG_CC4OF: TIM Capture Compare 4 overcapture Flag
// 2478   *
// 2479   * @note   TIM6 and TIM7 can have only one update flag. 
// 2480   * @note   TIM15 can have only TIM_FLAG_Update, TIM_FLAG_CC1,TIM_FLAG_CC2 or 
// 2481   *         TIM_FLAG_Trigger. 
// 2482   * @note   TIM14, TIM16 and TIM17 can have TIM_FLAG_Update or TIM_FLAG_CC1.   
// 2483   * @note   TIM_FLAG_Break is used only with TIM1 and TIM15. 
// 2484   * @note   TIM_FLAG_COM is used only with TIM1, TIM15, TIM16 and TIM17.
// 2485   *
// 2486   * @retval None
// 2487   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2488 void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
// 2489 {  
// 2490   /* Check the parameters */
// 2491   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2492   assert_param(IS_TIM_CLEAR_FLAG(TIM_FLAG));
// 2493    
// 2494   /* Clear the flags */
// 2495   TIMx->SR = (uint16_t)~TIM_FLAG;
TIM_ClearFlag:
        MVNS     R2,R1
        STRH     R2,[R0, #+16]
// 2496 }
        BX       LR               ;; return
// 2497 
// 2498 /**
// 2499   * @brief  Checks whether the TIM interrupt has occurred or not.
// 2500   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the TIM peripheral.
// 2501   * @note   TIM7 is applicable only for STM32F072 devices
// 2502   * @note   TIM6 is not applivable for STM32F031 devices.
// 2503   * @note   TIM2 is not applicable for STM32F030 devices.
// 2504   * @param  TIM_IT: specifies the TIM interrupt source to check.
// 2505   *          This parameter can be one of the following values:
// 2506   *            @arg TIM_IT_Update: TIM update Interrupt source
// 2507   *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
// 2508   *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
// 2509   *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
// 2510   *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
// 2511   *            @arg TIM_IT_COM: TIM Commutation Interrupt source
// 2512   *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
// 2513   *            @arg TIM_IT_Break: TIM Break Interrupt source
// 2514   *
// 2515   * @note   TIM6 and TIM7 can generate only an update interrupt.
// 2516   * @note   TIM15 can have only TIM_IT_Update, TIM_IT_CC1, TIM_IT_CC2 or TIM_IT_Trigger. 
// 2517   * @note   TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.   
// 2518   * @note   TIM_IT_Break is used only with TIM1 and TIM15. 
// 2519   * @note   TIM_IT_COM is used only with TIM1, TIM15, TIM16 and TIM17.
// 2520   *
// 2521   * @retval The new state of the TIM_IT(SET or RESET).
// 2522   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2523 ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
// 2524 {
TIM_GetITStatus:
        MOVS     R2,R0
// 2525   ITStatus bitstatus = RESET;  
        MOVS     R0,#+0
// 2526   uint16_t itstatus = 0x0, itenable = 0x0;
// 2527   
// 2528   /* Check the parameters */
// 2529   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2530   assert_param(IS_TIM_GET_IT(TIM_IT));
// 2531    
// 2532   itstatus = TIMx->SR & TIM_IT;
        LDRH     R3,[R2, #+16]
// 2533   
// 2534   itenable = TIMx->DIER & TIM_IT;
        LDRH     R2,[R2, #+12]
        ANDS     R2,R2,R1
// 2535   if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
        TST      R3,R1
        BEQ      ??TIM_GetITStatus_0
        CMP      R2,#+0
        BEQ      ??TIM_GetITStatus_0
// 2536   {
// 2537     bitstatus = SET;
        MOVS     R0,#+1
// 2538   }
// 2539   else
// 2540   {
// 2541     bitstatus = RESET;
// 2542   }
// 2543   return bitstatus;
??TIM_GetITStatus_0:
        BX       LR               ;; return
// 2544 }
// 2545 
// 2546 /**
// 2547   * @brief  Clears the TIMx's interrupt pending bits.
// 2548   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 14, 15, 16 or 17 to select the TIM peripheral.
// 2549   * @note   TIM7 is applicable only for STM32F072 devices
// 2550   * @note   TIM6 is not applivable for STM32F031 devices.
// 2551   * @note   TIM2 is not applicable for STM32F030 devices.
// 2552   * @param  TIM_IT: specifies the pending bit to clear.
// 2553   *          This parameter can be any combination of the following values:
// 2554   *            @arg TIM_IT_Update: TIM1 update Interrupt source
// 2555   *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
// 2556   *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
// 2557   *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
// 2558   *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
// 2559   *            @arg TIM_IT_COM: TIM Commutation Interrupt source
// 2560   *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
// 2561   *            @arg TIM_IT_Break: TIM Break Interrupt source
// 2562   *
// 2563   * @note   TIM6 and TIM7 can generate only an update interrupt.
// 2564   * @note   TIM15 can have only TIM_IT_Update, TIM_IT_CC1, TIM_IT_CC2 or TIM_IT_Trigger. 
// 2565   * @note   TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.   
// 2566   * @note   TIM_IT_Break is used only with TIM1 and TIM15. 
// 2567   * @note   TIM_IT_COM is used only with TIM1, TIM15, TIM16 and TIM17.
// 2568   *
// 2569   * @retval None
// 2570   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2571 void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
// 2572 {
// 2573   /* Check the parameters */
// 2574   assert_param(IS_TIM_ALL_PERIPH(TIMx));
// 2575   assert_param(IS_TIM_IT(TIM_IT));
// 2576    
// 2577   /* Clear the IT pending Bit */
// 2578   TIMx->SR = (uint16_t)~TIM_IT;
TIM_ClearITPendingBit:
        MVNS     R2,R1
        STRH     R2,[R0, #+16]
// 2579 }
        BX       LR               ;; return
// 2580 
// 2581 /**
// 2582   * @brief  Configures the TIMx's DMA interface.
// 2583   * @param  TIMx: where x can be 1, 2, 3, 15, 16 or 17  to select the TIM peripheral.
// 2584   * @note   TIM2 is not applicable for STM32F030 devices.
// 2585   * @param  TIM_DMABase: DMA Base address.
// 2586   *          This parameter can be one of the following values:
// 2587   *            @arg TIM_DMABase_CR1
// 2588   *            @arg TIM_DMABase_CR2
// 2589   *            @arg TIM_DMABase_SMCR
// 2590   *            @arg TIM_DMABase_DIER
// 2591   *            @arg TIM_DMABase_SR
// 2592   *            @arg TIM_DMABase_EGR
// 2593   *            @arg TIM_DMABase_CCMR1
// 2594   *            @arg TIM_DMABase_CCMR2
// 2595   *            @arg TIM_DMABase_CCER
// 2596   *            @arg TIM_DMABase_CNT
// 2597   *            @arg TIM_DMABase_PSC
// 2598   *            @arg TIM_DMABase_ARR
// 2599   *            @arg TIM_DMABase_CCR1
// 2600   *            @arg TIM_DMABase_CCR2
// 2601   *            @arg TIM_DMABase_CCR3 
// 2602   *            @arg TIM_DMABase_CCR4
// 2603   *            @arg TIM_DMABase_DCR
// 2604   *            @arg TIM_DMABase_OR
// 2605   * @param  TIM_DMABurstLength: DMA Burst length. This parameter can be one value
// 2606   *         between: TIM_DMABurstLength_1Transfer and TIM_DMABurstLength_18Transfers.
// 2607   * @retval None
// 2608   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2609 void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength)
// 2610 {
// 2611   /* Check the parameters */
// 2612   assert_param(IS_TIM_LIST4_PERIPH(TIMx));
// 2613   assert_param(IS_TIM_DMA_BASE(TIM_DMABase)); 
// 2614   assert_param(IS_TIM_DMA_LENGTH(TIM_DMABurstLength));
// 2615   /* Set the DMA Base and the DMA Burst Length */
// 2616   TIMx->DCR = TIM_DMABase | TIM_DMABurstLength;
TIM_DMAConfig:
        ORRS     R2,R2,R1
        ADDS     R0,R0,#+72
        STRH     R2,[R0, #+0]
// 2617 }
        BX       LR               ;; return
// 2618 
// 2619 /**
// 2620   * @brief  Enables or disables the TIMx's DMA Requests.
// 2621   * @param  TIMx: where x can be 1, 2, 3, 6, 7, 15, 16 or 17 to select the TIM peripheral. 
// 2622   * @note   TIM7 is applicable only for STM32F072 devices
// 2623   * @note   TIM6 is not applivable for STM32F031 devices.
// 2624   * @note   TIM2 is not applicable for STM32F030 devices.
// 2625   * @param  TIM_DMASource: specifies the DMA Request sources.
// 2626   *          This parameter can be any combination of the following values:
// 2627   *            @arg TIM_DMA_Update: TIM update Interrupt source
// 2628   *            @arg TIM_DMA_CC1: TIM Capture Compare 1 DMA source
// 2629   *            @arg TIM_DMA_CC2: TIM Capture Compare 2 DMA source
// 2630   *            @arg TIM_DMA_CC3: TIM Capture Compare 3 DMA source
// 2631   *            @arg TIM_DMA_CC4: TIM Capture Compare 4 DMA source
// 2632   *            @arg TIM_DMA_COM: TIM Commutation DMA source
// 2633   *            @arg TIM_DMA_Trigger: TIM Trigger DMA source
// 2634   * @param  NewState: new state of the DMA Request sources.
// 2635   *          This parameter can be: ENABLE or DISABLE.
// 2636   * @retval None
// 2637   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2638 void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
// 2639 { 
// 2640   /* Check the parameters */
// 2641   assert_param(IS_TIM_LIST10_PERIPH(TIMx));
// 2642   assert_param(IS_TIM_DMA_SOURCE(TIM_DMASource));
// 2643   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 2644   
// 2645   if (NewState != DISABLE)
TIM_DMACmd:
        CMP      R2,#+0
        LDRH     R2,[R0, #+12]
        BEQ      ??TIM_DMACmd_0
// 2646   {
// 2647     /* Enable the DMA sources */
// 2648     TIMx->DIER |= TIM_DMASource; 
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+12]
        BX       LR
// 2649   }
// 2650   else
// 2651   {
// 2652     /* Disable the DMA sources */
// 2653     TIMx->DIER &= (uint16_t)~TIM_DMASource;
??TIM_DMACmd_0:
        BICS     R2,R2,R1
        STRH     R2,[R0, #+12]
// 2654   }
// 2655 }
        BX       LR               ;; return
// 2656 
// 2657 /**
// 2658   * @brief  Selects the TIMx peripheral Capture Compare DMA source.
// 2659   * @param  TIMx: where x can be 1, 2, 3, 15, 16 or 17  to select the TIM peripheral.
// 2660   * @note   TIM2 is not applicable for STM32F030 devices.
// 2661   * @param  NewState: new state of the Capture Compare DMA source
// 2662   *          This parameter can be: ENABLE or DISABLE.
// 2663   * @retval None
// 2664   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2665 void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState)
// 2666 {
// 2667   /* Check the parameters */
// 2668   assert_param(IS_TIM_LIST5_PERIPH(TIMx));
// 2669   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 2670   
// 2671   if (NewState != DISABLE)
TIM_SelectCCDMA:
        CMP      R1,#+0
        LDRH     R1,[R0, #+4]
        BEQ      ??TIM_SelectCCDMA_0
// 2672   {
// 2673     /* Set the CCDS Bit */
// 2674     TIMx->CR2 |= TIM_CR2_CCDS;
        MOVS     R2,#+8
        ORRS     R2,R2,R1
        B        ??TIM_SelectCCDMA_1
// 2675   }
// 2676   else
// 2677   {
// 2678     /* Reset the CCDS Bit */
// 2679     TIMx->CR2 &= (uint16_t)~((uint16_t)TIM_CR2_CCDS);
??TIM_SelectCCDMA_0:
        LDR      R2,??DataTable48  ;; 0xfff7
        ANDS     R2,R2,R1
??TIM_SelectCCDMA_1:
        STRH     R2,[R0, #+4]
// 2680   }
// 2681 }
        BX       LR               ;; return
// 2682 
// 2683 /**
// 2684   * @}
// 2685   */
// 2686 
// 2687 /** @defgroup TIM_Group6 Clocks management functions
// 2688  *  @brief    Clocks management functions
// 2689  *
// 2690 @verbatim
// 2691  ===============================================================================
// 2692                      ##### Clocks management functions #####
// 2693  ===============================================================================
// 2694 
// 2695 @endverbatim
// 2696   * @{
// 2697   */
// 2698 
// 2699 /**
// 2700   * @brief  Configures the TIMx internal Clock
// 2701   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 2702   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2703   * @retval None
// 2704   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2705 void TIM_InternalClockConfig(TIM_TypeDef* TIMx)
// 2706 {
// 2707   /* Check the parameters */
// 2708   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2709   /* Disable slave mode to clock the prescaler directly with the internal clock */
// 2710   TIMx->SMCR &=  (uint16_t)(~((uint16_t)TIM_SMCR_SMS));
TIM_InternalClockConfig:
        LDRH     R1,[R0, #+8]
        LDR      R2,??DataTable48_1  ;; 0xfff8
        ANDS     R2,R2,R1
        STRH     R2,[R0, #+8]
// 2711 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41:
        DC32     0xfffb

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_1:
        DC32     0xfff3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_2:
        DC32     0xffef
// 2712 
// 2713 /**
// 2714   * @brief  Configures the TIMx Internal Trigger as External Clock
// 2715   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 2716   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2717   * @param  TIM_ITRSource: Trigger source.
// 2718   *          This parameter can be one of the following values:
// 2719   *            @arg  TIM_TS_ITR0: Internal Trigger 0
// 2720   *            @arg  TIM_TS_ITR1: Internal Trigger 1
// 2721   *            @arg  TIM_TS_ITR2: Internal Trigger 2
// 2722   *            @arg  TIM_TS_ITR3: Internal Trigger 3
// 2723   * @retval None
// 2724   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2725 void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource)
// 2726 {
// 2727   /* Check the parameters */
// 2728   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2729   assert_param(IS_TIM_INTERNAL_TRIGGER_SELECTION(TIM_InputTriggerSource));
// 2730   /* Select the Internal Trigger */
// 2731   TIM_SelectInputTrigger(TIMx, TIM_InputTriggerSource);
TIM_ITRxExternalClockConfig:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable48_2  ;; 0xff8f
        ANDS     R3,R3,R2
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+8]
// 2732   /* Select the External clock mode1 */
// 2733   TIMx->SMCR |= TIM_SlaveMode_External1;
        LDRH     R1,[R0, #+8]
        MOVS     R2,#+7
        ORRS     R2,R2,R1
        STRH     R2,[R0, #+8]
// 2734 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42:
        DC32     0xcff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_1:
        DC32     0xff5f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_2:
        DC32     0xf3ff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_3:
        DC32     0xfeff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_4:
        DC32     0xf5ff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_5:
        DC32     0xefff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_6:
        DC32     0x5fff
// 2735 
// 2736 /**
// 2737   * @brief  Configures the TIMx Trigger as External Clock
// 2738   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 2739   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2740   * @param  TIM_TIxExternalCLKSource: Trigger source.
// 2741   *          This parameter can be one of the following values:
// 2742   *            @arg TIM_TIxExternalCLK1Source_TI1ED: TI1 Edge Detector
// 2743   *            @arg TIM_TIxExternalCLK1Source_TI1: Filtered Timer Input 1
// 2744   *            @arg TIM_TIxExternalCLK1Source_TI2: Filtered Timer Input 2
// 2745   * @param  TIM_ICPolarity: specifies the TIx Polarity.
// 2746   *          This parameter can be one of the following values:
// 2747   *            @arg TIM_ICPolarity_Rising
// 2748   *            @arg TIM_ICPolarity_Falling
// 2749   * @param  ICFilter: specifies the filter value.
// 2750   *          This parameter must be a value between 0x0 and 0xF.
// 2751   * @retval None
// 2752   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2753 void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
// 2754                                 uint16_t TIM_ICPolarity, uint16_t ICFilter)
// 2755 {
TIM_TIxExternalClockConfig:
        PUSH     {R3-R7}
// 2756   /* Check the parameters */
// 2757   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2758   assert_param(IS_TIM_IC_POLARITY(TIM_ICPolarity));
// 2759   assert_param(IS_TIM_IC_FILTER(ICFilter));
// 2760   
// 2761   /* Configure the Timer Input Clock Source */
// 2762   if (TIM_TIxExternalCLKSource == TIM_TIxExternalCLK1Source_TI2)
        CMP      R1,#+96
        LDRH     R4,[R0, #+32]
        BNE      ??TIM_TIxExternalClockConfig_0
// 2763   {
// 2764     TI2_Config(TIMx, TIM_ICPolarity, TIM_ICSelection_DirectTI, ICFilter);
        LDR      R5,??DataTable48_3  ;; 0xffef
        ANDS     R5,R5,R4
        STRH     R5,[R0, #+32]
        LDRH     R5,[R0, #+24]
        LDRH     R4,[R0, #+32]
        LDR      R6,??DataTable48_4  ;; 0xcff
        ANDS     R6,R6,R5
        LSLS     R3,R3,#+12
        ORRS     R3,R3,R6
        MOVS     R5,#+128
        LSLS     R5,R5,#+1        ;; #+256
        ORRS     R5,R5,R3
        STRH     R5,[R0, #+24]
        LDR      R3,??DataTable48_5  ;; 0xff5f
        ANDS     R3,R3,R4
        LSLS     R2,R2,#+4
        ORRS     R2,R2,R3
        MOVS     R3,#+16
        ORRS     R3,R3,R2
        STRH     R3,[R0, #+32]
// 2765   }
        B        ??TIM_TIxExternalClockConfig_1
// 2766   else
// 2767   {
// 2768     TI1_Config(TIMx, TIM_ICPolarity, TIM_ICSelection_DirectTI, ICFilter);
??TIM_TIxExternalClockConfig_0:
        LDR      R5,??DataTable48_6  ;; 0xfffe
        ANDS     R5,R5,R4
        STRH     R5,[R0, #+32]
        LDRH     R7,[R0, #+24]
        LDRH     R4,[R0, #+32]
        MOVS     R5,#+1
        LDR      R6,??DataTable48_7  ;; 0xff0c
        ANDS     R6,R6,R7
        LSLS     R3,R3,#+4
        ORRS     R3,R3,R6
        ORRS     R3,R3,R5
        STRH     R3,[R0, #+24]
        LDR      R3,??DataTable48_8  ;; 0xfff5
        ANDS     R3,R3,R4
        ORRS     R2,R2,R3
        ORRS     R5,R5,R2
        STRH     R5,[R0, #+32]
// 2769   }
// 2770   /* Select the Trigger source */
// 2771   TIM_SelectInputTrigger(TIMx, TIM_TIxExternalCLKSource);
??TIM_TIxExternalClockConfig_1:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable48_2  ;; 0xff8f
        ANDS     R3,R3,R2
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+8]
// 2772   /* Select the External clock mode1 */
// 2773   TIMx->SMCR |= TIM_SlaveMode_External1;
        LDRH     R1,[R0, #+8]
        MOVS     R2,#+7
        ORRS     R2,R2,R1
        STRH     R2,[R0, #+8]
// 2774 }
        POP      {R0,R4-R7}
        BX       LR               ;; return
// 2775 
// 2776 /**
// 2777   * @brief  Configures the External clock Mode1
// 2778   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2779   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2780   * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
// 2781   *          This parameter can be one of the following values:
// 2782   *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
// 2783   *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
// 2784   *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
// 2785   *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
// 2786   * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
// 2787   *          This parameter can be one of the following values:
// 2788   *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
// 2789   *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
// 2790   * @param  ExtTRGFilter: External Trigger Filter.
// 2791   *          This parameter must be a value between 0x00 and 0x0F
// 2792   * @retval None
// 2793   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2794 void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
// 2795                              uint16_t ExtTRGFilter)
// 2796 {
TIM_ETRClockMode1Config:
        PUSH     {R4}
// 2797   uint16_t tmpsmcr = 0;
// 2798   
// 2799   /* Check the parameters */
// 2800   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2801   assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
// 2802   assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
// 2803   assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));
// 2804   
// 2805   /* Configure the ETR Clock source */
// 2806   TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);
        LDRH     R4,[R0, #+8]
        UXTB     R4,R4
        ORRS     R1,R1,R4
        ORRS     R2,R2,R1
        LSLS     R1,R3,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+8]
// 2807   
// 2808   /* Get the TIMx SMCR register value */
// 2809   tmpsmcr = TIMx->SMCR;
// 2810   /* Reset the SMS Bits */
// 2811   tmpsmcr &= (uint16_t)(~((uint16_t)TIM_SMCR_SMS));
// 2812   /* Select the External clock mode1 */
// 2813   tmpsmcr |= TIM_SlaveMode_External1;
// 2814   /* Select the Trigger selection : ETRF */
// 2815   tmpsmcr &= (uint16_t)(~((uint16_t)TIM_SMCR_TS));
// 2816   tmpsmcr |= TIM_TS_ETRF;
// 2817   /* Write to TIMx SMCR */
// 2818   TIMx->SMCR = tmpsmcr;
        LDRH     R1,[R0, #+8]
        MOVS     R2,#+119
        ORRS     R2,R2,R1
        STRH     R2,[R0, #+8]
// 2819 }
        POP      {R4}
        BX       LR               ;; return
// 2820 
// 2821 /**
// 2822   * @brief  Configures the External clock Mode2
// 2823   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 2824   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2825   * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
// 2826   *          This parameter can be one of the following values:
// 2827   *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
// 2828   *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
// 2829   *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
// 2830   *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
// 2831   * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
// 2832   *          This parameter can be one of the following values:
// 2833   *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
// 2834   *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
// 2835   * @param  ExtTRGFilter: External Trigger Filter.
// 2836   *          This parameter must be a value between 0x00 and 0x0F
// 2837   * @retval None
// 2838   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2839 void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
// 2840                              uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter)
// 2841 {
TIM_ETRClockMode2Config:
        PUSH     {R4}
// 2842   /* Check the parameters */
// 2843   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 2844   assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
// 2845   assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
// 2846   assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));
// 2847   
// 2848   /* Configure the ETR Clock source */
// 2849   TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);
        LDRH     R4,[R0, #+8]
        UXTB     R4,R4
        ORRS     R1,R1,R4
        ORRS     R2,R2,R1
        LSLS     R1,R3,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+8]
// 2850   /* Enable the External clock mode2 */
// 2851   TIMx->SMCR |= TIM_SMCR_ECE;
        LDRH     R1,[R0, #+8]
        MOVS     R2,#+128
        LSLS     R2,R2,#+7        ;; #+16384
        ORRS     R2,R2,R1
        STRH     R2,[R0, #+8]
// 2852 }
        POP      {R4}
        BX       LR               ;; return
// 2853 
// 2854 /**
// 2855   * @}
// 2856   */
// 2857 
// 2858 /** @defgroup TIM_Group7 Synchronization management functions
// 2859  *  @brief    Synchronization management functions 
// 2860  *
// 2861 @verbatim
// 2862  ===============================================================================
// 2863                ##### Synchronization management functions #####
// 2864  ===============================================================================
// 2865         *** TIM Driver: how to use it in synchronization Mode ***
// 2866  ===============================================================================
// 2867     [..] Case of two/several Timers
// 2868          (#) Configure the Master Timers using the following functions:
// 2869              (++) void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx,
// 2870                   uint16_t TIM_TRGOSource).
// 2871              (++) void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx,
// 2872                   uint16_t TIM_MasterSlaveMode);  
// 2873          (#) Configure the Slave Timers using the following functions: 
// 2874              (++) void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, 
// 2875                   uint16_t TIM_InputTriggerSource);  
// 2876              (++) void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
// 2877     [..] Case of Timers and external trigger(ETR pin)
// 2878          (#) Configure the Etrenal trigger using this function:
// 2879              (++) void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler,
// 2880                   uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
// 2881          (#) Configure the Slave Timers using the following functions:
// 2882              (++) void TIM_SelectInputTrigger(TIM_TypeDef* TIMx,
// 2883                   uint16_t TIM_InputTriggerSource);
// 2884              (++) void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
// 2885 
// 2886 @endverbatim
// 2887   * @{
// 2888   */
// 2889 /**
// 2890   * @brief  Selects the Input Trigger source
// 2891   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 2892   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2893   * @param  TIM_InputTriggerSource: The Input Trigger source.
// 2894   *          This parameter can be one of the following values:
// 2895   *            @arg TIM_TS_ITR0: Internal Trigger 0
// 2896   *            @arg TIM_TS_ITR1: Internal Trigger 1
// 2897   *            @arg TIM_TS_ITR2: Internal Trigger 2
// 2898   *            @arg TIM_TS_ITR3: Internal Trigger 3
// 2899   *            @arg TIM_TS_TI1F_ED: TI1 Edge Detector
// 2900   *            @arg TIM_TS_TI1FP1: Filtered Timer Input 1
// 2901   *            @arg TIM_TS_TI2FP2: Filtered Timer Input 2
// 2902   *            @arg TIM_TS_ETRF: External Trigger input
// 2903   * @retval None
// 2904   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2905 void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource)
// 2906 {
// 2907   uint16_t tmpsmcr = 0;
// 2908 
// 2909   /* Check the parameters */
// 2910   assert_param(IS_TIM_LIST6_PERIPH(TIMx)); 
// 2911   assert_param(IS_TIM_TRIGGER_SELECTION(TIM_InputTriggerSource));
// 2912 
// 2913   /* Get the TIMx SMCR register value */
// 2914   tmpsmcr = TIMx->SMCR;
// 2915   /* Reset the TS Bits */
// 2916   tmpsmcr &= (uint16_t)(~((uint16_t)TIM_SMCR_TS));
// 2917   /* Set the Input Trigger source */
// 2918   tmpsmcr |= TIM_InputTriggerSource;
// 2919   /* Write to TIMx SMCR */
// 2920   TIMx->SMCR = tmpsmcr;
TIM_SelectInputTrigger:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable48_2  ;; 0xff8f
        ANDS     R3,R3,R2
        ORRS     R1,R1,R3
        STRH     R1,[R0, #+8]
// 2921 }
        BX       LR               ;; return
// 2922 
// 2923 /**
// 2924   * @brief  Selects the TIMx Trigger Output Mode.
// 2925   * @param  TIMx: where x can be 1, 2, 3, 6, 7, or 15 to select the TIM peripheral.
// 2926   * @note   TIM7 is applicable only for STM32F072 devices
// 2927   * @note   TIM6 is not applivable for STM32F031 devices.
// 2928   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2929   * @param  TIM_TRGOSource: specifies the Trigger Output source.
// 2930   *          This parameter can be one of the following values:
// 2931   *
// 2932   *   - For all TIMx
// 2933   *            @arg TIM_TRGOSource_Reset:  The UG bit in the TIM_EGR register is used as the trigger output (TRGO).
// 2934   *            @arg TIM_TRGOSource_Enable: The Counter Enable CEN is used as the trigger output (TRGO).
// 2935   *            @arg TIM_TRGOSource_Update: The update event is selected as the trigger output (TRGO).
// 2936   *
// 2937   *   - For all TIMx except TIM6 and TIM7
// 2938   *            @arg TIM_TRGOSource_OC1: The trigger output sends a positive pulse when the CC1IF flag
// 2939   *                                     is to be set, as soon as a capture or compare match occurs (TRGO).
// 2940   *            @arg TIM_TRGOSource_OC1Ref: OC1REF signal is used as the trigger output (TRGO).
// 2941   *            @arg TIM_TRGOSource_OC2Ref: OC2REF signal is used as the trigger output (TRGO).
// 2942   *            @arg TIM_TRGOSource_OC3Ref: OC3REF signal is used as the trigger output (TRGO).
// 2943   *            @arg TIM_TRGOSource_OC4Ref: OC4REF signal is used as the trigger output (TRGO).
// 2944   *
// 2945   * @retval None
// 2946   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2947 void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource)
// 2948 {
// 2949   /* Check the parameters */
// 2950   assert_param(IS_TIM_LIST9_PERIPH(TIMx));
// 2951   assert_param(IS_TIM_TRGO_SOURCE(TIM_TRGOSource));
// 2952 
// 2953   /* Reset the MMS Bits */
// 2954   TIMx->CR2 &= (uint16_t)~((uint16_t)TIM_CR2_MMS);
TIM_SelectOutputTrigger:
        LDRH     R2,[R0, #+4]
        LDR      R3,??DataTable48_2  ;; 0xff8f
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+4]
// 2955   /* Select the TRGO source */
// 2956   TIMx->CR2 |=  TIM_TRGOSource;
        LDRH     R2,[R0, #+4]
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+4]
// 2957 }
        BX       LR               ;; return
// 2958 
// 2959 /**
// 2960   * @brief  Selects the TIMx Slave Mode.
// 2961   * @param  TIMx: where x can be 1, 2, 3 or 15 to select the TIM peripheral.
// 2962   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2963   * @param  TIM_SlaveMode: specifies the Timer Slave Mode.
// 2964   *          This parameter can be one of the following values:
// 2965   *            @arg TIM_SlaveMode_Reset: Rising edge of the selected trigger signal (TRGI) re-initializes
// 2966   *                                      the counter and triggers an update of the registers.
// 2967   *            @arg TIM_SlaveMode_Gated:     The counter clock is enabled when the trigger signal (TRGI) is high.
// 2968   *            @arg TIM_SlaveMode_Trigger:   The counter starts at a rising edge of the trigger TRGI.
// 2969   *            @arg TIM_SlaveMode_External1: Rising edges of the selected trigger (TRGI) clock the counter.
// 2970   * @retval None
// 2971   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2972 void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode)
// 2973 {
// 2974   /* Check the parameters */
// 2975   assert_param(IS_TIM_LIST6_PERIPH(TIMx)); 
// 2976   assert_param(IS_TIM_SLAVE_MODE(TIM_SlaveMode));
// 2977   
// 2978   /* Reset the SMS Bits */
// 2979   TIMx->SMCR &= (uint16_t)~((uint16_t)TIM_SMCR_SMS);
TIM_SelectSlaveMode:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable48_1  ;; 0xfff8
        REQUIRE ?Subroutine2
        ;; // Fall through to label ?Subroutine2
// 2980   /* Select the Slave Mode */
// 2981   TIMx->SMCR |= TIM_SlaveMode;
// 2982 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine2:
        ANDS     R3,R3,R2
        STRH     R3,[R0, #+8]
        LDRH     R2,[R0, #+8]
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+8]
        BX       LR               ;; return
// 2983 
// 2984 /**
// 2985   * @brief  Sets or Resets the TIMx Master/Slave Mode.
// 2986   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 2987   * @note   TIM2 is not applicable for STM32F030 devices.  
// 2988   * @param  TIM_MasterSlaveMode: specifies the Timer Master Slave Mode.
// 2989   *          This parameter can be one of the following values:
// 2990   *            @arg TIM_MasterSlaveMode_Enable: synchronization between the current timer
// 2991   *                                             and its slaves (through TRGO).
// 2992   *            @arg TIM_MasterSlaveMode_Disable: No action
// 2993   * @retval None
// 2994   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2995 void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode)
// 2996 {
// 2997   /* Check the parameters */
// 2998   assert_param(IS_TIM_LIST6_PERIPH(TIMx));
// 2999   assert_param(IS_TIM_MSM_STATE(TIM_MasterSlaveMode));
// 3000   
// 3001   /* Reset the MSM Bit */
// 3002   TIMx->SMCR &= (uint16_t)~((uint16_t)TIM_SMCR_MSM);
TIM_SelectMasterSlaveMode:
        LDRH     R2,[R0, #+8]
        LDR      R3,??DataTable48_9  ;; 0xff7f
        B.N      ?Subroutine2
// 3003   
// 3004   /* Set or Reset the MSM Bit */
// 3005   TIMx->SMCR |= TIM_MasterSlaveMode;
// 3006 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46:
        DC32     0xf3ff
// 3007 
// 3008 /**
// 3009   * @brief  Configures the TIMx External Trigger (ETR).
// 3010   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 3011   * @note   TIM2 is not applicable for STM32F030 devices.   
// 3012   * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
// 3013   *          This parameter can be one of the following values:
// 3014   *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
// 3015   *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
// 3016   *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
// 3017   *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
// 3018   * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
// 3019   *          This parameter can be one of the following values:
// 3020   *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
// 3021   *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
// 3022   * @param  ExtTRGFilter: External Trigger Filter.
// 3023   *          This parameter must be a value between 0x00 and 0x0F
// 3024   * @retval None
// 3025   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 3026 void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
// 3027                    uint16_t ExtTRGFilter)
// 3028 {
TIM_ETRConfig:
        PUSH     {R4}
// 3029   uint16_t tmpsmcr = 0;
// 3030   
// 3031   /* Check the parameters */
// 3032   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 3033   assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
// 3034   assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
// 3035   assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));
// 3036   
// 3037   tmpsmcr = TIMx->SMCR;
// 3038   /* Reset the ETR Bits */
// 3039   tmpsmcr &= SMCR_ETR_MASK;
// 3040   /* Set the Prescaler, the Filter value and the Polarity */
// 3041   tmpsmcr |= (uint16_t)(TIM_ExtTRGPrescaler | (uint16_t)(TIM_ExtTRGPolarity | (uint16_t)(ExtTRGFilter << (uint16_t)8)));
// 3042   /* Write to TIMx SMCR */
// 3043   TIMx->SMCR = tmpsmcr;
        LDRH     R4,[R0, #+8]
        UXTB     R4,R4
        ORRS     R1,R1,R4
        ORRS     R2,R2,R1
        LSLS     R1,R3,#+8
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+8]
// 3044 }
        POP      {R4}
        BX       LR               ;; return
// 3045 
// 3046 /**
// 3047   * @}
// 3048   */
// 3049 
// 3050 /** @defgroup TIM_Group8 Specific interface management functions
// 3051  *  @brief    Specific interface management functions 
// 3052  *
// 3053 @verbatim
// 3054  ===============================================================================
// 3055              ##### Specific interface management functions #####
// 3056  ===============================================================================
// 3057 
// 3058 @endverbatim
// 3059   * @{
// 3060   */
// 3061 
// 3062 /**
// 3063   * @brief  Configures the TIMx Encoder Interface.
// 3064   * @param  TIMx: where x can be  1, 2 or 3 to select the TIM peripheral.
// 3065   * @note   TIM2 is not applicable for STM32F030 devices.   
// 3066   * @param  TIM_EncoderMode: specifies the TIMx Encoder Mode.
// 3067   *          This parameter can be one of the following values:
// 3068   *            @arg TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
// 3069   *            @arg TIM_EncoderMode_TI2: Counter counts on TI2FP2 edge depending on TI1FP1 level.
// 3070   *            @arg TIM_EncoderMode_TI12: Counter counts on both TI1FP1 and TI2FP2 edges depending
// 3071   *                                       on the level of the other input.
// 3072   * @param  TIM_IC1Polarity: specifies the IC1 Polarity
// 3073   *          This parmeter can be one of the following values:
// 3074   *            @arg TIM_ICPolarity_Falling: IC Falling edge.
// 3075   *            @arg TIM_ICPolarity_Rising: IC Rising edge.
// 3076   * @param  TIM_IC2Polarity: specifies the IC2 Polarity
// 3077   *          This parmeter can be one of the following values:
// 3078   *            @arg TIM_ICPolarity_Falling: IC Falling edge.
// 3079   *            @arg TIM_ICPolarity_Rising: IC Rising edge.
// 3080   * @retval None
// 3081   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 3082 void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
// 3083                                 uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity)
// 3084 {
TIM_EncoderInterfaceConfig:
        PUSH     {R4-R7}
// 3085   uint16_t tmpsmcr = 0;
// 3086   uint16_t tmpccmr1 = 0;
// 3087   uint16_t tmpccer = 0;
// 3088     
// 3089   /* Check the parameters */
// 3090   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 3091   assert_param(IS_TIM_ENCODER_MODE(TIM_EncoderMode));
// 3092   assert_param(IS_TIM_IC_POLARITY(TIM_IC1Polarity));
// 3093   assert_param(IS_TIM_IC_POLARITY(TIM_IC2Polarity));
// 3094   
// 3095   /* Get the TIMx SMCR register value */
// 3096   tmpsmcr = TIMx->SMCR;
        LDRH     R6,[R0, #+8]
// 3097   /* Get the TIMx CCMR1 register value */
// 3098   tmpccmr1 = TIMx->CCMR1;
        LDRH     R5,[R0, #+24]
// 3099   /* Get the TIMx CCER register value */
// 3100   tmpccer = TIMx->CCER;
        LDRH     R4,[R0, #+32]
// 3101   /* Set the encoder Mode */
// 3102   tmpsmcr &= (uint16_t)(~((uint16_t)TIM_SMCR_SMS));
// 3103   tmpsmcr |= TIM_EncoderMode;
// 3104   /* Select the Capture Compare 1 and the Capture Compare 2 as input */
// 3105   tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC1S)) & (uint16_t)(~((uint16_t)TIM_CCMR1_CC2S)));
// 3106   tmpccmr1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
// 3107   /* Set the TI1 and the TI2 Polarities */
// 3108   tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P | TIM_CCER_CC1NP)) & (uint16_t)~((uint16_t)(TIM_CCER_CC2P | TIM_CCER_CC2NP));
// 3109   tmpccer |= (uint16_t)(TIM_IC1Polarity | (uint16_t)(TIM_IC2Polarity << (uint16_t)4));
// 3110   /* Write to TIMx SMCR */
// 3111   TIMx->SMCR = tmpsmcr;
        LDR      R7,??DataTable48_1  ;; 0xfff8
        ANDS     R7,R7,R6
        ORRS     R1,R1,R7
        STRH     R1,[R0, #+8]
// 3112   /* Write to TIMx CCMR1 */
// 3113   TIMx->CCMR1 = tmpccmr1;
        LDR      R1,??DataTable48_10  ;; 0xfcfc
        ANDS     R1,R1,R5
        MOVS     R5,#+255
        ADDS     R5,R5,#+2        ;; #+257
        ORRS     R5,R5,R1
        STRH     R5,[R0, #+24]
// 3114   /* Write to TIMx CCER */
// 3115   TIMx->CCER = tmpccer;
        LDR      R1,??DataTable48_11  ;; 0xff55
        ANDS     R1,R1,R4
        ORRS     R2,R2,R1
        LSLS     R1,R3,#+4
        ORRS     R1,R1,R2
        STRH     R1,[R0, #+32]
// 3116 }
        POP      {R4-R7}
        BX       LR               ;; return
// 3117 
// 3118 /**
// 3119   * @brief  Enables or disables the TIMx's Hall sensor interface.
// 3120   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 3121   * @note   TIM2 is not applicable for STM32F030 devices.   
// 3122   * @param  NewState: new state of the TIMx Hall sensor interface.
// 3123   *          This parameter can be: ENABLE or DISABLE.
// 3124   * @retval None
// 3125   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 3126 void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState)
// 3127 {
// 3128   /* Check the parameters */
// 3129   assert_param(IS_TIM_LIST3_PERIPH(TIMx));
// 3130   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 3131   
// 3132   if (NewState != DISABLE)
TIM_SelectHallSensor:
        CMP      R1,#+0
        LDRH     R1,[R0, #+4]
        BEQ      ??TIM_SelectHallSensor_0
// 3133   {
// 3134     /* Set the TI1S Bit */
// 3135     TIMx->CR2 |= TIM_CR2_TI1S;
        MOVS     R2,#+128
        ORRS     R2,R2,R1
        B        ??TIM_SelectHallSensor_1
// 3136   }
// 3137   else
// 3138   {
// 3139     /* Reset the TI1S Bit */
// 3140     TIMx->CR2 &= (uint16_t)~((uint16_t)TIM_CR2_TI1S);
??TIM_SelectHallSensor_0:
        LDR      R2,??DataTable48_9  ;; 0xff7f
        ANDS     R2,R2,R1
??TIM_SelectHallSensor_1:
        STRH     R2,[R0, #+4]
// 3141   }
// 3142 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48:
        DC32     0xfff7

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_1:
        DC32     0xfff8

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_2:
        DC32     0xff8f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_3:
        DC32     0xffef

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_4:
        DC32     0xcff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_5:
        DC32     0xff5f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_6:
        DC32     0xfffe

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_7:
        DC32     0xff0c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_8:
        DC32     0xfff5

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_9:
        DC32     0xff7f

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_10:
        DC32     0xfcfc

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable48_11:
        DC32     0xff55
// 3143 
// 3144 /**
// 3145   * @}
// 3146   */
// 3147 
// 3148 /** @defgroup TIM_Group9 Specific remapping management function
// 3149  *  @brief   Specific remapping management function
// 3150  *
// 3151 @verbatim
// 3152  ===============================================================================
// 3153                ##### Specific remapping management function #####
// 3154  ===============================================================================
// 3155 
// 3156 @endverbatim
// 3157   * @{
// 3158   */
// 3159 /**
// 3160   * @brief  Configures the TIM14 Remapping input Capabilities.
// 3161   * @param  TIMx: where x can be 14 to select the TIM peripheral.
// 3162   * @param  TIM_Remap: specifies the TIM input reampping source.
// 3163   *          This parameter can be one of the following values:
// 3164   *            @arg TIM14_GPIO: TIM14 Channel 1 is connected to GPIO.
// 3165   *            @arg TIM14_RTC_CLK: TIM14 Channel 1 is connected to RTC input clock.
// 3166   *                                RTC input clock can be LSE, LSI or HSE/div128.
// 3167   *            @arg TIM14_HSE_DIV32: TIM14 Channel 1 is connected to HSE/32 clock.  
// 3168   *            @arg TIM14_MCO: TIM14 Channel 1 is connected to MCO clock.  
// 3169   *                            MCO clock can be HSI14, SYSCLK, HSI, HSE or PLL/2.  
// 3170   * @retval None
// 3171   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 3172 void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap)
// 3173 {
// 3174  /* Check the parameters */
// 3175   assert_param(IS_TIM_LIST11_PERIPH(TIMx));
// 3176   assert_param(IS_TIM_REMAP(TIM_Remap));
// 3177 
// 3178   /* Set the Timer remapping configuration */
// 3179   TIMx->OR =  TIM_Remap;
TIM_RemapConfig:
        ADDS     R0,R0,#+80
        STRH     R1,[R0, #+0]
// 3180 }
        BX       LR               ;; return

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
// 3181 
// 3182 /**
// 3183   * @}
// 3184   */
// 3185 
// 3186 /**
// 3187   * @brief  Configure the TI1 as Input.
// 3188   * @param  TIMx: where x can be 1, 2, 3, 14, 15, 16 or 17 to select the TIM peripheral.
// 3189   * @note   TIM2 is not applicable for STM32F030 devices.   
// 3190   * @param  TIM_ICPolarity: The Input Polarity.
// 3191   *          This parameter can be one of the following values:
// 3192   *            @arg TIM_ICPolarity_Rising
// 3193   *            @arg TIM_ICPolarity_Falling
// 3194   * @param  TIM_ICSelection: specifies the input to be used.
// 3195   *          This parameter can be one of the following values:
// 3196   *            @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
// 3197   *            @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
// 3198   *            @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
// 3199   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
// 3200   *          This parameter must be a value between 0x00 and 0x0F.
// 3201   * @retval None
// 3202   */
// 3203 static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
// 3204                        uint16_t TIM_ICFilter)
// 3205 {
// 3206   uint16_t tmpccmr1 = 0, tmpccer = 0;
// 3207   /* Disable the Channel 1: Reset the CC1E Bit */
// 3208   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC1E);
// 3209   tmpccmr1 = TIMx->CCMR1;
// 3210   tmpccer = TIMx->CCER;
// 3211   /* Select the Input and set the filter */
// 3212   tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC1S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC1F)));
// 3213   tmpccmr1 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
// 3214  
// 3215   /* Select the Polarity and set the CC1E Bit */
// 3216   tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P | TIM_CCER_CC1NP));
// 3217   tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);
// 3218   /* Write to TIMx CCMR1 and CCER registers */
// 3219   TIMx->CCMR1 = tmpccmr1;
// 3220   TIMx->CCER = tmpccer;
// 3221 }
// 3222 
// 3223 /**
// 3224   * @brief  Configure the TI2 as Input.
// 3225   * @param  TIMx: where x can be 1, 2, 3, or 15 to select the TIM peripheral.
// 3226   * @note   TIM2 is not applicable for STM32F030 devices.  
// 3227   * @param  TIM_ICPolarity: The Input Polarity.
// 3228   *          This parameter can be one of the following values:
// 3229   *            @arg TIM_ICPolarity_Rising
// 3230   *            @arg TIM_ICPolarity_Falling
// 3231   * @param  TIM_ICSelection: specifies the input to be used.
// 3232   *          This parameter can be one of the following values:
// 3233   *            @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
// 3234   *            @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
// 3235   *            @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
// 3236   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
// 3237   *          This parameter must be a value between 0x00 and 0x0F.
// 3238   * @retval None
// 3239   */
// 3240 static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
// 3241                        uint16_t TIM_ICFilter)
// 3242 {
// 3243   uint16_t tmpccmr1 = 0, tmpccer = 0, tmp = 0;
// 3244   /* Disable the Channel 2: Reset the CC2E Bit */
// 3245   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC2E);
// 3246   tmpccmr1 = TIMx->CCMR1;
// 3247   tmpccer = TIMx->CCER;
// 3248   tmp = (uint16_t)(TIM_ICPolarity << 4);
// 3249   /* Select the Input and set the filter */
// 3250   tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC2S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC2F)));
// 3251   tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);
// 3252   tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8); 
// 3253   /* Select the Polarity and set the CC2E Bit */
// 3254   tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC2P | TIM_CCER_CC2NP));
// 3255   tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC2E);  
// 3256   /* Write to TIMx CCMR1 and CCER registers */
// 3257   TIMx->CCMR1 = tmpccmr1 ;
// 3258   TIMx->CCER = tmpccer;
// 3259 }
// 3260 
// 3261 /**
// 3262   * @brief  Configure the TI3 as Input.
// 3263   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 3264   * @note   TIM2 is not applicable for STM32F030 devices.   
// 3265   * @param  TIM_ICPolarity: The Input Polarity.
// 3266   *          This parameter can be one of the following values:
// 3267   *            @arg TIM_ICPolarity_Rising
// 3268   *            @arg TIM_ICPolarity_Falling
// 3269   * @param  TIM_ICSelection: specifies the input to be used.
// 3270   *          This parameter can be one of the following values:
// 3271   *            @arg TIM_ICSelection_DirectTI: TIM Input 3 is selected to be connected to IC3.
// 3272   *            @arg TIM_ICSelection_IndirectTI: TIM Input 3 is selected to be connected to IC4.
// 3273   *            @arg TIM_ICSelection_TRC: TIM Input 3 is selected to be connected to TRC.
// 3274   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
// 3275   *          This parameter must be a value between 0x00 and 0x0F.
// 3276   * @retval None
// 3277   */
// 3278 static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
// 3279                        uint16_t TIM_ICFilter)
// 3280 {
// 3281   uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;
// 3282   /* Disable the Channel 3: Reset the CC3E Bit */
// 3283   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC3E);
// 3284   tmpccmr2 = TIMx->CCMR2;
// 3285   tmpccer = TIMx->CCER;
// 3286   tmp = (uint16_t)(TIM_ICPolarity << 8);
// 3287   /* Select the Input and set the filter */
// 3288   tmpccmr2 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR2_CC3S)) & ((uint16_t)~((uint16_t)TIM_CCMR2_IC3F)));
// 3289   tmpccmr2 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
// 3290   /* Select the Polarity and set the CC3E Bit */
// 3291   tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC3P | TIM_CCER_CC3NP));
// 3292   tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC3E);  
// 3293   /* Write to TIMx CCMR2 and CCER registers */
// 3294   TIMx->CCMR2 = tmpccmr2;
// 3295   TIMx->CCER = tmpccer;
// 3296 }
// 3297 
// 3298 /**
// 3299   * @brief  Configure the TI4 as Input.
// 3300   * @param  TIMx: where x can be 1, 2 or 3 to select the TIM peripheral.
// 3301   * @note   TIM2 is not applicable for STM32F030 devices.  
// 3302   * @param  TIM_ICPolarity: The Input Polarity.
// 3303   *          This parameter can be one of the following values:
// 3304   *            @arg TIM_ICPolarity_Rising
// 3305   *            @arg TIM_ICPolarity_Falling
// 3306   * @param  TIM_ICSelection: specifies the input to be used.
// 3307   *          This parameter can be one of the following values:
// 3308   *            @arg TIM_ICSelection_DirectTI: TIM Input 4 is selected to be connected to IC4.
// 3309   *            @arg TIM_ICSelection_IndirectTI: TIM Input 4 is selected to be connected to IC3.
// 3310   *            @arg TIM_ICSelection_TRC: TIM Input 4 is selected to be connected to TRC.
// 3311   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
// 3312   *          This parameter must be a value between 0x00 and 0x0F.
// 3313   * @retval None
// 3314   */
// 3315 static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
// 3316                        uint16_t TIM_ICFilter)
// 3317 {
// 3318   uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;
// 3319 
// 3320    /* Disable the Channel 4: Reset the CC4E Bit */
// 3321   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC4E);
// 3322   tmpccmr2 = TIMx->CCMR2;
// 3323   tmpccer = TIMx->CCER;
// 3324   tmp = (uint16_t)(TIM_ICPolarity << 12);
// 3325   /* Select the Input and set the filter */
// 3326   tmpccmr2 &= (uint16_t)((uint16_t)(~(uint16_t)TIM_CCMR2_CC4S) & ((uint16_t)~((uint16_t)TIM_CCMR2_IC4F)));
// 3327   tmpccmr2 |= (uint16_t)(TIM_ICSelection << 8);
// 3328   tmpccmr2 |= (uint16_t)(TIM_ICFilter << 12);  
// 3329   /* Select the Polarity and set the CC4E Bit */
// 3330   tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC4P | TIM_CCER_CC4NP));
// 3331   tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC4E);
// 3332   /* Write to TIMx CCMR2 and CCER registers */
// 3333   TIMx->CCMR2 = tmpccmr2;
// 3334   TIMx->CCER = tmpccer;
// 3335 }
// 3336 
// 3337 /**
// 3338   * @}
// 3339   */
// 3340 
// 3341 /**
// 3342   * @}
// 3343   */
// 3344 
// 3345 /**
// 3346   * @}
// 3347   */
// 3348 
// 3349 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
// 2 626 bytes in section .text
// 
// 2 626 bytes of CODE memory
//
//Errors: none
//Warnings: none
