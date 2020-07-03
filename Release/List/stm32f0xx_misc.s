///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:50 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    misc.c                                                  /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    misc.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER   /
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
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_misc /
//                    .s                                                      /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_misc

        #define SHT_PROGBITS 0x1

        PUBLIC NVIC_Init
        PUBLIC NVIC_SystemLPConfig
        PUBLIC SysTick_CLKSourceConfig

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_misc.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_misc.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides all the miscellaneous firmware functions (add-on
//    8   *          to CMSIS functions).
//    9   ******************************************************************************
//   10   * @attention
//   11   *
//   12   * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
//   13   *
//   14   * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//   15   * You may not use this file except in compliance with the License.
//   16   * You may obtain a copy of the License at:
//   17   *
//   18   *        http://www.st.com/software_license_agreement_liberty_v2
//   19   *
//   20   * Unless required by applicable law or agreed to in writing, software 
//   21   * distributed under the License is distributed on an "AS IS" BASIS, 
//   22   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   23   * See the License for the specific language governing permissions and
//   24   * limitations under the License.
//   25   *
//   26   ******************************************************************************
//   27   */
//   28 
//   29 /* Includes ------------------------------------------------------------------*/
//   30 #include "stm32f0xx_misc.h"
//   31 
//   32 /** @addtogroup STM32F0xx_StdPeriph_Driver
//   33   * @{
//   34   */
//   35 
//   36 /** @defgroup MISC 
//   37   * @brief MISC driver modules
//   38   * @{
//   39   */
//   40 
//   41 /* Private typedef -----------------------------------------------------------*/
//   42 /* Private define ------------------------------------------------------------*/
//   43 /* Private macro -------------------------------------------------------------*/
//   44 /* Private variables ---------------------------------------------------------*/
//   45 /* Private function prototypes -----------------------------------------------*/
//   46 /* Private functions ---------------------------------------------------------*/
//   47 
//   48 /** @defgroup MISC_Private_Functions
//   49   * @{
//   50   */
//   51 /**
//   52   *
//   53 @verbatim
//   54  *******************************************************************************
//   55                    ##### Interrupts configuration functions #####
//   56  *******************************************************************************
//   57     [..] This section provide functions allowing to configure the NVIC interrupts
//   58         (IRQ). The Cortex-M0 exceptions are managed by CMSIS functions.
//   59          (#) Enable and Configure the priority of the selected IRQ Channels. 
//   60              The priority can be 0..3. 
//   61 
//   62         -@- Lower priority values gives higher priority.
//   63         -@- Priority Order:
//   64             (#@) Lowest priority.
//   65             (#@) Lowest hardware priority (IRQn position).  
//   66   
//   67 @endverbatim
//   68 */
//   69 
//   70 /**
//   71   * @brief  Initializes the NVIC peripheral according to the specified
//   72   *         parameters in the NVIC_InitStruct.
//   73   * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
//   74   *         the configuration information for the specified NVIC peripheral.
//   75   * @retval None
//   76   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   77 void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
//   78 {
NVIC_Init:
        PUSH     {R3-R7}
        MOVS     R1,R0
//   79   uint32_t tmppriority = 0x00;
//   80   
//   81   /* Check the parameters */
//   82   assert_param(IS_FUNCTIONAL_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));
//   83   assert_param(IS_NVIC_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelPriority));  
//   84     
//   85   if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
        LDRB     R0,[R1, #+0]
        MOVS     R2,#+31
        MOVS     R3,#+1
        LDRB     R4,[R1, #+2]
        CMP      R4,#+0
        BEQ      ??NVIC_Init_0
//   86   {
//   87     /* Compute the Corresponding IRQ Priority --------------------------------*/    
//   88     tmppriority = NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02];
        MOVS     R4,R0
        MOVS     R5,#+3
        BICS     R4,R4,R5
        LDR      R5,??DataTable2  ;; 0xe000e400
        ADDS     R4,R5,R4
        LDR      R5,[R4, #+0]
//   89     tmppriority &= (uint32_t)(~(((uint32_t)0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8)));
//   90     tmppriority |= (uint32_t)((((uint32_t)NVIC_InitStruct->NVIC_IRQChannelPriority << 6) & 0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8));    
//   91     
//   92     NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02] = tmppriority;
        LSLS     R0,R0,#+30
        LSRS     R6,R0,#+27
        MOVS     R7,#+255
        MOVS     R0,#+255
        LSLS     R0,R0,R6
        BICS     R5,R5,R0
        LDRB     R0,[R1, #+1]
        LSLS     R0,R0,#+6
        ANDS     R7,R7,R0
        LSLS     R7,R7,R6
        ORRS     R7,R7,R5
        STR      R7,[R4, #+0]
//   93     
//   94     /* Enable the Selected IRQ Channels --------------------------------------*/
//   95     NVIC->ISER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
        LDRB     R0,[R1, #+0]
        ANDS     R2,R2,R0
        LSLS     R3,R3,R2
        LDR      R0,??DataTable2_1  ;; 0xe000e100
        B        ??NVIC_Init_1
//   96   }
//   97   else
//   98   {
//   99     /* Disable the Selected IRQ Channels -------------------------------------*/
//  100     NVIC->ICER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
??NVIC_Init_0:
        ANDS     R2,R2,R0
        LSLS     R3,R3,R2
        LDR      R0,??DataTable2_2  ;; 0xe000e180
??NVIC_Init_1:
        STR      R3,[R0, #+0]
//  101   }
//  102 }
        POP      {R0,R4-R7}
        BX       LR               ;; return
//  103 
//  104 /**
//  105   * @brief  Selects the condition for the system to enter low power mode.
//  106   * @param  LowPowerMode: Specifies the new mode for the system to enter low power mode.
//  107   *          This parameter can be one of the following values:
//  108   *            @arg NVIC_LP_SEVONPEND: Low Power SEV on Pend.
//  109   *            @arg NVIC_LP_SLEEPDEEP: Low Power DEEPSLEEP request.
//  110   *            @arg NVIC_LP_SLEEPONEXIT: Low Power Sleep on Exit.
//  111   * @param  NewState: new state of LP condition. 
//  112   *          This parameter can be: ENABLE or DISABLE.
//  113   * @retval None
//  114   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  115 void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState)
//  116 {
//  117   /* Check the parameters */
//  118   assert_param(IS_NVIC_LP(LowPowerMode));
//  119   
//  120   assert_param(IS_FUNCTIONAL_STATE(NewState));  
//  121   
//  122   if (NewState != DISABLE)
NVIC_SystemLPConfig:
        LDR      R2,??DataTable2_3  ;; 0xe000ed10
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??NVIC_SystemLPConfig_0
//  123   {
//  124     SCB->SCR |= LowPowerMode;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
//  125   }
//  126   else
//  127   {
//  128     SCB->SCR &= (uint32_t)(~(uint32_t)LowPowerMode);
??NVIC_SystemLPConfig_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
//  129   }
//  130 }
        BX       LR               ;; return
//  131 
//  132 /**
//  133   * @brief  Configures the SysTick clock source.
//  134   * @param  SysTick_CLKSource: specifies the SysTick clock source.
//  135   *          This parameter can be one of the following values:
//  136   *            @arg SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8 selected as SysTick clock source.
//  137   *            @arg SysTick_CLKSource_HCLK: AHB clock selected as SysTick clock source.
//  138   * @retval None
//  139   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  140 void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
//  141 {
//  142   /* Check the parameters */
//  143   assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));
//  144   
//  145   if (SysTick_CLKSource == SysTick_CLKSource_HCLK)
SysTick_CLKSourceConfig:
        LDR      R1,??DataTable2_4  ;; 0xe000e010
        CMP      R0,#+4
        LDR      R0,[R1, #+0]
        BNE      ??SysTick_CLKSourceConfig_0
//  146   {
//  147     SysTick->CTRL |= SysTick_CLKSource_HCLK;
        MOVS     R2,#+4
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
        BX       LR
//  148   }
//  149   else
//  150   {
//  151     SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
??SysTick_CLKSourceConfig_0:
        MOVS     R2,#+4
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
//  152   }
//  153 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2:
        DC32     0xe000e400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_1:
        DC32     0xe000e100

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_2:
        DC32     0xe000e180

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_3:
        DC32     0xe000ed10

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_4:
        DC32     0xe000e010

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  154 
//  155 /**
//  156   * @}
//  157   */
//  158 
//  159 /**
//  160   * @}
//  161   */
//  162 
//  163 /**
//  164   * @}
//  165   */
//  166 
//  167 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
// 138 bytes in section .text
// 
// 138 bytes of CODE memory
//
//Errors: none
//Warnings: none
