///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:49 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    gpio.c                                                  /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    gpio.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER   /
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
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_gpio /
//                    .s                                                      /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_gpio

        #define SHT_PROGBITS 0x1

        EXTERN RCC_AHBPeriphResetCmd

        PUBLIC GPIO_DeInit
        PUBLIC GPIO_Init
        PUBLIC GPIO_PinAFConfig
        PUBLIC GPIO_PinLockConfig
        PUBLIC GPIO_ReadInputData
        PUBLIC GPIO_ReadInputDataBit
        PUBLIC GPIO_ReadOutputData
        PUBLIC GPIO_ReadOutputDataBit
        PUBLIC GPIO_ResetBits
        PUBLIC GPIO_SetBits
        PUBLIC GPIO_StructInit
        PUBLIC GPIO_Write
        PUBLIC GPIO_WriteBit

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_gpio.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_gpio.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides firmware functions to manage the following 
//    8   *          functionalities of the GPIO peripheral:
//    9   *           + Initialization and Configuration functions
//   10   *           + GPIO Read and Write functions
//   11   *           + GPIO Alternate functions configuration functions
//   12   *
//   13   *  @verbatim
//   14   *
//   15   *
//   16     ===========================================================================
//   17                          ##### How to use this driver #####
//   18     ===========================================================================
//   19       [..]
//   20       (#) Enable the GPIO AHB clock using RCC_AHBPeriphClockCmd()
//   21       (#) Configure the GPIO pin(s) using GPIO_Init()
//   22           Four possible configuration are available for each pin:
//   23          (++) Input: Floating, Pull-up, Pull-down.
//   24          (++) Output: Push-Pull (Pull-up, Pull-down or no Pull)
//   25                       Open Drain (Pull-up, Pull-down or no Pull).
//   26               In output mode, the speed is configurable: Low, Medium, Fast or High.
//   27          (++) Alternate Function: Push-Pull (Pull-up, Pull-down or no Pull)
//   28                                   Open Drain (Pull-up, Pull-down or no Pull).
//   29          (++) Analog: required mode when a pin is to be used as ADC channel,
//   30               DAC output or comparator input.
//   31       (#) Peripherals alternate function:
//   32          (++) For ADC, DAC and comparators, configure the desired pin in analog 
//   33               mode using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
//   34          (++) For other peripherals (TIM, USART...):
//   35               (+++) Connect the pin to the desired peripherals' Alternate 
//   36                     Function (AF) using GPIO_PinAFConfig() function. For PortC, 
//   37                     PortD and PortF, no configuration is needed.
//   38               (+++) Configure the desired pin in alternate function mode using
//   39                     GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
//   40               (+++) Select the type, pull-up/pull-down and output speed via 
//   41                     GPIO_PuPd, GPIO_OType and GPIO_Speed members
//   42               (+++) Call GPIO_Init() function
//   43       (#) To get the level of a pin configured in input mode use GPIO_ReadInputDataBit()
//   44       (#) To set/reset the level of a pin configured in output mode use
//   45           GPIO_SetBits()/GPIO_ResetBits()
//   46       (#) During and just after reset, the alternate functions are not active and 
//   47           the GPIO pins are configured in input floating mode (except JTAG pins).
//   48       (#) The LSE oscillator pins OSC32_IN and OSC32_OUT can be used as 
//   49           general-purpose (PC14 and PC15, respectively) when the LSE oscillator 
//   50           is off. The LSE has priority over the GPIO function.
//   51       (#) The HSE oscillator pins OSC_IN/OSC_OUT can be used as general-purpose 
//   52           PD0 and PD1, respectively, when the HSE oscillator is off. The HSE has 
//   53           priority over the GPIO function.
//   54     @endverbatim
//   55   ******************************************************************************
//   56   * @attention
//   57   *
//   58   * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
//   59   *
//   60   * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//   61   * You may not use this file except in compliance with the License.
//   62   * You may obtain a copy of the License at:
//   63   *
//   64   *        http://www.st.com/software_license_agreement_liberty_v2
//   65   *
//   66   * Unless required by applicable law or agreed to in writing, software 
//   67   * distributed under the License is distributed on an "AS IS" BASIS, 
//   68   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   69   * See the License for the specific language governing permissions and
//   70   * limitations under the License.
//   71   *
//   72   ******************************************************************************
//   73   */
//   74 
//   75 /* Includes ------------------------------------------------------------------*/
//   76 #include "stm32f0xx_gpio.h"
//   77 #include "stm32f0xx_rcc.h"
//   78 
//   79 /** @addtogroup STM32F0xx_StdPeriph_Driver
//   80   * @{
//   81   */
//   82 
//   83 /** @defgroup GPIO 
//   84   * @brief GPIO driver modules
//   85   * @{
//   86   */
//   87 
//   88 /* Private typedef -----------------------------------------------------------*/
//   89 /* Private define ------------------------------------------------------------*/
//   90 /* Private macro -------------------------------------------------------------*/
//   91 /* Private variables ---------------------------------------------------------*/
//   92 /* Private function prototypes -----------------------------------------------*/
//   93 /* Private functions ---------------------------------------------------------*/
//   94 
//   95 /** @defgroup GPIO_Private_Functions 
//   96   * @{
//   97   */
//   98 
//   99 /** @defgroup GPIO_Group1 Initialization and Configuration
//  100  *  @brief   Initialization and Configuration
//  101  *
//  102 @verbatim
//  103  ===============================================================================
//  104                     ##### Initialization and Configuration #####
//  105  ===============================================================================
//  106 
//  107 @endverbatim
//  108   * @{
//  109   */
//  110 
//  111 /**
//  112   * @brief  Deinitializes the GPIOx peripheral registers to their default reset 
//  113   *         values.
//  114   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  115   * @note   GPIOE is available only for STM32F072.
//  116   * @note   GPIOD is not available for STM32F031.    
//  117   * @retval None
//  118   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  119 void GPIO_DeInit(GPIO_TypeDef* GPIOx)
//  120 {
GPIO_DeInit:
        PUSH     {R4,LR}
//  121   /* Check the parameters */
//  122   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  123 
//  124   if(GPIOx == GPIOA)
        MOVS     R1,#+144
        LSLS     R1,R1,#+23       ;; #+1207959552
        CMP      R0,R1
        BNE      ??GPIO_DeInit_0
//  125   {
//  126     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+10       ;; #+131072
        B        ??GPIO_DeInit_1
//  127     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//  128   }
//  129   else if(GPIOx == GPIOB)
??GPIO_DeInit_0:
        LDR      R1,??DataTable1  ;; 0x48000400
        CMP      R0,R1
        BNE      ??GPIO_DeInit_2
//  130   {
//  131     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+11       ;; #+262144
        B        ??GPIO_DeInit_1
//  132     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, DISABLE);
//  133   }
//  134   else if(GPIOx == GPIOC)
??GPIO_DeInit_2:
        LDR      R1,??DataTable1_1  ;; 0x48000800
        CMP      R0,R1
        BNE      ??GPIO_DeInit_3
//  135   {
//  136     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+12       ;; #+524288
        B        ??GPIO_DeInit_1
//  137     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, DISABLE);
//  138   }
//  139   else if(GPIOx == GPIOD)
??GPIO_DeInit_3:
        LDR      R1,??DataTable1_2  ;; 0x48000c00
        CMP      R0,R1
        BNE      ??GPIO_DeInit_4
//  140   {
//  141     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+13       ;; #+1048576
        B        ??GPIO_DeInit_1
//  142     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, DISABLE);
//  143   }
//  144   else if(GPIOx == GPIOE)
??GPIO_DeInit_4:
        LDR      R1,??DataTable1_3  ;; 0x48001000
        CMP      R0,R1
        BNE      ??GPIO_DeInit_5
//  145   {
//  146     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+14       ;; #+2097152
        B        ??GPIO_DeInit_1
//  147     RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, DISABLE);
//  148   }
//  149   else
//  150   {
//  151     if(GPIOx == GPIOF)
??GPIO_DeInit_5:
        LDR      R1,??DataTable1_4  ;; 0x48001400
        CMP      R0,R1
        BNE      ??GPIO_DeInit_6
//  152     {
//  153       RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOF, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+15       ;; #+4194304
??GPIO_DeInit_1:
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       RCC_AHBPeriphResetCmd
//  154       RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOF, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,R4
        BL       RCC_AHBPeriphResetCmd
//  155     }
//  156   }
//  157 }
??GPIO_DeInit_6:
        POP      {R4,PC}          ;; return
//  158 
//  159 /**
//  160   * @brief  Initializes the GPIOx peripheral according to the specified 
//  161   *         parameters in the GPIO_InitStruct.
//  162   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  163   * @note   GPIOE is available only for STM32F072.
//  164   * @note   GPIOD is not available for STM32F031.   
//  165   * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that contains
//  166   *         the configuration information for the specified GPIO peripheral.
//  167   * @retval None
//  168   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  169 void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
//  170 {
GPIO_Init:
        PUSH     {R3-R6}
//  171   uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;
        MOVS     R3,#+0
        MOVS     R2,#+0
//  172 
//  173   /* Check the parameters */
//  174   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  175   assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
//  176   assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
//  177   assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));
//  178 
//  179   /*-------------------------- Configure the port pins -----------------------*/
//  180   /*-- GPIO Mode Configuration --*/
//  181   for (pinpos = 0x00; pinpos < 0x10; pinpos++)
//  182   {
//  183     pos = ((uint32_t)0x01) << pinpos;
??GPIO_Init_0:
        MOVS     R4,#+1
        LSLS     R4,R4,R2
//  184 
//  185     /* Get the port pins position */
//  186     currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;
//  187 
//  188     if (currentpin == pos)
        LDR      R5,[R1, #+0]
        ANDS     R5,R5,R4
        CMP      R5,R4
        BNE      ??GPIO_Init_1
//  189     {
//  190       if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
        LDRB     R5,[R1, #+4]
        CMP      R5,#+1
        BEQ      ??GPIO_Init_2
        CMP      R5,#+2
        BNE      ??GPIO_Init_3
//  191       {
//  192         /* Check Speed mode parameters */
//  193         assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));
//  194 
//  195         /* Speed mode configuration */
//  196         GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
??GPIO_Init_2:
        LDR      R5,[R0, #+8]
        MOVS     R6,#+3
        LSLS     R6,R6,R3
        BICS     R5,R5,R6
        STR      R5,[R0, #+8]
//  197         GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));
        LDR      R5,[R0, #+8]
        LDRB     R6,[R1, #+5]
        LSLS     R6,R6,R3
        ORRS     R6,R6,R5
        STR      R6,[R0, #+8]
//  198 
//  199         /* Check Output mode parameters */
//  200         assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));
//  201 
//  202         /* Output mode configuration */
//  203         GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));
        LDRH     R5,[R0, #+4]
        BICS     R5,R5,R4
        STRH     R5,[R0, #+4]
//  204         GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));
        LDRH     R4,[R0, #+4]
        LDRB     R5,[R1, #+6]
        LSLS     R5,R5,R2
        ORRS     R5,R5,R4
        STRH     R5,[R0, #+4]
//  205       }
//  206 
//  207       GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
??GPIO_Init_3:
        MOVS     R5,#+3
        LSLS     R5,R5,R3
        MVNS     R4,R5
        LDR      R5,[R0, #+0]
        ANDS     R5,R5,R4
        STR      R5,[R0, #+0]
//  208 
//  209       GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));
        LDR      R5,[R0, #+0]
        LDRB     R6,[R1, #+4]
        LSLS     R6,R6,R3
        ORRS     R6,R6,R5
        STR      R6,[R0, #+0]
//  210 
//  211       /* Pull-up Pull down resistor configuration */
//  212       GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
        LDR      R5,[R0, #+12]
        ANDS     R4,R4,R5
        STR      R4,[R0, #+12]
//  213       GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));
        LDR      R4,[R0, #+12]
        LDRB     R5,[R1, #+7]
        LSLS     R5,R5,R3
        ORRS     R5,R5,R4
        STR      R5,[R0, #+12]
//  214     }
//  215   }
??GPIO_Init_1:
        ADDS     R2,R2,#+1
        ADDS     R3,R3,#+2
        CMP      R2,#+16
        BCC      ??GPIO_Init_0
//  216 }
        POP      {R0,R4-R6}
        BX       LR               ;; return
//  217 
//  218 /**
//  219   * @brief  Fills each GPIO_InitStruct member with its default value.
//  220   * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure which will 
//  221   *         be initialized.
//  222   * @retval None
//  223   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  224 void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct)
//  225 {
//  226   /* Reset GPIO init structure parameters values */
//  227   GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
GPIO_StructInit:
        LDR      R1,??DataTable1_5  ;; 0xffff
        STR      R1,[R0, #+0]
//  228   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN;
        MOVS     R1,#+0
        STRB     R1,[R0, #+4]
//  229   GPIO_InitStruct->GPIO_Speed = GPIO_Speed_Level_2;
        MOVS     R2,#+1
        STRB     R2,[R0, #+5]
//  230   GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
        STRB     R1,[R0, #+6]
//  231   GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
        STRB     R1,[R0, #+7]
//  232 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1:
        DC32     0x48000400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_1:
        DC32     0x48000800

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_2:
        DC32     0x48000c00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_3:
        DC32     0x48001000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_4:
        DC32     0x48001400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_5:
        DC32     0xffff
//  233 
//  234 /**
//  235   * @brief  Locks GPIO Pins configuration registers.
//  236   * @note   The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
//  237   *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
//  238   * @note   The configuration of the locked GPIO pins can no longer be modified
//  239   *         until the next device reset.
//  240   * @param  GPIOx: where x can be (A or B) to select the GPIO peripheral.
//  241   * @param  GPIO_Pin: specifies the port bit to be written.
//  242   *          This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
//  243   * @retval None
//  244   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  245 void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//  246 {
GPIO_PinLockConfig:
        SUB      SP,SP,#+4
//  247   __IO uint32_t tmp = 0x00010000;
        MOVS     R2,#+128
        LSLS     R2,R2,#+9        ;; #+65536
        STR      R2,[SP, #+0]
//  248 
//  249   /* Check the parameters */
//  250   assert_param(IS_GPIO_LIST_PERIPH(GPIOx));
//  251   assert_param(IS_GPIO_PIN(GPIO_Pin));
//  252 
//  253   tmp |= GPIO_Pin;
        LDR      R2,[SP, #+0]
        ORRS     R2,R2,R1
        STR      R2,[SP, #+0]
//  254   /* Set LCKK bit */
//  255   GPIOx->LCKR = tmp;
        LDR      R2,[SP, #+0]
        STR      R2,[R0, #+28]
//  256   /* Reset LCKK bit */
//  257   GPIOx->LCKR =  GPIO_Pin;
        STR      R1,[R0, #+28]
//  258   /* Set LCKK bit */
//  259   GPIOx->LCKR = tmp;
        LDR      R1,[SP, #+0]
        STR      R1,[R0, #+28]
//  260   /* Read LCKK bit */
//  261   tmp = GPIOx->LCKR;
        LDR      R1,[R0, #+28]
        STR      R1,[SP, #+0]
//  262   /* Read LCKK bit */
//  263   tmp = GPIOx->LCKR;
        LDR      R0,[R0, #+28]
        STR      R0,[SP, #+0]
//  264 }
        ADD      SP,SP,#+4
        BX       LR               ;; return
//  265 
//  266 /**
//  267   * @}
//  268   */
//  269 
//  270 /** @defgroup GPIO_Group2 GPIO Read and Write
//  271  *  @brief   GPIO Read and Write
//  272  *
//  273 @verbatim   
//  274  ===============================================================================
//  275                       ##### GPIO Read and Write #####
//  276  ===============================================================================  
//  277 
//  278 @endverbatim
//  279   * @{
//  280   */
//  281 
//  282 /**
//  283   * @brief  Reads the specified input port pin.
//  284   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  285   * @note   GPIOE is available only for STM32F072.
//  286   * @note   GPIOD is not available for STM32F031.   
//  287   * @param  GPIO_Pin: specifies the port bit to read.
//  288   * @note   This parameter can be GPIO_Pin_x where x can be:
//  289   *         For STM32F051 and STM32F030: (0..15) for GPIOA, GPIOB, GPIOC, (2) for GPIOD and (0..1, 4..7) for GIIOF.
//  290   *         For STM32F072: (0..15) for GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, (0..10) for GPIOF.
//  291   *         For STM32F031: (0..15) for GPIOA, GPIOB, (13..15) for GPIOC and (0..1, 6..7) for GPIOF.  
//  292   * @retval The input port pin value.
//  293   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  294 uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//  295 {
GPIO_ReadInputDataBit:
        MOVS     R2,R0
//  296   uint8_t bitstatus = 0x00;
        MOVS     R0,#+0
//  297 
//  298   /* Check the parameters */
//  299   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  300   assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
//  301 
//  302   if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
        LDRH     R2,[R2, #+16]
        TST      R2,R1
        BEQ      ??GPIO_ReadInputDataBit_0
//  303   {
//  304     bitstatus = (uint8_t)Bit_SET;
        MOVS     R0,#+1
//  305   }
//  306   else
//  307   {
//  308     bitstatus = (uint8_t)Bit_RESET;
//  309   }
//  310   return bitstatus;
??GPIO_ReadInputDataBit_0:
        BX       LR               ;; return
//  311 }
//  312 
//  313 /**
//  314   * @brief  Reads the specified input port pin.
//  315   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  316   * @note   GPIOE is available only for STM32F072.
//  317   * @note   GPIOD is not available for STM32F031.   
//  318   * @retval The input port pin value.
//  319   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  320 uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
//  321 {
//  322   /* Check the parameters */
//  323   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  324 
//  325   return ((uint16_t)GPIOx->IDR);
GPIO_ReadInputData:
        LDRH     R0,[R0, #+16]
        BX       LR               ;; return
//  326 }
//  327 
//  328 /**
//  329   * @brief  Reads the specified output data port bit.
//  330   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  331   * @note   GPIOE is available only for STM32F072.
//  332   * @note   GPIOD is not available for STM32F031.   
//  333   * @param  GPIO_Pin: Specifies the port bit to read.
//  334   * @note   This parameter can be GPIO_Pin_x where x can be:
//  335   *         For STM32F051 and STM32F030: (0..15) for GPIOA, GPIOB, GPIOC, (2) for GPIOD and (0..1, 4..7) for GIIOF.
//  336   *         For STM32F072: (0..15) for GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, (0..10) for GPIOF.
//  337   *         For STM32F031: (0..15) for GPIOA, GPIOB, (13..15) for GPIOC and (0..1, 6..7) for GPIOF. 
//  338   * @retval The output port pin value.
//  339   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  340 uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//  341 {
GPIO_ReadOutputDataBit:
        MOVS     R2,R0
//  342   uint8_t bitstatus = 0x00;
        MOVS     R0,#+0
//  343 
//  344   /* Check the parameters */
//  345   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  346   assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
//  347 
//  348   if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
        LDRH     R2,[R2, #+20]
        TST      R2,R1
        BEQ      ??GPIO_ReadOutputDataBit_0
//  349   {
//  350     bitstatus = (uint8_t)Bit_SET;
        MOVS     R0,#+1
//  351   }
//  352   else
//  353   {
//  354     bitstatus = (uint8_t)Bit_RESET;
//  355   }
//  356   return bitstatus;
??GPIO_ReadOutputDataBit_0:
        BX       LR               ;; return
//  357 }
//  358 
//  359 /**
//  360   * @brief  Reads the specified GPIO output data port.
//  361   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  362   * @note   GPIOE is available only for STM32F072.
//  363   * @note   GPIOD is not available for STM32F031.    
//  364   * @retval GPIO output data port value.
//  365   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  366 uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
//  367 {
//  368   /* Check the parameters */
//  369   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  370 
//  371   return ((uint16_t)GPIOx->ODR);
GPIO_ReadOutputData:
        LDRH     R0,[R0, #+20]
        BX       LR               ;; return
//  372 }
//  373 
//  374 /**
//  375   * @brief  Sets the selected data port bits.
//  376   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  377   * @note   GPIOE is available only for STM32F072.
//  378   * @note   GPIOD is not available for STM32F031.    
//  379   * @param  GPIO_Pin: specifies the port bits to be written.
//  380   * @note   This parameter can be GPIO_Pin_x where x can be:
//  381   *         For STM32F051 and STM32F030: (0..15) for GPIOA, GPIOB, GPIOC, (2) for GPIOD and (0..1, 4..7) for GIIOF.
//  382   *         For STM32F072: (0..15) for GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, (0..10) for GPIOF.
//  383   *         For STM32F031: (0..15) for GPIOA, GPIOB, (13..15) for GPIOC and (0..1, 6..7) for GPIOF. 
//  384   * @retval None
//  385   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  386 void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//  387 {
//  388   /* Check the parameters */
//  389   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  390   assert_param(IS_GPIO_PIN(GPIO_Pin));
//  391 
//  392   GPIOx->BSRR = GPIO_Pin;
GPIO_SetBits:
        STR      R1,[R0, #+24]
//  393 }
        BX       LR               ;; return
//  394 
//  395 /**
//  396   * @brief  Clears the selected data port bits.
//  397   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  398   * @note   GPIOE is available only for STM32F072.
//  399   * @note   GPIOD is not available for STM32F031.
//  400   * @param  GPIO_Pin: specifies the port bits to be written.
//  401   * @note   This parameter can be GPIO_Pin_x where x can be:
//  402   *         For STM32F051 and STM32F030: (0..15) for GPIOA, GPIOB, GPIOC, (2) for GPIOD and (0..1, 4..7) for GIIOF.
//  403   *         For STM32F072: (0..15) for GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, (0..10) for GPIOF.
//  404   *         For STM32F031: (0..15) for GPIOA, GPIOB, (13..15) for GPIOC and (0..1, 6..7) for GPIOF. 
//  405   * @retval None
//  406   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  407 void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//  408 {
//  409   /* Check the parameters */
//  410   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  411   assert_param(IS_GPIO_PIN(GPIO_Pin));
//  412 
//  413   GPIOx->BRR = GPIO_Pin;
GPIO_ResetBits:
        STRH     R1,[R0, #+40]
//  414 }
        BX       LR               ;; return
//  415 
//  416 /**
//  417   * @brief  Sets or clears the selected data port bit.
//  418   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  419   * @note   GPIOE is available only for STM32F072.
//  420   * @note   GPIOD is not available for STM32F031.  
//  421   * @param  GPIO_Pin: specifies the port bit to be written.
//  422   * @param  BitVal: specifies the value to be written to the selected bit.
//  423   *          This parameter can be one of the BitAction enumeration values:
//  424   *            @arg Bit_RESET: to clear the port pin
//  425   *            @arg Bit_SET: to set the port pin
//  426   * @note   This parameter can be GPIO_Pin_x where x can be:
//  427   *         For STM32F051 and STM32F030: (0..15) for GPIOA, GPIOB, GPIOC, (2) for GPIOD and (0..1, 4..7) for GIIOF.
//  428   *         For STM32F072: (0..15) for GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, (0..10) for GPIOF.
//  429   *         For STM32F031: (0..15) for GPIOA, GPIOB, (13..15) for GPIOC and (0..1, 6..7) for GPIOF.
//  430   * @retval None
//  431   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  432 void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
//  433 {
//  434   /* Check the parameters */
//  435   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  436   assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
//  437   assert_param(IS_GPIO_BIT_ACTION(BitVal));
//  438 
//  439   if (BitVal != Bit_RESET)
GPIO_WriteBit:
        CMP      R2,#+0
        BEQ      ??GPIO_WriteBit_0
//  440   {
//  441     GPIOx->BSRR = GPIO_Pin;
        STR      R1,[R0, #+24]
        BX       LR
//  442   }
//  443   else
//  444   {
//  445     GPIOx->BRR = GPIO_Pin ;
??GPIO_WriteBit_0:
        STRH     R1,[R0, #+40]
//  446   }
//  447 }
        BX       LR               ;; return
//  448 
//  449 /**
//  450   * @brief  Writes data to the specified GPIO data port.
//  451   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  452   * @note   GPIOE is available only for STM32F072.
//  453   * @note   GPIOD is not available for STM32F031.  
//  454   * @param  PortVal: specifies the value to be written to the port output data register.
//  455   * @retval None
//  456   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  457 void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
//  458 {
//  459   /* Check the parameters */
//  460   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  461 
//  462   GPIOx->ODR = PortVal;
GPIO_Write:
        STRH     R1,[R0, #+20]
//  463 }
        BX       LR               ;; return
//  464 
//  465 /**
//  466   * @}
//  467   */
//  468 
//  469 /** @defgroup GPIO_Group3 GPIO Alternate functions configuration functions
//  470  *  @brief   GPIO Alternate functions configuration functions
//  471  *
//  472 @verbatim   
//  473  ===============================================================================
//  474           ##### GPIO Alternate functions configuration functions #####
//  475  ===============================================================================  
//  476 
//  477 @endverbatim
//  478   * @{
//  479   */
//  480 
//  481 /**
//  482   * @brief  Writes data to the specified GPIO data port.
//  483   * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
//  484   * @note   GPIOC, GPIOD, GPIOE and GPIOF  are available only for STM32F072 and STM32F091. 
//  485   * @param  GPIO_PinSource: specifies the pin for the Alternate function.
//  486   *          This parameter can be GPIO_PinSourcex where x can be (0..15) for GPIOA, GPIOB, GPIOD, GPIOE
//  487   *          and (0..12) for GPIOC and (0, 2..5, 9..10) for GPIOF.    
//  488   * @param  GPIO_AF: selects the pin to used as Alternate function.
//  489   *          This parameter can be one of the following value:
//  490   *            @arg GPIO_AF_0:  WKUP, EVENTOUT, TIM15, SPI1, TIM17, MCO, SWDAT, SWCLK,
//  491   *                             TIM14, BOOT, USART1, CEC, IR_OUT, SPI2, TIM3, USART4,
//  492   *                             CAN, USART2, CRS, TIM16, TIM1, TS, USART8 
//  493   *            @arg GPIO_AF_1: USART2, CEC, TIM3, USART1, USART2, EVENTOUT, I2C1,
//  494   *                            I2C2, TIM15, SPI2, USART3, TS, SPI1, USART7, USART8
//  495   *                            USART5, USART4, USART6, I2C1   
//  496   *            @arg GPIO_AF_2: TIM2, TIM1, EVENTOUT, TIM16, TIM17, USB, USART6, USART5,
//  497   *                            USART8, USART7, USART6  
//  498   *            @arg GPIO_AF_3: TS, I2C1, TIM15, EVENTOUT 
//  499   *            @arg GPIO_AF_4: TIM14, USART4, USART3, CRS, CAN, I2C1, USART5
//  500   *            @arg GPIO_AF_5: TIM16, TIM17, TIM15, SPI2, I2C2, USART6, MCO
//  501   *            @arg GPIO_AF_6: EVENTOUT
//  502   *            @arg GPIO_AF_7: COMP1 OUT, COMP2 OUT 
//  503   * @note   The pin should already been configured in Alternate Function mode(AF)
//  504   *         using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
//  505   * @note   Refer to the Alternate function mapping table in the device datasheet 
//  506   *         for the detailed mapping of the system and peripherals'alternate 
//  507   *         function I/O pins.
//  508   * @retval None
//  509   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  510 void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
//  511 {
GPIO_PinAFConfig:
        PUSH     {R4}
//  512   uint32_t temp = 0x00;
//  513   uint32_t temp_2 = 0x00;
//  514 
//  515   /* Check the parameters */
//  516   assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
//  517   assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
//  518   assert_param(IS_GPIO_AF(GPIO_AF));
//  519 
//  520   temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
//  521   GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
        LSRS     R3,R1,#+3
        LSLS     R3,R3,#+2
        ADDS     R0,R0,R3
        LSLS     R1,R1,#+29
        LSRS     R1,R1,#+27
        LDR      R3,[R0, #+32]
        MOVS     R4,#+15
        LSLS     R4,R4,R1
        BICS     R3,R3,R4
        STR      R3,[R0, #+32]
//  522   temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
//  523   GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
        LDR      R3,[R0, #+32]
        LSLS     R2,R2,R1
        ORRS     R2,R2,R3
        STR      R2,[R0, #+32]
//  524 }
        POP      {R4}
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
//  525 
//  526 /**
//  527   * @}
//  528   */
//  529 
//  530 /**
//  531   * @}
//  532   */
//  533 
//  534 /**
//  535   * @}
//  536   */
//  537 
//  538 /**
//  539   * @}
//  540   */
//  541 
//  542 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
// 378 bytes in section .text
// 
// 378 bytes of CODE memory
//
//Errors: none
//Warnings: none
