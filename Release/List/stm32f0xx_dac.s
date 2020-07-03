///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:51 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    dac.c                                                   /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    dac.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER    /
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
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_dac. /
//                    s                                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_dac

        #define SHT_PROGBITS 0x1

        EXTERN RCC_APB1PeriphResetCmd

        PUBLIC DAC_ClearFlag
        PUBLIC DAC_ClearITPendingBit
        PUBLIC DAC_Cmd
        PUBLIC DAC_DMACmd
        PUBLIC DAC_DeInit
        PUBLIC DAC_DualSoftwareTriggerCmd
        PUBLIC DAC_GetDataOutputValue
        PUBLIC DAC_GetFlagStatus
        PUBLIC DAC_GetITStatus
        PUBLIC DAC_ITConfig
        PUBLIC DAC_Init
        PUBLIC DAC_SetChannel1Data
        PUBLIC DAC_SetChannel2Data
        PUBLIC DAC_SetDualChannelData
        PUBLIC DAC_SoftwareTriggerCmd
        PUBLIC DAC_StructInit
        PUBLIC DAC_WaveGenerationCmd

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_dac.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_dac.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides firmware functions to manage the following 
//    8   *          functionalities of the Digital-to-Analog Converter (DAC) peripheral
//    9   *          applicable only on STM32F051 and STM32F072 devices:
//   10   *           + DAC channel configuration: trigger, output buffer, data format
//   11   *           + DMA management
//   12   *           + Interrupts and flags management
//   13   *
//   14   *  @verbatim
//   15   *
//   16  ===============================================================================
//   17                         ##### DAC Peripheral features #####
//   18  ===============================================================================
//   19     [..] The device integrates two 12-bit Digital Analog Converters refered as
//   20          DAC channel1 with DAC_OUT1 (PA4) and DAC_OUT2 (PA5) as outputs.
//   21   
//   22     [..] Digital to Analog conversion can be non-triggered using DAC_Trigger_None
//   23          and DAC_OUTx is available once writing to DHRx register using 
//   24          DAC_SetChannel1Data() or DAC_SetChannel2Data() 
//   25   
//   26     [..] Digital to Analog conversion can be triggered by:
//   27          (#) External event: EXTI Line 9 (any GPIOx_Pin9) using DAC_Trigger_Ext_IT9.
//   28              The used pin (GPIOx_Pin9) must be configured in input mode.
//   29   
//   30          (#) Timers TRGO: TIM2, TIM3,TIM7, TIM6 and TIM15 
//   31              (DAC_Trigger_T2_TRGO, DAC_Trigger_T3_TRGO...)
//   32              The timer TRGO event should be selected using TIM_SelectOutputTrigger()
//   33   
//   34          (#) Software using DAC_Trigger_Software
//   35   
//   36     [..] Each DAC integrates an output buffer that can be used to 
//   37          reduce the output impedance, and to drive external loads directly
//   38          without having to add an external operational amplifier.
//   39          To enable the output buffer use  
//   40          DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
//   41   
//   42     [..] Refer to the device datasheet for more details about output impedance
//   43          value with and without output buffer.
//   44          
//   45     [..] DAC wave generation feature
//   46          Both DAC channels can be used to generate
//   47              1- Noise wave using DAC_WaveGeneration_Noise
//   48              2- Triangle wave using DAC_WaveGeneration_Triangle
//   49   
//   50     [..] The DAC data format can be:
//   51          (#) 8-bit right alignment using DAC_Align_8b_R
//   52          (#) 12-bit left alignment using DAC_Align_12b_L
//   53          (#) 12-bit right alignment using DAC_Align_12b_R
//   54   
//   55     [..] The analog output voltage on each DAC channel pin is determined
//   56          by the following equation: DAC_OUTx = VREF+ * DOR / 4095
//   57          with  DOR is the Data Output Register
//   58          VEF+ is the input voltage reference (refer to the device datasheet)
//   59          e.g. To set DAC_OUT1 to 0.7V, use
//   60          DAC_SetChannel1Data(DAC_Align_12b_R, 868);
//   61          Assuming that VREF+ = 3.3, DAC_OUT1 = (3.3 * 868) / 4095 = 0.7V
//   62   
//   63     [..] A DMA1 request can be generated when an external trigger (but not
//   64          a software trigger) occurs if DMA1 requests are enabled using
//   65          DAC_DMACmd()
//   66          DMA1 requests are mapped as following:
//   67          (+) DAC channel1 is mapped on DMA1 channel3 which must be already 
//   68              configured
//   69          (+) DAC channel2 is mapped on DMA1 channel4 which must be already 
//   70              configured
//   71     
//   72                       ##### How to use this driver #####
//   73  ===============================================================================
//   74     [..]
//   75          (+) Enable DAC APB1 clock to get write access to DAC registers
//   76              using RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE)
//   77               
//   78          (+) Configure DAC_OUTx (DAC_OUT1: PA4, DAC_OUT2: PA5) in analog mode
//   79              using GPIO_Init() function  
//   80               
//   81          (+) Configure the DAC channel using DAC_Init()
//   82               
//   83          (+) Enable the DAC channel using DAC_Cmd()
//   84   
//   85     @endverbatim
//   86   *
//   87   ******************************************************************************
//   88   * @attention
//   89   *
//   90   * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
//   91   *
//   92   * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//   93   * You may not use this file except in compliance with the License.
//   94   * You may obtain a copy of the License at:
//   95   *
//   96   *        http://www.st.com/software_license_agreement_liberty_v2
//   97   *
//   98   * Unless required by applicable law or agreed to in writing, software 
//   99   * distributed under the License is distributed on an "AS IS" BASIS, 
//  100   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  101   * See the License for the specific language governing permissions and
//  102   * limitations under the License.
//  103   *
//  104   ******************************************************************************
//  105   */
//  106 
//  107 /* Includes ------------------------------------------------------------------*/
//  108 #include "stm32f0xx_dac.h"
//  109 
//  110 /** @addtogroup STM32F0xx_StdPeriph_Driver
//  111   * @{
//  112   */
//  113 
//  114 /** @defgroup DAC 
//  115   * @brief DAC driver modules
//  116   * @{
//  117   */ 
//  118 
//  119 /* Private typedef -----------------------------------------------------------*/
//  120 /* Private define ------------------------------------------------------------*/
//  121 /* CR register Mask */
//  122 #define CR_CLEAR_MASK              ((uint32_t)0x00000FFE) /* check the value of the mask */
//  123 
//  124 /* DAC Dual Channels SWTRIG masks */
//  125 #define DUAL_SWTRIG_SET            ((uint32_t)0x00000003) /*!< Only applicable for STM32F072 devices */
//  126 #define DUAL_SWTRIG_RESET          ((uint32_t)0xFFFFFFFC) /*!< Only applicable for STM32F072 devices */
//  127 
//  128 /* DHR registers offsets */
//  129 #define DHR12R1_OFFSET             ((uint32_t)0x00000008)
//  130 #define DHR12R2_OFFSET             ((uint32_t)0x00000014) /*!< Only applicable for STM32F072 devices */
//  131 #define DHR12RD_OFFSET             ((uint32_t)0x00000020) /*!< Only applicable for STM32F072 devices */
//  132 
//  133 /* DOR register offset */
//  134 #define DOR_OFFSET                 ((uint32_t)0x0000002C)
//  135 
//  136 /* Private macro -------------------------------------------------------------*/
//  137 /* Private variables ---------------------------------------------------------*/
//  138 /* Private function prototypes -----------------------------------------------*/
//  139 /* Private functions ---------------------------------------------------------*/
//  140 
//  141 /** @defgroup DAC_Private_Functions
//  142   * @{
//  143   */ 
//  144 
//  145 /** @defgroup DAC_Group1 DAC channels configuration
//  146  *  @brief   DAC channels configuration: trigger, output buffer, data format 
//  147  *
//  148 @verbatim
//  149  ===============================================================================
//  150   ##### DAC channels configuration: trigger, output buffer, data format #####
//  151  ===============================================================================  
//  152 
//  153 @endverbatim
//  154   * @{
//  155   */
//  156 
//  157 /**
//  158   * @brief  Deinitializes the DAC peripheral registers to their default reset values.
//  159   * @param  None
//  160   * @retval None
//  161   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  162 void DAC_DeInit(void)
//  163 {
DAC_DeInit:
        PUSH     {R4,LR}
//  164   /* Enable DAC reset state */
//  165   RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, ENABLE);
        MOVS     R4,#+128
        LSLS     R4,R4,#+22       ;; #+536870912
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       RCC_APB1PeriphResetCmd
//  166   /* Release DAC from reset state */
//  167   RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,R4
        BL       RCC_APB1PeriphResetCmd
//  168 }
        POP      {R4,PC}          ;; return
//  169 
//  170 /**
//  171   * @brief  Initializes the DAC peripheral according to the specified parameters
//  172   *         in the DAC_InitStruct.
//  173   * @param  DAC_Channel: the selected DAC channel. 
//  174   *          This parameter can be:
//  175   *            @arg DAC_Channel_1: DAC Channel1 selected
//  176   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  177   * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure that contains
//  178   *         the configuration information for the  specified DAC channel.
//  179   * @retval None
//  180   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  181 void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct)
//  182 {
DAC_Init:
        PUSH     {R4,R5}
//  183   uint32_t tmpreg1 = 0, tmpreg2 = 0;
//  184 
//  185   /* Check the DAC parameters */
//  186   assert_param(IS_DAC_TRIGGER(DAC_InitStruct->DAC_Trigger));
//  187   assert_param(IS_DAC_GENERATE_WAVE(DAC_InitStruct->DAC_WaveGeneration));
//  188   assert_param(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude));
//  189   assert_param(IS_DAC_OUTPUT_BUFFER_STATE(DAC_InitStruct->DAC_OutputBuffer));
//  190 
//  191 /*---------------------------- DAC CR Configuration --------------------------*/
//  192   /* Get the DAC CR value */
//  193   tmpreg1 = DAC->CR;
        LDR      R2,??DataTable13  ;; 0x40007400
        LDR      R3,[R2, #+0]
//  194   /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
//  195   tmpreg1 &= ~(CR_CLEAR_MASK << DAC_Channel);
//  196   /* Configure for the selected DAC channel: buffer output, trigger, 
//  197      wave generation, mask/amplitude for wave generation */
//  198   /* Set TSELx and TENx bits according to DAC_Trigger value */
//  199   /* Set WAVEx bits according to DAC_WaveGeneration value */
//  200   /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
//  201   /* Set BOFFx bit according to DAC_OutputBuffer value */   
//  202   tmpreg2 = (DAC_InitStruct->DAC_Trigger | DAC_InitStruct->DAC_WaveGeneration |
//  203              DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude | \ 
//  204              DAC_InitStruct->DAC_OutputBuffer);
//  205   /* Calculate CR register value depending on DAC_Channel */
//  206   tmpreg1 |= tmpreg2 << DAC_Channel;
//  207   /* Write to DAC CR */
//  208   DAC->CR = tmpreg1;
        LDR      R4,??DataTable13_1  ;; 0xffe
        LSLS     R4,R4,R0
        BICS     R3,R3,R4
        LDR      R4,[R1, #+0]
        LDR      R5,[R1, #+4]
        ORRS     R5,R5,R4
        LDR      R4,[R1, #+8]
        ORRS     R4,R4,R5
        LDR      R1,[R1, #+12]
        ORRS     R1,R1,R4
        LSLS     R1,R1,R0
        ORRS     R1,R1,R3
        STR      R1,[R2, #+0]
//  209 }
        POP      {R4,R5}
        BX       LR               ;; return
//  210 
//  211 /**
//  212   * @brief  Fills each DAC_InitStruct member with its default value.
//  213   * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure which will 
//  214   *         be initialized.
//  215   * @retval None
//  216   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  217 void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
//  218 {
//  219 /*--------------- Reset DAC init structure parameters values -----------------*/
//  220   /* Initialize the DAC_Trigger member */
//  221   DAC_InitStruct->DAC_Trigger = DAC_Trigger_None;
DAC_StructInit:
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  222   
//  223   /* Initialize the DAC_WaveGeneration member */
//  224   DAC_InitStruct->DAC_WaveGeneration = DAC_WaveGeneration_None;
        STR      R1,[R0, #+4]
//  225   
//  226   /* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
//  227   DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
        STR      R1,[R0, #+8]
//  228   
//  229   /* Initialize the DAC_OutputBuffer member */
//  230   DAC_InitStruct->DAC_OutputBuffer = DAC_OutputBuffer_Enable;
        STR      R1,[R0, #+12]
//  231 }
        BX       LR               ;; return
//  232 
//  233 /**
//  234   * @brief  Enables or disables the specified DAC channel.
//  235   * @param  DAC_Channel: The selected DAC channel. 
//  236   *          This parameter can be one of the following values:
//  237   *            @arg DAC_Channel_1: DAC Channel1 selected
//  238   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  239   * @param  NewState: new state of the DAC channel. 
//  240   *          This parameter can be: ENABLE or DISABLE.
//  241   * @note   When the DAC channel is enabled the trigger source can no more be modified.
//  242   * @retval None
//  243   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  244 void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState)
//  245 {
//  246   /* Check the parameters */
//  247   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  248   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  249 
//  250   if (NewState != DISABLE)
DAC_Cmd:
        MOVS     R2,#+1
        LSLS     R2,R2,R0
        LDR      R0,??DataTable13  ;; 0x40007400
        CMP      R1,#+0
        LDR      R1,[R0, #+0]
        BEQ      ??DAC_Cmd_0
//  251   {
//  252     /* Enable the selected DAC channel */
//  253     DAC->CR |= (DAC_CR_EN1 << DAC_Channel);
        ORRS     R2,R2,R1
        STR      R2,[R0, #+0]
        BX       LR
//  254   }
//  255   else
//  256   {
//  257     /* Disable the selected DAC channel */
//  258     DAC->CR &= (~(DAC_CR_EN1 << DAC_Channel));
??DAC_Cmd_0:
        BICS     R1,R1,R2
        STR      R1,[R0, #+0]
//  259   }
//  260 }
        BX       LR               ;; return
//  261 
//  262 /**
//  263   * @brief  Enables or disables the selected DAC channel software trigger.
//  264   * @param  DAC_Channel: The selected DAC channel. 
//  265   *          This parameter can be one of the following values:
//  266   *            @arg DAC_Channel_1: DAC Channel1 selected
//  267   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  268   * @param  NewState: new state of the selected DAC channel software trigger.
//  269   *          This parameter can be: ENABLE or DISABLE.
//  270   * @retval None
//  271   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  272 void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState)
//  273 {
//  274   /* Check the parameters */
//  275   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  276   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  277 
//  278   if (NewState != DISABLE)
DAC_SoftwareTriggerCmd:
        MOVS     R2,#+1
        LSRS     R0,R0,#+4
        LSLS     R2,R2,R0
        LDR      R0,??DataTable13_2  ;; 0x40007404
        CMP      R1,#+0
        LDR      R1,[R0, #+0]
        BEQ      ??DAC_SoftwareTriggerCmd_0
//  279   {
//  280     /* Enable software trigger for the selected DAC channel */
//  281     DAC->SWTRIGR |= (uint32_t)DAC_SWTRIGR_SWTRIG1 << (DAC_Channel >> 4);
        ORRS     R2,R2,R1
        STR      R2,[R0, #+0]
        BX       LR
//  282   }
//  283   else
//  284   {
//  285     /* Disable software trigger for the selected DAC channel */
//  286     DAC->SWTRIGR &= ~((uint32_t)DAC_SWTRIGR_SWTRIG1 << (DAC_Channel >> 4));
??DAC_SoftwareTriggerCmd_0:
        BICS     R1,R1,R2
        STR      R1,[R0, #+0]
//  287   }
//  288 }
        BX       LR               ;; return
//  289 
//  290 /**
//  291   * @brief  Enables or disables simultaneously the two DAC channels software triggers.
//  292   *         This function is applicable only for STM32F072 devices.  
//  293   * @param  NewState: new state of the DAC channels software triggers.
//  294   *          This parameter can be: ENABLE or DISABLE.
//  295   * @retval None
//  296   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  297 void DAC_DualSoftwareTriggerCmd(FunctionalState NewState)
//  298 {
//  299   /* Check the parameters */
//  300   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  301 
//  302   if (NewState != DISABLE)
DAC_DualSoftwareTriggerCmd:
        LDR      R1,??DataTable13_2  ;; 0x40007404
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??DAC_DualSoftwareTriggerCmd_0
//  303   {
//  304     /* Enable software trigger for both DAC channels */
//  305     DAC->SWTRIGR |= DUAL_SWTRIG_SET;
        MOVS     R2,#+3
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
        BX       LR
//  306   }
//  307   else
//  308   {
//  309     /* Disable software trigger for both DAC channels */
//  310     DAC->SWTRIGR &= DUAL_SWTRIG_RESET;
??DAC_DualSoftwareTriggerCmd_0:
        MOVS     R2,#+3
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
//  311   }
//  312 }
        BX       LR               ;; return
//  313 
//  314 /**
//  315   * @brief  Enables or disables the selected DAC channel wave generation.
//  316   *         This function is applicable only for STM32F072 devices.  
//  317   * @param  DAC_Channel: The selected DAC channel. 
//  318   *          This parameter can be:
//  319   *            @arg DAC_Channel_1: DAC Channel1 selected
//  320   *            @arg DAC_Channel_2: DAC Channel2 selected
//  321   * @param  DAC_Wave: specifies the wave type to enable or disable.
//  322   *          This parameter can be:
//  323   *            @arg DAC_Wave_Noise: noise wave generation
//  324   *            @arg DAC_Wave_Triangle: triangle wave generation
//  325   * @param  NewState: new state of the selected DAC channel wave generation.
//  326   *          This parameter can be: ENABLE or DISABLE.  
//  327   * @retval None
//  328   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  329 void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState)
//  330 {
//  331   /* Check the parameters */
//  332   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  333   assert_param(IS_DAC_WAVE(DAC_Wave)); 
//  334   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  335 
//  336   if (NewState != DISABLE)
DAC_WaveGenerationCmd:
        LSLS     R1,R1,R0
        LDR      R0,??DataTable13  ;; 0x40007400
        CMP      R2,#+0
        LDR      R2,[R0, #+0]
        BEQ      ??DAC_WaveGenerationCmd_0
//  337   {
//  338     /* Enable the selected wave generation for the selected DAC channel */
//  339     DAC->CR |= DAC_Wave << DAC_Channel;
        ORRS     R1,R1,R2
        STR      R1,[R0, #+0]
        BX       LR
//  340   }
//  341   else
//  342   {
//  343     /* Disable the selected wave generation for the selected DAC channel */
//  344     DAC->CR &= ~(DAC_Wave << DAC_Channel);
??DAC_WaveGenerationCmd_0:
        BICS     R2,R2,R1
        STR      R2,[R0, #+0]
//  345   }
//  346 }
        BX       LR               ;; return
//  347 
//  348 /**
//  349   * @brief  Set the specified data holding register value for DAC channel1.
//  350   * @param  DAC_Align: Specifies the data alignment for DAC channel1.
//  351   *          This parameter can be one of the following values:
//  352   *            @arg DAC_Align_8b_R: 8bit right data alignment selected
//  353   *            @arg DAC_Align_12b_L: 12bit left data alignment selected
//  354   *            @arg DAC_Align_12b_R: 12bit right data alignment selected
//  355   * @param  Data: Data to be loaded in the selected data holding register.
//  356   * @retval None
//  357   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  358 void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
//  359 {  
DAC_SetChannel1Data:
        SUB      SP,SP,#+4
//  360   __IO uint32_t tmp = 0;
        MOVS     R2,#+0
        STR      R2,[SP, #+0]
//  361   
//  362   /* Check the parameters */
//  363   assert_param(IS_DAC_ALIGN(DAC_Align));
//  364   assert_param(IS_DAC_DATA(Data));
//  365   
//  366   tmp = (uint32_t)DAC_BASE; 
        LDR      R2,??DataTable13  ;; 0x40007400
        STR      R2,[SP, #+0]
//  367   tmp += DHR12R1_OFFSET + DAC_Align;
        LDR      R2,[SP, #+0]
        ADDS     R0,R0,#+8
        B.N      ?Subroutine0
//  368 
//  369   /* Set the DAC channel1 selected data holding register */
//  370   *(__IO uint32_t *) tmp = Data;
//  371 }
//  372 
//  373 /**
//  374   * @brief  Sets the specified data holding register value for DAC channel2.
//  375   *         This function is applicable only for STM32F072 devices.  
//  376   * @param  DAC_Align: Specifies the data alignment for DAC channel2.
//  377   *          This parameter can be:
//  378   *            @arg DAC_Align_8b_R: 8bit right data alignment selected
//  379   *            @arg DAC_Align_12b_L: 12bit left data alignment selected
//  380   *            @arg DAC_Align_12b_R: 12bit right data alignment selected
//  381   * @param  Data: Data to be loaded in the selected data holding register.
//  382   * @retval None
//  383   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  384 void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data)
//  385 {
DAC_SetChannel2Data:
        SUB      SP,SP,#+4
//  386   __IO uint32_t tmp = 0;
        MOVS     R2,#+0
        STR      R2,[SP, #+0]
//  387 
//  388   /* Check the parameters */
//  389   assert_param(IS_DAC_ALIGN(DAC_Align));
//  390   assert_param(IS_DAC_DATA(Data));
//  391   
//  392   tmp = (uint32_t)DAC_BASE;
        LDR      R2,??DataTable13  ;; 0x40007400
        STR      R2,[SP, #+0]
//  393   tmp += DHR12R2_OFFSET + DAC_Align;
        LDR      R2,[SP, #+0]
        ADDS     R0,R0,#+20
        REQUIRE ?Subroutine0
        ;; // Fall through to label ?Subroutine0
//  394 
//  395   /* Set the DAC channel2 selected data holding register */
//  396   *(__IO uint32_t *)tmp = Data;
//  397 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine0:
        ADDS     R0,R2,R0
        STR      R0,[SP, #+0]
        LDR      R0,[SP, #+0]
        STR      R1,[R0, #+0]
        ADD      SP,SP,#+4
        BX       LR               ;; return
//  398 
//  399 /**
//  400   * @brief  Sets the specified data holding register value for dual channel DAC.
//  401   *         This function is applicable only for STM32F072 devices.  
//  402   * @param  DAC_Align: Specifies the data alignment for dual channel DAC.
//  403   *          This parameter can be:
//  404   *            @arg DAC_Align_8b_R: 8bit right data alignment selected
//  405   *            @arg DAC_Align_12b_L: 12bit left data alignment selected
//  406   *            @arg DAC_Align_12b_R: 12bit right data alignment selected
//  407   * @param  Data2: Data for DAC Channel2 to be loaded in the selected data holding register.
//  408   * @param  Data1: Data for DAC Channel1 to be loaded in the selected data  holding register.
//  409   * @note   In dual mode, a unique register access is required to write in both
//  410   *          DAC channels at the same time.
//  411   * @retval None
//  412   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  413 void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1)
//  414 {
//  415   uint32_t data = 0, tmp = 0;
//  416   
//  417   /* Check the parameters */
//  418   assert_param(IS_DAC_ALIGN(DAC_Align));
//  419   assert_param(IS_DAC_DATA(Data1));
//  420   assert_param(IS_DAC_DATA(Data2));
//  421   
//  422   /* Calculate and set dual DAC data holding register value */
//  423   if (DAC_Align == DAC_Align_8b_R)
DAC_SetDualChannelData:
        CMP      R0,#+8
        BNE      ??DAC_SetDualChannelData_0
//  424   {
//  425     data = ((uint32_t)Data2 << 8) | Data1; 
        LSLS     R1,R1,#+8
        B        ??DAC_SetDualChannelData_1
//  426   }
//  427   else
//  428   {
//  429     data = ((uint32_t)Data2 << 16) | Data1;
??DAC_SetDualChannelData_0:
        LSLS     R1,R1,#+16
??DAC_SetDualChannelData_1:
        ORRS     R2,R2,R1
//  430   }
//  431   
//  432   tmp = (uint32_t)DAC_BASE;
//  433   tmp += DHR12RD_OFFSET + DAC_Align;
//  434 
//  435   /* Set the dual DAC selected data holding register */
//  436   *(__IO uint32_t *)tmp = data;
        LDR      R1,??DataTable13_3  ;; 0x40007420
        STR      R2,[R0, R1]
//  437 }
        BX       LR               ;; return
//  438 
//  439 /**
//  440   * @brief  Returns the last data output value of the selected DAC channel.
//  441   * @param  DAC_Channel: The selected DAC channel. 
//  442   *          This parameter can be one of the following values:
//  443   *            @arg DAC_Channel_1: DAC Channel1 selected
//  444   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  445   * @retval The selected DAC channel data output value.
//  446   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  447 uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel)
//  448 {
DAC_GetDataOutputValue:
        SUB      SP,SP,#+4
//  449   __IO uint32_t tmp = 0;
        MOVS     R1,#+0
        STR      R1,[SP, #+0]
//  450   
//  451   /* Check the parameters */
//  452   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  453   
//  454   tmp = (uint32_t) DAC_BASE ;
        LDR      R1,??DataTable13  ;; 0x40007400
        STR      R1,[SP, #+0]
//  455   tmp += DOR_OFFSET + ((uint32_t)DAC_Channel >> 2);
        LDR      R1,[SP, #+0]
        LSRS     R0,R0,#+2
        ADDS     R0,R0,#+44
        ADDS     R0,R1,R0
        STR      R0,[SP, #+0]
//  456   
//  457   /* Returns the DAC channel data output register value */
//  458   return (uint16_t) (*(__IO uint32_t*) tmp);
        LDR      R0,[SP, #+0]
        LDR      R0,[R0, #+0]
        UXTH     R0,R0
        ADD      SP,SP,#+4
        BX       LR               ;; return
//  459 }
//  460 
//  461 /**
//  462   * @}
//  463   */
//  464 
//  465 /** @defgroup DAC_Group2 DMA management functions
//  466  *  @brief   DMA management functions
//  467  *
//  468 @verbatim   
//  469  ===============================================================================
//  470                     ##### DMA management functions #####
//  471  ===============================================================================  
//  472 
//  473 @endverbatim
//  474   * @{
//  475   */
//  476 
//  477 /**
//  478   * @brief  Enables or disables the specified DAC channel DMA request.
//  479   *         When enabled DMA1 is generated when an external trigger (EXTI Line9,
//  480   *         TIM2, TIM3, TIM6 or TIM15  but not a software trigger) occurs
//  481   * @param  DAC_Channel: the selected DAC channel.
//  482   *          This parameter can be one of the following values:
//  483   *            @arg DAC_Channel_1: DAC Channel1 selected
//  484   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  485   * @param  NewState: new state of the selected DAC channel DMA request.
//  486   *          This parameter can be: ENABLE or DISABLE.
//  487   * @note   The DAC channel1 is mapped on DMA1 channel3 which must be already configured. 
//  488   * @note   The DAC channel2 is mapped on DMA1 channel4 which must be already configured.  
//  489   * @retval None
//  490   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  491 void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState)
//  492 {
//  493   /* Check the parameters */
//  494   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  495   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  496 
//  497   if (NewState != DISABLE)
DAC_DMACmd:
        MOVS     R2,#+128
        LSLS     R2,R2,#+5        ;; #+4096
        LSLS     R2,R2,R0
        LDR      R0,??DataTable13  ;; 0x40007400
        CMP      R1,#+0
        LDR      R1,[R0, #+0]
        BEQ      ??DAC_DMACmd_0
//  498   {
//  499     /* Enable the selected DAC channel DMA request */
//  500     DAC->CR |= (DAC_CR_DMAEN1 << DAC_Channel);
        ORRS     R2,R2,R1
        STR      R2,[R0, #+0]
        BX       LR
//  501   }
//  502   else
//  503   {
//  504     /* Disable the selected DAC channel DMA request */
//  505     DAC->CR &= (~(DAC_CR_DMAEN1 << DAC_Channel));
??DAC_DMACmd_0:
        BICS     R1,R1,R2
        STR      R1,[R0, #+0]
//  506   }
//  507 }
        BX       LR               ;; return
//  508 
//  509 /**
//  510   * @}
//  511   */
//  512 
//  513 /** @defgroup DAC_Group3 Interrupts and flags management functions
//  514  *  @brief   Interrupts and flags management functions
//  515  *
//  516 @verbatim   
//  517  ===============================================================================
//  518             ##### Interrupts and flags management functions #####
//  519  ===============================================================================  
//  520 
//  521 @endverbatim
//  522   * @{
//  523   */
//  524 
//  525 /**
//  526   * @brief  Enables or disables the specified DAC interrupts.
//  527   * @param  DAC_Channel: The selected DAC channel. 
//  528   *          This parameter can be:
//  529   *            @arg DAC_Channel_1: DAC Channel1 selected
//  530   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  531   * @param  DAC_IT: specifies the DAC interrupt sources to be enabled or disabled. 
//  532   *          This parameter can be the following values:
//  533   *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask
//  534   * @note   The DMA underrun occurs when a second external trigger arrives before the 
//  535   *         acknowledgement for the first external trigger is received (first request).
//  536   * @param  NewState: new state of the specified DAC interrupts.
//  537   *          This parameter can be: ENABLE or DISABLE.
//  538   * @retval None
//  539   */ 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  540 void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState)  
//  541 {
//  542   /* Check the parameters */
//  543   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  544   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  545   assert_param(IS_DAC_IT(DAC_IT)); 
//  546 
//  547   if (NewState != DISABLE)
DAC_ITConfig:
        LSLS     R1,R1,R0
        LDR      R0,??DataTable13  ;; 0x40007400
        CMP      R2,#+0
        LDR      R2,[R0, #+0]
        BEQ      ??DAC_ITConfig_0
//  548   {
//  549     /* Enable the selected DAC interrupts */
//  550     DAC->CR |=  (DAC_IT << DAC_Channel);
        ORRS     R1,R1,R2
        STR      R1,[R0, #+0]
        BX       LR
//  551   }
//  552   else
//  553   {
//  554     /* Disable the selected DAC interrupts */
//  555     DAC->CR &= (~(uint32_t)(DAC_IT << DAC_Channel));
??DAC_ITConfig_0:
        BICS     R2,R2,R1
        STR      R2,[R0, #+0]
//  556   }
//  557 }
        BX       LR               ;; return
//  558 
//  559 /**
//  560   * @brief  Checks whether the specified DAC flag is set or not.
//  561   * @param  DAC_Channel: The selected DAC channel. 
//  562   *          This parameter can be one of the following values:
//  563   *            @arg DAC_Channel_1: DAC Channel1 selected
//  564   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  565   * @param  DAC_FLAG: specifies the flag to check. 
//  566   *          This parameter can be only of the following value:
//  567   *            @arg DAC_FLAG_DMAUDR: DMA underrun flag
//  568   * @note   The DMA underrun occurs when a second external trigger arrives before the 
//  569   *         acknowledgement for the first external trigger is received (first request).
//  570   * @retval The new state of DAC_FLAG (SET or RESET).
//  571   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  572 FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG)
//  573 {
DAC_GetFlagStatus:
        MOVS     R2,R0
//  574   FlagStatus bitstatus = RESET;
        MOVS     R0,#+0
//  575   /* Check the parameters */
//  576   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  577   assert_param(IS_DAC_FLAG(DAC_FLAG));
//  578 
//  579   /* Check the status of the specified DAC flag */
//  580   if ((DAC->SR & (DAC_FLAG << DAC_Channel)) != (uint8_t)RESET)
        LDR      R3,??DataTable13_4  ;; 0x40007434
        LDR      R3,[R3, #+0]
        LSLS     R1,R1,R2
        ANDS     R1,R1,R3
        BEQ      ??DAC_GetFlagStatus_0
//  581   {
//  582     /* DAC_FLAG is set */
//  583     bitstatus = SET;
        MOVS     R0,#+1
//  584   }
//  585   else
//  586   {
//  587     /* DAC_FLAG is reset */
//  588     bitstatus = RESET;
//  589   }
//  590   /* Return the DAC_FLAG status */
//  591   return  bitstatus;
??DAC_GetFlagStatus_0:
        BX       LR               ;; return
//  592 }
//  593 
//  594 /**
//  595   * @brief  Clears the DAC channel's pending flags.
//  596   * @param  DAC_Channel: The selected DAC channel. 
//  597   *          This parameter can be one of the following values:
//  598   *            @arg DAC_Channel_1: DAC Channel1 selected
//  599   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  600   * @param  DAC_FLAG: specifies the flag to clear. 
//  601   *          This parameter can be of the following value:
//  602   *            @arg DAC_FLAG_DMAUDR: DMA underrun flag                           
//  603   * @retval None
//  604   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  605 void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG)
//  606 {
//  607   /* Check the parameters */
//  608   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  609   assert_param(IS_DAC_FLAG(DAC_FLAG));
//  610 
//  611   /* Clear the selected DAC flags */
//  612   DAC->SR = (DAC_FLAG << DAC_Channel);
DAC_ClearFlag:
        LSLS     R1,R1,R0
        LDR      R0,??DataTable13_4  ;; 0x40007434
        STR      R1,[R0, #+0]
//  613 }
        BX       LR               ;; return
//  614 
//  615 /**
//  616   * @brief  Checks whether the specified DAC interrupt has occurred or not.
//  617   * @param  DAC_Channel: The selected DAC channel. 
//  618   *          This parameter can be one of the following values:
//  619   *            @arg DAC_Channel_1: DAC Channel1 selected
//  620   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  621   * @param  DAC_IT: specifies the DAC interrupt source to check. 
//  622   *          This parameter can be the following values:
//  623   *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask
//  624   * @note   The DMA underrun occurs when a second external trigger arrives before the 
//  625   *         acknowledgement for the first external trigger is received (first request).
//  626   * @retval The new state of DAC_IT (SET or RESET).
//  627   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  628 ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT)
//  629 {
DAC_GetITStatus:
        MOVS     R2,R0
//  630   ITStatus bitstatus = RESET;
        MOVS     R0,#+0
//  631   uint32_t enablestatus = 0;
//  632   
//  633   /* Check the parameters */
//  634   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  635   assert_param(IS_DAC_IT(DAC_IT));
//  636 
//  637   /* Get the DAC_IT enable bit status */
//  638   enablestatus = (DAC->CR & (DAC_IT << DAC_Channel)) ;
        LSLS     R1,R1,R2
        LDR      R2,??DataTable13  ;; 0x40007400
        LDR      R3,[R2, #+0]
        ANDS     R3,R3,R1
//  639   
//  640   /* Check the status of the specified DAC interrupt */
//  641   if (((DAC->SR & (DAC_IT << DAC_Channel)) != (uint32_t)RESET) && enablestatus)
        LDR      R2,[R2, #+52]
        ANDS     R1,R1,R2
        BEQ      ??DAC_GetITStatus_0
        CMP      R3,#+0
        BEQ      ??DAC_GetITStatus_0
//  642   {
//  643     /* DAC_IT is set */
//  644     bitstatus = SET;
        MOVS     R0,#+1
//  645   }
//  646   else
//  647   {
//  648     /* DAC_IT is reset */
//  649     bitstatus = RESET;
//  650   }
//  651   /* Return the DAC_IT status */
//  652   return  bitstatus;
??DAC_GetITStatus_0:
        BX       LR               ;; return
//  653 }
//  654 
//  655 /**
//  656   * @brief  Clears the DAC channel's interrupt pending bits.
//  657   * @param  DAC_Channel: The selected DAC channel. 
//  658   *          This parameter can be one of the following values:
//  659   *            @arg DAC_Channel_1: DAC Channel1 selected
//  660   *            @arg DAC_Channel_2: DAC Channel2 selected, applicable only for STM32F072 devices
//  661   * @param  DAC_IT: specifies the DAC interrupt pending bit to clear.
//  662   *          This parameter can be the following values:
//  663   *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                                                    
//  664   * @retval None
//  665   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  666 void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT)
//  667 {
//  668   /* Check the parameters */
//  669   assert_param(IS_DAC_CHANNEL(DAC_Channel));
//  670   assert_param(IS_DAC_IT(DAC_IT)); 
//  671 
//  672   /* Clear the selected DAC interrupt pending bits */
//  673   DAC->SR = (DAC_IT << DAC_Channel);
DAC_ClearITPendingBit:
        LSLS     R1,R1,R0
        LDR      R0,??DataTable13_4  ;; 0x40007434
        STR      R1,[R0, #+0]
//  674 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13:
        DC32     0x40007400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_1:
        DC32     0xffe

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_2:
        DC32     0x40007404

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_3:
        DC32     0x40007420

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_4:
        DC32     0x40007434

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  675 
//  676 /**
//  677   * @}
//  678   */
//  679 
//  680 /**
//  681   * @}
//  682   */ 
//  683 
//  684 /**
//  685   * @}
//  686   */ 
//  687 
//  688 /**
//  689   * @}
//  690   */ 
//  691 
//  692 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
// 386 bytes in section .text
// 
// 386 bytes of CODE memory
//
//Errors: none
//Warnings: none
