///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:51 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    adc.c                                                   /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    adc.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER    /
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
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_adc. /
//                    s                                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_adc

        #define SHT_PROGBITS 0x1

        EXTERN RCC_APB2PeriphResetCmd

        PUBLIC ADC_AnalogWatchdogCmd
        PUBLIC ADC_AnalogWatchdogSingleChannelCmd
        PUBLIC ADC_AnalogWatchdogSingleChannelConfig
        PUBLIC ADC_AnalogWatchdogThresholdsConfig
        PUBLIC ADC_AutoPowerOffCmd
        PUBLIC ADC_ChannelConfig
        PUBLIC ADC_ClearFlag
        PUBLIC ADC_ClearITPendingBit
        PUBLIC ADC_ClockModeConfig
        PUBLIC ADC_Cmd
        PUBLIC ADC_ContinuousModeCmd
        PUBLIC ADC_DMACmd
        PUBLIC ADC_DMARequestModeConfig
        PUBLIC ADC_DeInit
        PUBLIC ADC_DiscModeCmd
        PUBLIC ADC_GetCalibrationFactor
        PUBLIC ADC_GetConversionValue
        PUBLIC ADC_GetFlagStatus
        PUBLIC ADC_GetITStatus
        PUBLIC ADC_ITConfig
        PUBLIC ADC_Init
        PUBLIC ADC_JitterCmd
        PUBLIC ADC_OverrunModeCmd
        PUBLIC ADC_StartOfConversion
        PUBLIC ADC_StopOfConversion
        PUBLIC ADC_StructInit
        PUBLIC ADC_TempSensorCmd
        PUBLIC ADC_VbatCmd
        PUBLIC ADC_VrefintCmd
        PUBLIC ADC_WaitModeCmd

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_adc.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_adc.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides firmware functions to manage the following 
//    8   *          functionalities of the Analog to Digital Convertor (ADC) peripheral:
//    9   *           + Initialization and Configuration
//   10   *           + Power saving
//   11   *           + Analog Watchdog configuration
//   12   *           + Temperature Sensor, Vrefint (Internal Reference Voltage) and 
//   13   *             Vbat (Voltage battery) management 
//   14   *           + ADC Channels Configuration
//   15   *           + ADC Channels DMA Configuration
//   16   *           + Interrupts and flags management
//   17   *
//   18   *  @verbatim
//   19 ================================================================================
//   20                       ##### How to use this driver #####
//   21 ================================================================================
//   22     [..]
//   23     (#) Enable the ADC interface clock using 
//   24         RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
//   25     (#) ADC pins configuration
//   26        (++) Enable the clock for the ADC GPIOs using the following function:
//   27             RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx, ENABLE);   
//   28        (++) Configure these ADC pins in analog mode using GPIO_Init();  
//   29     (#) Configure the ADC conversion resolution, data alignment, external
//   30         trigger and edge, scan direction and Enable/Disable the continuous mode
//   31         using the ADC_Init() function.
//   32     (#) Activate the ADC peripheral using ADC_Cmd() function.
//   33 
//   34     *** ADC channels group configuration ***
//   35     ============================================
//   36     [..] 
//   37     (+) To configure the ADC channels features, use ADC_Init() and 
//   38         ADC_ChannelConfig() functions.
//   39     (+) To activate the continuous mode, use the ADC_ContinuousModeCmd()
//   40         function.
//   41     (+) To activate the Discontinuous mode, use the ADC_DiscModeCmd() functions. 
//   42     (+) To activate the overrun mode, use the ADC_OverrunModeCmd() functions.
//   43     (+) To activate the calibration mode, use the ADC_GetCalibrationFactor() functions.
//   44     (+) To read the ADC converted values, use the ADC_GetConversionValue()
//   45         function.
//   46 
//   47     *** DMA for ADC channels features configuration ***
//   48     =============================================================
//   49     [..] 
//   50     (+) To enable the DMA mode for ADC channels group, use the ADC_DMACmd() function.
//   51     (+) To configure the DMA transfer request, use ADC_DMARequestModeConfig() function.
//   52 
//   53   *  @endverbatim
//   54   *
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
//   76 #include "stm32f0xx_adc.h"
//   77 #include "stm32f0xx_rcc.h"
//   78 
//   79 /** @addtogroup STM32F0xx_StdPeriph_Driver
//   80   * @{
//   81   */
//   82 
//   83 /** @defgroup ADC 
//   84   * @brief ADC driver modules
//   85   * @{
//   86   */
//   87 
//   88 /* Private typedef -----------------------------------------------------------*/
//   89 /* Private define ------------------------------------------------------------*/
//   90 /* ADC CFGR mask */
//   91 #define CFGR1_CLEAR_MASK           ((uint32_t)0xFFFFD203)
//   92 
//   93 /* Calibration time out */
//   94 #define CALIBRATION_TIMEOUT       ((uint32_t)0x0000F000)
//   95 
//   96 /* Private macro -------------------------------------------------------------*/
//   97 /* Private variables ---------------------------------------------------------*/
//   98 /* Private function prototypes -----------------------------------------------*/
//   99 /* Private functions ---------------------------------------------------------*/
//  100 
//  101 /** @defgroup ADC_Private_Functions
//  102   * @{
//  103   */
//  104 
//  105 /** @defgroup ADC_Group1 Initialization and Configuration functions
//  106  *  @brief   Initialization and Configuration functions 
//  107  *
//  108 @verbatim
//  109  ===============================================================================
//  110           ##### Initialization and Configuration functions #####
//  111  ===============================================================================
//  112     [..] This section provides functions allowing to:
//  113         (+) Initialize and configure the ADC Prescaler
//  114         (+) ADC Conversion Resolution (12bit..6bit)
//  115         (+) ADC Continuous Conversion Mode (Continuous or Single conversion)
//  116         (+) External trigger Edge and source 
//  117         (+) Converted data alignment (left or right)
//  118         (+) The direction in which the channels will be scanned in the sequence
//  119         (+) Enable or disable the ADC peripheral
//  120    
//  121 @endverbatim
//  122   * @{
//  123   */
//  124 
//  125 /**
//  126   * @brief  Deinitializes ADC1 peripheral registers to their default reset values.
//  127   * @param  ADCx: where x can be 1 to select the ADC peripheral.
//  128   * @retval None
//  129   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  130 void ADC_DeInit(ADC_TypeDef* ADCx)
//  131 {
//  132   /* Check the parameters */
//  133   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  134 
//  135   if(ADCx == ADC1)
ADC_DeInit:
        LDR      R1,??DataTable13  ;; 0x40012400
        CMP      R0,R1
        BEQ      ??ADC_DeInit_0
        BX       LR
//  136   {
//  137     /* Enable ADC1 reset state */
//  138     RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
??ADC_DeInit_0:
        PUSH     {R4,LR}
        LSRS     R4,R1,#+21
        MOVS     R1,#+1
        MOVS     R0,R4
        BL       RCC_APB2PeriphResetCmd
//  139 
//  140     /* Release ADC1 from reset state */
//  141     RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);
        MOVS     R1,#+0
        MOVS     R0,R4
        BL       RCC_APB2PeriphResetCmd
//  142   }
//  143 }
        POP      {R4,PC}          ;; return
//  144 
//  145 /**
//  146   * @brief  Initializes the ADCx peripheral according to the specified parameters
//  147   *         in the ADC_InitStruct.
//  148   * @note   This function is used to configure the global features of the ADC ( 
//  149   *         Resolution, Data Alignment, continuous mode activation, External 
//  150   *         trigger source and edge, Sequence Scan Direction).   
//  151   * @param  ADCx: where x can be 1 to select the ADC peripheral.
//  152   * @param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure that contains 
//  153   *         the configuration information for the specified ADC peripheral.
//  154   * @retval None
//  155   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  156 void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
//  157 {
//  158   uint32_t tmpreg = 0;
//  159 
//  160   /* Check the parameters */
//  161   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  162   assert_param(IS_ADC_RESOLUTION(ADC_InitStruct->ADC_Resolution));
//  163   assert_param(IS_FUNCTIONAL_STATE(ADC_InitStruct->ADC_ContinuousConvMode));
//  164   assert_param(IS_ADC_EXT_TRIG_EDGE(ADC_InitStruct->ADC_ExternalTrigConvEdge));
//  165   assert_param(IS_ADC_EXTERNAL_TRIG_CONV(ADC_InitStruct->ADC_ExternalTrigConv));
//  166   assert_param(IS_ADC_DATA_ALIGN(ADC_InitStruct->ADC_DataAlign));
//  167   assert_param(IS_ADC_SCAN_DIRECTION(ADC_InitStruct->ADC_ScanDirection)); 
//  168 
//  169   /* Get the ADCx CFGR value */
//  170   tmpreg = ADCx->CFGR1;
//  171 
//  172   /* Clear SCANDIR, RES[1:0], ALIGN, EXTSEL[2:0], EXTEN[1:0] and CONT bits */
//  173   tmpreg &= CFGR1_CLEAR_MASK;
//  174 
//  175   /*---------------------------- ADCx CFGR Configuration ---------------------*/
//  176 
//  177   /* Set RES[1:0] bits according to ADC_Resolution value */
//  178   /* Set CONT bit according to ADC_ContinuousConvMode value */
//  179   /* Set EXTEN[1:0] bits according to ADC_ExternalTrigConvEdge value */
//  180   /* Set EXTSEL[2:0] bits according to ADC_ExternalTrigConv value */
//  181   /* Set ALIGN bit according to ADC_DataAlign value */
//  182   /* Set SCANDIR bit according to ADC_ScanDirection value */
//  183  
//  184   tmpreg  |= (uint32_t)(ADC_InitStruct->ADC_Resolution | ((uint32_t)(ADC_InitStruct->ADC_ContinuousConvMode) << 13) |
//  185              ADC_InitStruct->ADC_ExternalTrigConvEdge | ADC_InitStruct->ADC_ExternalTrigConv |
//  186              ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ScanDirection);
//  187 
//  188   /* Write to ADCx CFGR */
//  189   ADCx->CFGR1 = tmpreg;
ADC_Init:
        LDR      R2,[R0, #+12]
        LDR      R3,??DataTable13_1  ;; 0xffffd203
        ANDS     R3,R3,R2
        LDR      R2,[R1, #+0]
        ORRS     R2,R2,R3
        LDRB     R3,[R1, #+4]
        LSLS     R3,R3,#+13
        ORRS     R3,R3,R2
        LDR      R2,[R1, #+8]
        ORRS     R2,R2,R3
        LDR      R3,[R1, #+12]
        ORRS     R3,R3,R2
        LDR      R2,[R1, #+16]
        ORRS     R2,R2,R3
        LDR      R1,[R1, #+20]
        ORRS     R1,R1,R2
        STR      R1,[R0, #+12]
//  190 }
        BX       LR               ;; return
//  191 
//  192 /**
//  193   * @brief  Fills each ADC_InitStruct member with its default value.
//  194   * @note   This function is used to initialize the global features of the ADC ( 
//  195   *         Resolution, Data Alignment, continuous mode activation, External 
//  196   *         trigger source and edge, Sequence Scan Direction).
//  197   * @param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure which will 
//  198   *         be initialized.
//  199   * @retval None
//  200   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  201 void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
//  202 {
//  203   /* Reset ADC init structure parameters values */
//  204   /* Initialize the ADC_Resolution member */
//  205   ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b;
ADC_StructInit:
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  206 
//  207    /* Initialize the ADC_ContinuousConvMode member */
//  208   ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;
        STRB     R1,[R0, #+4]
//  209 
//  210   /* Initialize the ADC_ExternalTrigConvEdge member */
//  211   ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
        STR      R1,[R0, #+8]
//  212 
//  213   /* Initialize the ADC_ExternalTrigConv member */
//  214   ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
        STR      R1,[R0, #+12]
//  215 
//  216   /* Initialize the ADC_DataAlign member */
//  217   ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;
        STR      R1,[R0, #+16]
//  218 
//  219   /* Initialize the ADC_ScanDirection member */
//  220   ADC_InitStruct->ADC_ScanDirection = ADC_ScanDirection_Upward;
        STR      R1,[R0, #+20]
//  221 }
        BX       LR               ;; return
//  222 
//  223 /**
//  224   * @brief  Enables or disables the specified ADC peripheral.
//  225   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  226   * @param  NewState: new state of the ADCx peripheral. 
//  227   *          This parameter can be: ENABLE or DISABLE.
//  228   * @retval None
//  229   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  230 void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  231 {
//  232   /* Check the parameters */
//  233   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  234   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  235 
//  236   if (NewState != DISABLE)
ADC_Cmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+8]
        BEQ      ??ADC_Cmd_0
//  237   {
//  238     /* Set the ADEN bit to Enable the ADC peripheral */
//  239     ADCx->CR |= (uint32_t)ADC_CR_ADEN;
        MOVS     R2,#+1
        B        ??ADC_Cmd_1
//  240   }
//  241   else
//  242   {
//  243     /* Set the ADDIS to Disable the ADC peripheral */
//  244     ADCx->CR |= (uint32_t)ADC_CR_ADDIS;
??ADC_Cmd_0:
        MOVS     R2,#+2
??ADC_Cmd_1:
        ORRS     R2,R2,R1
        STR      R2,[R0, #+8]
//  245   }
//  246 }
        BX       LR               ;; return
//  247 
//  248 /**
//  249   * @brief  Configure the ADC to either be clocked by the asynchronous clock(which is
//  250   *         independent, the dedicated 14MHz clock) or the synchronous clock derived from
//  251   *         the APB clock of the ADC bus interface divided by 2 or 4
//  252   * @note   This function can be called only when ADC is disabled.
//  253   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  254   * @param  ADC_ClockMode: This parameter can be :
//  255   *            @arg ADC_ClockMode_AsynClk: ADC clocked by the dedicated 14MHz clock
//  256   *            @arg ADC_ClockMode_SynClkDiv2: ADC clocked by PCLK/2
//  257   *            @arg ADC_ClockMode_SynClkDiv4: ADC clocked by PCLK/4  
//  258   * @retval None
//  259   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  260 void ADC_ClockModeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ClockMode)
//  261 {
//  262   /* Check the parameters */
//  263   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  264   assert_param(IS_ADC_CLOCKMODE(ADC_ClockMode));
//  265 
//  266     /* Configure the ADC Clock mode according to ADC_ClockMode */
//  267     ADCx->CFGR2 = (uint32_t)ADC_ClockMode;
ADC_ClockModeConfig:
        STR      R1,[R0, #+16]
//  268 
//  269 }
        BX       LR               ;; return
//  270 
//  271 /**
//  272   * @brief  Enables or disables the jitter when the ADC is clocked by PCLK div2
//  273   *         or div4
//  274   * @note   This function is obsolete and maintained for legacy purpose only. ADC_ClockModeConfig()
//  275   *         function should be used instead.  
//  276   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  277   * @param  ADC_JitterOff: This parameter can be :
//  278   *            @arg ADC_JitterOff_PCLKDiv2: Remove jitter when ADC is clocked by PLCK divided by 2
//  279   *            @arg ADC_JitterOff_PCLKDiv4: Remove jitter when ADC is clocked by PLCK divided by 4
//  280   * @param  NewState: new state of the ADCx jitter. 
//  281   *          This parameter can be: ENABLE or DISABLE.
//  282   * @retval None
//  283   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  284 void ADC_JitterCmd(ADC_TypeDef* ADCx, uint32_t ADC_JitterOff, FunctionalState NewState)
//  285 {
//  286   /* Check the parameters */
//  287   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  288   assert_param(IS_ADC_JITTEROFF(ADC_JitterOff));
//  289   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  290 
//  291   if (NewState != DISABLE)
ADC_JitterCmd:
        CMP      R2,#+0
        LDR      R2,[R0, #+16]
        BEQ      ??ADC_JitterCmd_0
//  292   {
//  293     /* Disable Jitter */
//  294     ADCx->CFGR2 |= (uint32_t)ADC_JitterOff;
        ORRS     R1,R1,R2
        STR      R1,[R0, #+16]
        BX       LR
//  295   }
//  296   else
//  297   {
//  298     /* Enable Jitter */
//  299     ADCx->CFGR2 &= (uint32_t)(~ADC_JitterOff);
??ADC_JitterCmd_0:
        BICS     R2,R2,R1
        STR      R2,[R0, #+16]
//  300   }
//  301 }
        BX       LR               ;; return
//  302 
//  303 /**
//  304   * @}
//  305   */
//  306 
//  307 /** @defgroup ADC_Group2 Power saving functions
//  308  *  @brief   Power saving functions 
//  309  *
//  310 @verbatim
//  311  ===============================================================================
//  312           ##### Power saving functions #####
//  313  ===============================================================================
//  314     [..] This section provides functions allowing to reduce power consumption.
//  315     [..] The two function must be combined to get the maximal benefits:
//  316          When the ADC frequency is higher than the CPU one, it is recommended to 
//  317          (#) Enable the Auto Delayed Conversion mode : 
//  318              ==> using ADC_WaitModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
//  319          (#) Enable the power off in Delay phases :
//  320              ==> using ADC_AutoPowerOffCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
//  321 
//  322 @endverbatim
//  323   * @{
//  324   */
//  325 
//  326 /**
//  327   * @brief  Enables or disables the ADC Power Off.
//  328   * @note   ADC power-on and power-off can be managed by hardware to cut the 
//  329   *         consumption when the ADC is not converting. 
//  330   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  331   * @note   The ADC can be powered down: 
//  332   *         - During the Auto delay phase:  The ADC is powered on again at the end
//  333   *           of the delay (until the previous data is read from the ADC data register). 
//  334   *         - During the ADC is waiting for a trigger event: The ADC is powered up
//  335   *           at the next trigger event (when the conversion is started).
//  336   * @param  NewState: new state of the ADCx power Off. 
//  337   *          This parameter can be: ENABLE or DISABLE.
//  338   * @retval None
//  339   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  340 void ADC_AutoPowerOffCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  341 {
//  342   /* Check the parameters */
//  343   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  344   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  345   
//  346   if (NewState != DISABLE)
ADC_AutoPowerOffCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_AutoPowerOffCmd_0
//  347   {
//  348     /* Enable the ADC Automatic Power-Off */
//  349     ADCx->CFGR1 |= ADC_CFGR1_AUTOFF;
        MOVS     R2,#+128
        LSLS     R2,R2,#+8        ;; #+32768
        ORRS     R2,R2,R1
        B        ??ADC_AutoPowerOffCmd_1
//  350   }
//  351   else
//  352   {
//  353     /* Disable the ADC Automatic Power-Off */
//  354     ADCx->CFGR1 &= (uint32_t)~ADC_CFGR1_AUTOFF;
??ADC_AutoPowerOffCmd_0:
        LDR      R2,??DataTable13_2  ;; 0xffff7fff
        ANDS     R2,R2,R1
??ADC_AutoPowerOffCmd_1:
        STR      R2,[R0, #+12]
//  355   }
//  356 }
        BX       LR               ;; return
//  357 
//  358 /**
//  359   * @brief  Enables or disables the Wait conversion mode.
//  360   * @note   When the CPU clock is not fast enough to manage the data rate, a 
//  361   *         Hardware delay can be introduced between ADC conversions to reduce 
//  362   *         this data rate. 
//  363   * @note   The Hardware delay is inserted after each conversions and until the
//  364   *         previous data is read from the ADC data register
//  365   * @note   This is a way to automatically adapt the speed of the ADC to the speed 
//  366   *         of the system which will read the data.
//  367   * @note   Any hardware triggers wich occur while a conversion is on going or 
//  368   *         while the automatic Delay is applied are ignored 
//  369   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  370   * @param  NewState: new state of the ADCx Auto-Delay.
//  371   *          This parameter can be: ENABLE or DISABLE.
//  372   * @retval None
//  373   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  374 void ADC_WaitModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  375 {
//  376   /* Check the parameters */
//  377   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  378   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  379   
//  380   if (NewState != DISABLE)
ADC_WaitModeCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_WaitModeCmd_0
//  381   {
//  382     /* Enable the ADC Automatic Delayed conversion */
//  383     ADCx->CFGR1 |= ADC_CFGR1_WAIT;
        MOVS     R2,#+128
        LSLS     R2,R2,#+7        ;; #+16384
        ORRS     R2,R2,R1
        B        ??ADC_WaitModeCmd_1
//  384   }
//  385   else
//  386   {
//  387     /* Disable the ADC Automatic Delayed conversion */
//  388     ADCx->CFGR1 &= (uint32_t)~ADC_CFGR1_WAIT;
??ADC_WaitModeCmd_0:
        LDR      R2,??DataTable13_3  ;; 0xffffbfff
        ANDS     R2,R2,R1
??ADC_WaitModeCmd_1:
        STR      R2,[R0, #+12]
//  389   }
//  390 }
        BX       LR               ;; return
//  391 
//  392 /**
//  393   * @}
//  394   */
//  395 
//  396 /** @defgroup ADC_Group3 Analog Watchdog configuration functions
//  397  *  @brief   Analog Watchdog configuration functions 
//  398  *
//  399 @verbatim
//  400  ===============================================================================
//  401                    ##### Analog Watchdog configuration functions #####
//  402  ===============================================================================  
//  403     [..] This section provides functions allowing to configure the Analog Watchdog
//  404          (AWD) feature in the ADC.
//  405     [..] A typical configuration Analog Watchdog is done following these steps :
//  406          (#) the ADC guarded channel(s) is (are) selected using the 
//  407              ADC_AnalogWatchdogSingleChannelConfig() function.
//  408          (#) The Analog watchdog lower and higher threshold are configured using the  
//  409              ADC_AnalogWatchdogThresholdsConfig() function.
//  410          (#) The Analog watchdog is enabled and configured to enable the check, on one
//  411              or more channels, using the  ADC_AnalogWatchdogCmd() function.
//  412          (#) Enable the analog watchdog on the selected channel using
//  413              ADC_AnalogWatchdogSingleChannelCmd() function
//  414 
//  415 @endverbatim
//  416   * @{
//  417   */
//  418 
//  419 /**
//  420   * @brief  Enables or disables the analog watchdog 
//  421   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  422   * @param  NewState: new state of the ADCx Analog Watchdog.
//  423   *          This parameter can be: ENABLE or DISABLE.
//  424   * @retval None
//  425   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  426 void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  427 {
//  428   /* Check the parameters */
//  429   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  430   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  431   
//  432   if (NewState != DISABLE)
ADC_AnalogWatchdogCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_AnalogWatchdogCmd_0
//  433   {
//  434     /* Enable the ADC Analog Watchdog */
//  435     ADCx->CFGR1 |= ADC_CFGR1_AWDEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+16       ;; #+8388608
        ORRS     R2,R2,R1
        B        ??ADC_AnalogWatchdogCmd_1
//  436   }
//  437   else
//  438   {
//  439     /* Disable the ADC Analog Watchdog */
//  440     ADCx->CFGR1 &= (uint32_t)~ADC_CFGR1_AWDEN;
??ADC_AnalogWatchdogCmd_0:
        LDR      R2,??DataTable13_4  ;; 0xff7fffff
        ANDS     R2,R2,R1
??ADC_AnalogWatchdogCmd_1:
        STR      R2,[R0, #+12]
//  441   }
//  442 }
        BX       LR               ;; return
//  443 
//  444 /**
//  445   * @brief  Configures the high and low thresholds of the analog watchdog. 
//  446   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  447   * @param  HighThreshold: the ADC analog watchdog High threshold value.
//  448   *          This parameter must be a 12bit value.
//  449   * @param  LowThreshold: the ADC analog watchdog Low threshold value.
//  450   *          This parameter must be a 12bit value.
//  451   * @retval None
//  452   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  453 void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,
//  454                                         uint16_t LowThreshold)
//  455 {
//  456   /* Check the parameters */
//  457   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  458   assert_param(IS_ADC_THRESHOLD(HighThreshold));
//  459   assert_param(IS_ADC_THRESHOLD(LowThreshold));
//  460 
//  461   /* Set the ADCx high and low threshold */
//  462   ADCx->TR = LowThreshold | ((uint32_t)HighThreshold << 16);
ADC_AnalogWatchdogThresholdsConfig:
        LSLS     R1,R1,#+16
        ORRS     R1,R1,R2
        STR      R1,[R0, #+32]
//  463 
//  464 }
        BX       LR               ;; return
//  465 
//  466 /**
//  467   * @brief  Configures the analog watchdog guarded single channel
//  468   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  469   * @param  ADC_AnalogWatchdog_Channel: the ADC channel to configure for the analog watchdog.
//  470   *          This parameter can be one of the following values:
//  471   *            @arg ADC_AnalogWatchdog_Channel_0: ADC Channel0 selected
//  472   *            @arg ADC_AnalogWatchdog_Channel_1: ADC Channel1 selected
//  473   *            @arg ADC_AnalogWatchdog_Channel_2: ADC Channel2 selected
//  474   *            @arg ADC_AnalogWatchdog_Channel_3: ADC Channel3 selected
//  475   *            @arg ADC_AnalogWatchdog_Channel_4: ADC Channel4 selected
//  476   *            @arg ADC_AnalogWatchdog_Channel_5: ADC Channel5 selected
//  477   *            @arg ADC_AnalogWatchdog_Channel_6: ADC Channel6 selected
//  478   *            @arg ADC_AnalogWatchdog_Channel_7: ADC Channel7 selected
//  479   *            @arg ADC_AnalogWatchdog_Channel_8: ADC Channel8 selected
//  480   *            @arg ADC_AnalogWatchdog_Channel_9: ADC Channel9 selected
//  481   *            @arg ADC_AnalogWatchdog_Channel_10: ADC Channel10 selected, not available for STM32F031 devices
//  482   *            @arg ADC_AnalogWatchdog_Channel_11: ADC Channel11 selected, not available for STM32F031 devices
//  483   *            @arg ADC_AnalogWatchdog_Channel_12: ADC Channel12 selected, not available for STM32F031 devices
//  484   *            @arg ADC_AnalogWatchdog_Channel_13: ADC Channel13 selected, not available for STM32F031 devices
//  485   *            @arg ADC_AnalogWatchdog_Channel_14: ADC Channel14 selected, not available for STM32F031 devices
//  486   *            @arg ADC_AnalogWatchdog_Channel_15: ADC Channel15 selected, not available for STM32F031 devices
//  487   *            @arg ADC_AnalogWatchdog_Channel_16: ADC Channel16 selected
//  488   *            @arg ADC_AnalogWatchdog_Channel_17: ADC Channel17 selected
//  489   *            @arg ADC_AnalogWatchdog_Channel_18: ADC Channel18 selected, not available for STM32F030 devices
//  490   * @note   The channel selected on the AWDCH must be also set into the CHSELR 
//  491   *         register 
//  492   * @retval None
//  493   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  494 void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog_Channel)
//  495 {
//  496   uint32_t tmpreg = 0;
//  497 
//  498   /* Check the parameters */
//  499   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  500   assert_param(IS_ADC_ANALOG_WATCHDOG_CHANNEL(ADC_AnalogWatchdog_Channel));
//  501 
//  502   /* Get the old register value */
//  503   tmpreg = ADCx->CFGR1;
//  504 
//  505   /* Clear the Analog watchdog channel select bits */
//  506   tmpreg &= ~ADC_CFGR1_AWDCH;
//  507 
//  508   /* Set the Analog watchdog channel */
//  509   tmpreg |= ADC_AnalogWatchdog_Channel;
//  510 
//  511   /* Store the new register value */
//  512   ADCx->CFGR1 = tmpreg;
ADC_AnalogWatchdogSingleChannelConfig:
        LDR      R2,[R0, #+12]
        LDR      R3,??DataTable13_5  ;; 0x83ffffff
        ANDS     R3,R3,R2
        ORRS     R1,R1,R3
        STR      R1,[R0, #+12]
//  513 }
        BX       LR               ;; return
//  514 
//  515 /**
//  516   * @brief  Enables or disables the ADC Analog Watchdog Single Channel.
//  517   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  518   * @param  NewState: new state of the ADCx ADC Analog Watchdog Single Channel.
//  519   *          This parameter can be: ENABLE or DISABLE.
//  520   * @retval None
//  521   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  522 void ADC_AnalogWatchdogSingleChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  523 {
//  524   /* Check the parameters */
//  525   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  526   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  527 
//  528   if (NewState != DISABLE)
ADC_AnalogWatchdogSingleChannelCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_AnalogWatchdogSingleChannelCmd_0
//  529   {
//  530     /* Enable the ADC Analog Watchdog Single Channel */
//  531     ADCx->CFGR1 |= ADC_CFGR1_AWDSGL;
        MOVS     R2,#+128
        LSLS     R2,R2,#+15       ;; #+4194304
        ORRS     R2,R2,R1
        B        ??ADC_AnalogWatchdogSingleChannelCmd_1
//  532   }
//  533   else
//  534   {
//  535     /* Disable the ADC Analog Watchdog Single Channel */
//  536     ADCx->CFGR1 &= (uint32_t)~ADC_CFGR1_AWDSGL;
??ADC_AnalogWatchdogSingleChannelCmd_0:
        LDR      R2,??DataTable13_6  ;; 0xffbfffff
        ANDS     R2,R2,R1
??ADC_AnalogWatchdogSingleChannelCmd_1:
        STR      R2,[R0, #+12]
//  537   }
//  538 }
        BX       LR               ;; return
//  539 
//  540 /**
//  541   * @}
//  542   */
//  543 
//  544 /** @defgroup ADC_Group4 Temperature Sensor, Vrefint  and Vbat management functions
//  545  *  @brief   Temperature Sensor, Vrefint  and Vbat management functions
//  546  *
//  547 @verbatim
//  548  ===============================================================================
//  549  ##### Temperature Sensor, Vrefint  and Vbat management function #####
//  550  ===============================================================================
//  551     [..] This section provides a function allowing to enable/disable the internal 
//  552          connections between the ADC and the Temperature Sensor, the Vrefint and
//  553          Vbat source.
//  554      
//  555     [..] A typical configuration to get the Temperature sensor, Vrefint and Vbat channels 
//  556          voltages is done following these steps :
//  557          (#) Enable the internal connection of Temperature sensor, Vrefint or Vbat sources 
//  558              with the ADC channels using ADC_TempSensorCmd(), ADC_VrefintCmd() or ADC_VbatCmd()
//  559              functions. 
//  560          (#) select the ADC_Channel_16(Temperature sensor), ADC_Channel_17(Vrefint)
//  561              or ADC_Channel_18(Voltage battery) using ADC_ChannelConfig() function 
//  562          (#) Get the voltage values, using ADC_GetConversionValue() function
//  563 
//  564 @endverbatim
//  565   * @{
//  566   */
//  567 
//  568 /**
//  569   * @brief  Enables or disables the temperature sensor channel.
//  570   * @param  NewState: new state of the temperature sensor input channel.
//  571   *          This parameter can be: ENABLE or DISABLE.
//  572   * @retval None
//  573   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  574 void ADC_TempSensorCmd(FunctionalState NewState)
//  575 {
//  576   /* Check the parameters */
//  577   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  578 
//  579   if (NewState != DISABLE)
ADC_TempSensorCmd:
        LDR      R1,??DataTable13_7  ;; 0x40012708
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??ADC_TempSensorCmd_0
//  580   {
//  581     /* Enable the temperature sensor channel*/
//  582     ADC->CCR |= (uint32_t)ADC_CCR_TSEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+16       ;; #+8388608
        ORRS     R2,R2,R0
        B        ??ADC_TempSensorCmd_1
//  583   }
//  584   else
//  585   {
//  586     /* Disable the temperature sensor channel*/
//  587     ADC->CCR &= (uint32_t)(~ADC_CCR_TSEN);
??ADC_TempSensorCmd_0:
        LDR      R2,??DataTable13_4  ;; 0xff7fffff
        ANDS     R2,R2,R0
??ADC_TempSensorCmd_1:
        STR      R2,[R1, #+0]
//  588   }
//  589 }
        BX       LR               ;; return
//  590 
//  591 /**
//  592   * @brief  Enables or disables the Vrefint channel.
//  593   * @param  NewState: new state of the Vref input channel.
//  594   *          This parameter can be: ENABLE or DISABLE.
//  595   * @retval None
//  596   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  597 void ADC_VrefintCmd(FunctionalState NewState)
//  598 {
//  599   /* Check the parameters */
//  600   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  601 
//  602   if (NewState != DISABLE)
ADC_VrefintCmd:
        LDR      R1,??DataTable13_7  ;; 0x40012708
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??ADC_VrefintCmd_0
//  603   {
//  604     /* Enable the Vrefint channel*/
//  605     ADC->CCR |= (uint32_t)ADC_CCR_VREFEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+15       ;; #+4194304
        ORRS     R2,R2,R0
        B        ??ADC_VrefintCmd_1
//  606   }
//  607   else
//  608   {
//  609     /* Disable the Vrefint channel*/
//  610     ADC->CCR &= (uint32_t)(~ADC_CCR_VREFEN);
??ADC_VrefintCmd_0:
        LDR      R2,??DataTable13_6  ;; 0xffbfffff
        ANDS     R2,R2,R0
??ADC_VrefintCmd_1:
        STR      R2,[R1, #+0]
//  611   }
//  612 }
        BX       LR               ;; return
//  613 
//  614 /**
//  615   * @brief  Enables or disables the Vbat channel. 
//  616   * @note   This feature is not applicable for STM32F030 devices. 
//  617   * @param  NewState: new state of the Vbat input channel.
//  618   *          This parameter can be: ENABLE or DISABLE.
//  619   * @retval None
//  620   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  621 void ADC_VbatCmd(FunctionalState NewState)
//  622 {
//  623   /* Check the parameters */
//  624   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  625 
//  626   if (NewState != DISABLE)
ADC_VbatCmd:
        LDR      R1,??DataTable13_7  ;; 0x40012708
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??ADC_VbatCmd_0
//  627   {
//  628     /* Enable the Vbat channel*/
//  629     ADC->CCR |= (uint32_t)ADC_CCR_VBATEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+17       ;; #+16777216
        ORRS     R2,R2,R0
        B        ??ADC_VbatCmd_1
//  630   }
//  631   else
//  632   {
//  633     /* Disable the Vbat channel*/
//  634     ADC->CCR &= (uint32_t)(~ADC_CCR_VBATEN);
??ADC_VbatCmd_0:
        LDR      R2,??DataTable13_8  ;; 0xfeffffff
        ANDS     R2,R2,R0
??ADC_VbatCmd_1:
        STR      R2,[R1, #+0]
//  635   }
//  636 }
        BX       LR               ;; return
//  637 
//  638 /**
//  639   * @}
//  640   */
//  641 
//  642 /** @defgroup ADC_Group5 Channels Configuration functions
//  643  *  @brief    Channels Configuration functions 
//  644  *
//  645 @verbatim
//  646  ===============================================================================
//  647             ##### Channels Configuration functions #####
//  648  ===============================================================================
//  649     [..] This section provides functions allowing to manage the ADC channels,
//  650          it is composed of 3 sub sections :
//  651          (#) Configuration and management functions for ADC channels: This subsection 
//  652              provides functions allowing to configure the ADC channels :    
//  653              (++) Select the ADC channels
//  654              (++) Activate ADC Calibration
//  655              (++) Activate the Overrun Mode.
//  656              (++) Activate the Discontinuous Mode 
//  657              (++) Activate the Continuous Mode.
//  658              (++) Configure the sampling time for each channel
//  659              (++) Select the conversion Trigger and Edge for ADC channels
//  660              (++) Select the scan direction.
//  661              -@@- Please Note that the following features for ADC channels are configurated
//  662                   using the ADC_Init() function : 
//  663                   (+@@) Activate the Continuous Mode (can be also activated by ADC_OverrunModeCmd().
//  664                   (+@@) Select the conversion Trigger and Edge for ADC channels
//  665                   (+@@) Select the scan direction.
//  666          (#) Control the ADC peripheral : This subsection permits to command the ADC:
//  667              (++) Stop or discard an on-going conversion (ADSTP command)
//  668              (++) Start the ADC conversion .
//  669          (#) Get the conversion data: This subsection provides an important function in 
//  670              the ADC peripheral since it returns the converted data of the current 
//  671              ADC channel. When the Conversion value is read, the EOC Flag is 
//  672              automatically cleared.
//  673 
//  674 @endverbatim
//  675   * @{
//  676   */
//  677 
//  678 /**
//  679   * @brief  Configures for the selected ADC and its sampling time.
//  680   * @param  ADCx: where x can be 1 to select the ADC peripheral.
//  681   * @param  ADC_Channel: the ADC channel to configure. 
//  682   *          This parameter can be any combination of the following values:
//  683   *            @arg ADC_Channel_0: ADC Channel0 selected
//  684   *            @arg ADC_Channel_1: ADC Channel1 selected
//  685   *            @arg ADC_Channel_2: ADC Channel2 selected
//  686   *            @arg ADC_Channel_3: ADC Channel3 selected
//  687   *            @arg ADC_Channel_4: ADC Channel4 selected
//  688   *            @arg ADC_Channel_5: ADC Channel5 selected
//  689   *            @arg ADC_Channel_6: ADC Channel6 selected
//  690   *            @arg ADC_Channel_7: ADC Channel7 selected
//  691   *            @arg ADC_Channel_8: ADC Channel8 selected
//  692   *            @arg ADC_Channel_9: ADC Channel9 selected
//  693   *            @arg ADC_Channel_10: ADC Channel10 selected, not available for STM32F031 devices
//  694   *            @arg ADC_Channel_11: ADC Channel11 selected, not available for STM32F031 devices
//  695   *            @arg ADC_Channel_12: ADC Channel12 selected, not available for STM32F031 devices
//  696   *            @arg ADC_Channel_13: ADC Channel13 selected, not available for STM32F031 devices
//  697   *            @arg ADC_Channel_14: ADC Channel14 selected, not available for STM32F031 devices
//  698   *            @arg ADC_Channel_15: ADC Channel15 selected, not available for STM32F031 devices
//  699   *            @arg ADC_Channel_16: ADC Channel16 selected
//  700   *            @arg ADC_Channel_17: ADC Channel17 selected
//  701   *            @arg ADC_Channel_18: ADC Channel18 selected, not available for STM32F030 devices
//  702   * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
//  703   *          This parameter can be one of the following values:
//  704   *            @arg ADC_SampleTime_1_5Cycles: Sample time equal to 1.5 cycles  
//  705   *            @arg ADC_SampleTime_7_5Cycles: Sample time equal to 7.5 cycles
//  706   *            @arg ADC_SampleTime_13_5Cycles: Sample time equal to 13.5 cycles
//  707   *            @arg ADC_SampleTime_28_5Cycles: Sample time equal to 28.5 cycles
//  708   *            @arg ADC_SampleTime_41_5Cycles: Sample time equal to 41.5 cycles
//  709   *            @arg ADC_SampleTime_55_5Cycles: Sample time equal to 55.5 cycles
//  710   *            @arg ADC_SampleTime_71_5Cycles: Sample time equal to 71.5 cycles
//  711   *            @arg ADC_SampleTime_239_5Cycles: Sample time equal to 239.5 cycles
//  712   * @retval None
//  713   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  714 void ADC_ChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_Channel, uint32_t ADC_SampleTime)
//  715 {
//  716   uint32_t tmpreg = 0;
//  717 
//  718   /* Check the parameters */
//  719   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  720   assert_param(IS_ADC_CHANNEL(ADC_Channel));
//  721   assert_param(IS_ADC_SAMPLE_TIME(ADC_SampleTime));
//  722 
//  723   /* Configure the ADC Channel */
//  724   ADCx->CHSELR |= (uint32_t)ADC_Channel;
ADC_ChannelConfig:
        LDR      R3,[R0, #+40]
        ORRS     R1,R1,R3
        STR      R1,[R0, #+40]
//  725 
//  726   /* Clear the Sampling time Selection bits */
//  727   tmpreg &= ~ADC_SMPR1_SMPR;
//  728 
//  729   /* Set the ADC Sampling Time register */
//  730   tmpreg |= (uint32_t)ADC_SampleTime;
//  731 
//  732   /* Configure the ADC Sample time register */
//  733   ADCx->SMPR = tmpreg ;
        STR      R2,[R0, #+20]
//  734 }
        BX       LR               ;; return
//  735 
//  736 /**
//  737   * @brief  Enable the Continuous mode for the selected ADCx channels.
//  738   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  739   * @param  NewState: new state of the Continuous mode.
//  740   *          This parameter can be: ENABLE or DISABLE.
//  741   * @note   It is not possible to have both discontinuous mode and continuous mode
//  742   *         enabled. In this case (If DISCEN and CONT are Set), the ADC behaves 
//  743   *         as if continuous mode was disabled
//  744   * @retval None
//  745   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  746 void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  747 {
//  748   /* Check the parameters */
//  749   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  750   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  751 
//  752     if (NewState != DISABLE)
ADC_ContinuousModeCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_ContinuousModeCmd_0
//  753   {
//  754     /* Enable the Continuous mode*/
//  755     ADCx->CFGR1 |= (uint32_t)ADC_CFGR1_CONT;
        MOVS     R2,#+128
        LSLS     R2,R2,#+6        ;; #+8192
        ORRS     R2,R2,R1
        B        ??ADC_ContinuousModeCmd_1
//  756   }
//  757   else
//  758   {
//  759     /* Disable the Continuous mode */
//  760     ADCx->CFGR1 &= (uint32_t)(~ADC_CFGR1_CONT);
??ADC_ContinuousModeCmd_0:
        LDR      R2,??DataTable13_9  ;; 0xffffdfff
        ANDS     R2,R2,R1
??ADC_ContinuousModeCmd_1:
        STR      R2,[R0, #+12]
//  761   }
//  762 }
        BX       LR               ;; return
//  763 
//  764 /**
//  765   * @brief  Enable the discontinuous mode for the selected ADC channels.
//  766   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  767   * @param  NewState: new state of the discontinuous mode.
//  768   *          This parameter can be: ENABLE or DISABLE.
//  769   * @note   It is not possible to have both discontinuous mode and continuous mode
//  770   *         enabled. In this case (If DISCEN and CONT are Set), the ADC behaves 
//  771   *         as if continuous mode was disabled
//  772   * @retval None
//  773   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  774 void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  775 {
//  776   /* Check the parameters */
//  777   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  778   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  779 
//  780     if (NewState != DISABLE)
ADC_DiscModeCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_DiscModeCmd_0
//  781   {
//  782     /* Enable the Discontinuous mode */
//  783     ADCx->CFGR1 |= (uint32_t)ADC_CFGR1_DISCEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+9        ;; #+65536
        ORRS     R2,R2,R1
        B        ??ADC_DiscModeCmd_1
//  784   }
//  785   else
//  786   {
//  787     /* Disable the Discontinuous mode */
//  788     ADCx->CFGR1 &= (uint32_t)(~ADC_CFGR1_DISCEN);
??ADC_DiscModeCmd_0:
        LDR      R2,??DataTable13_10  ;; 0xfffeffff
        ANDS     R2,R2,R1
??ADC_DiscModeCmd_1:
        STR      R2,[R0, #+12]
//  789   }
//  790 }
        BX       LR               ;; return
//  791 
//  792 /**
//  793   * @brief  Enable the Overrun mode for the selected ADC channels.
//  794   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  795   * @param  NewState: new state of the Overrun mode.
//  796   *          This parameter can be: ENABLE or DISABLE.
//  797   * @retval None
//  798   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  799 void ADC_OverrunModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  800 {
//  801   /* Check the parameters */
//  802   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  803   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  804 
//  805     if (NewState != DISABLE)
ADC_OverrunModeCmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_OverrunModeCmd_0
//  806   {
//  807     /* Enable the Overrun mode */
//  808     ADCx->CFGR1 |= (uint32_t)ADC_CFGR1_OVRMOD;
        MOVS     R2,#+128
        LSLS     R2,R2,#+5        ;; #+4096
        ORRS     R2,R2,R1
        B        ??ADC_OverrunModeCmd_1
//  809   }
//  810   else
//  811   {
//  812     /* Disable the Overrun mode */
//  813     ADCx->CFGR1 &= (uint32_t)(~ADC_CFGR1_OVRMOD);
??ADC_OverrunModeCmd_0:
        LDR      R2,??DataTable13_11  ;; 0xffffefff
        ANDS     R2,R2,R1
??ADC_OverrunModeCmd_1:
        STR      R2,[R0, #+12]
//  814   }
//  815 }
        BX       LR               ;; return
//  816 
//  817 /**
//  818   * @brief  Active the Calibration operation for the selected ADC.
//  819   * @note   The Calibration can be initiated only when ADC is still in the 
//  820   *         reset configuration (ADEN must be equal to 0).
//  821   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  822   * @retval ADC Calibration factor 
//  823   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  824 uint32_t ADC_GetCalibrationFactor(ADC_TypeDef* ADCx)
//  825 {
ADC_GetCalibrationFactor:
        PUSH     {R3-R5}
        MOVS     R1,R0
//  826   uint32_t tmpreg = 0, calibrationcounter = 0, calibrationstatus = 0;
        MOVS     R0,#+0
        MOVS     R2,#+0
//  827 
//  828   /* Check the parameters */
//  829   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  830   
//  831   /* Set the ADC calibartion */
//  832   ADCx->CR |= (uint32_t)ADC_CR_ADCAL;
        MOVS     R3,#+128
        LSLS     R3,R3,#+24       ;; #-2147483648
        LDR      R4,[R1, #+8]
        ORRS     R4,R4,R3
        STR      R4,[R1, #+8]
        MOVS     R4,#+240
        LSLS     R4,R4,#+8        ;; #+61440
//  833   
//  834   /* Wait until no ADC calibration is completed */
//  835   do
//  836   {
//  837     calibrationstatus = ADCx->CR & ADC_CR_ADCAL;
??ADC_GetCalibrationFactor_0:
        LDR      R5,[R1, #+8]
//  838     calibrationcounter++;  
        ADDS     R2,R2,#+1
//  839   } while((calibrationcounter != CALIBRATION_TIMEOUT) && (calibrationstatus != 0x00));
        CMP      R2,R4
        BEQ      ??ADC_GetCalibrationFactor_1
        ANDS     R5,R5,R3
        BNE      ??ADC_GetCalibrationFactor_0
//  840     
//  841   if((uint32_t)(ADCx->CR & ADC_CR_ADCAL) == RESET)
??ADC_GetCalibrationFactor_1:
        LDR      R2,[R1, #+8]
        ANDS     R3,R3,R2
        BNE      ??ADC_GetCalibrationFactor_2
//  842   {
//  843     /*Get the calibration factor from the ADC data register */
//  844     tmpreg = ADCx->DR;
        LDR      R0,[R1, #+64]
//  845   }
//  846   else
//  847   {
//  848     /* Error factor */
//  849     tmpreg = 0x00000000;
//  850   }
//  851   return tmpreg;
??ADC_GetCalibrationFactor_2:
        POP      {R1,R4,R5}
        BX       LR               ;; return
//  852 }
//  853 
//  854 /**
//  855   * @brief  Stop the on going conversions for the selected ADC.
//  856   * @note   When ADSTP is set, any on going conversion is aborted, and the ADC 
//  857   *         data register is not updated with current conversion. 
//  858   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  859   * @retval None
//  860   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  861 void ADC_StopOfConversion(ADC_TypeDef* ADCx)
//  862 {
//  863   /* Check the parameters */
//  864   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  865   
//  866   ADCx->CR |= (uint32_t)ADC_CR_ADSTP;
ADC_StopOfConversion:
        LDR      R1,[R0, #+8]
        MOVS     R2,#+16
        ORRS     R2,R2,R1
        STR      R2,[R0, #+8]
//  867 }
        BX       LR               ;; return
//  868 
//  869 /**
//  870   * @brief  Start Conversion for the selected ADC channels.
//  871   * @note   In continuous mode, ADSTART is not cleared by hardware with the 
//  872   *         assertion of EOSEQ because the sequence is automatic relaunched
//  873   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  874   * @retval None
//  875   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  876 void ADC_StartOfConversion(ADC_TypeDef* ADCx)
//  877 {
//  878   /* Check the parameters */
//  879   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  880   
//  881   ADCx->CR |= (uint32_t)ADC_CR_ADSTART;
ADC_StartOfConversion:
        LDR      R1,[R0, #+8]
        MOVS     R2,#+4
        ORRS     R2,R2,R1
        STR      R2,[R0, #+8]
//  882 }
        BX       LR               ;; return
//  883 
//  884 /**
//  885   * @brief  Returns the last ADCx conversion result data for ADC channel.  
//  886   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  887   * @retval The Data conversion value.
//  888   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  889 uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
//  890 {
//  891   /* Check the parameters */
//  892   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  893 
//  894   /* Return the selected ADC conversion value */
//  895   return (uint16_t) ADCx->DR;
ADC_GetConversionValue:
        LDR      R0,[R0, #+64]
        UXTH     R0,R0
        BX       LR               ;; return
//  896 }
//  897 
//  898 /**
//  899   * @}
//  900   */
//  901 
//  902 /** @defgroup ADC_Group6 DMA Configuration functions
//  903  *  @brief   Regular Channels DMA Configuration functions 
//  904  *
//  905 @verbatim
//  906  ===============================================================================
//  907           ##### DMA Configuration functions #####
//  908  ===============================================================================
//  909     [..] This section provides functions allowing to configure the DMA for ADC hannels.
//  910          Since converted channel values are stored into a unique data register, 
//  911          it is useful to use DMA for conversion of more than one channel. This 
//  912          avoids the loss of the data already stored in the ADC Data register. 
//  913          When the DMA mode is enabled (using the ADC_DMACmd() function), after each
//  914          conversion of a channel, a DMA request is generated.
//  915   
//  916     [..] Depending on the "DMA disable selection" configuration (using the 
//  917          ADC_DMARequestModeConfig() function), at the end of the last DMA 
//  918          transfer, two possibilities are allowed:
//  919          (+) No new DMA request is issued to the DMA controller (One Shot Mode) 
//  920          (+) Requests can continue to be generated (Circular Mode).
//  921 
//  922 @endverbatim
//  923   * @{
//  924   */
//  925 
//  926 /**
//  927   * @brief  Enables or disables the specified ADC DMA request.
//  928   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  929   * @param  NewState: new state of the selected ADC DMA transfer.
//  930   *          This parameter can be: ENABLE or DISABLE.
//  931   * @retval None
//  932   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  933 void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
//  934 {
//  935   /* Check the parameters */
//  936   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  937   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  938 
//  939   if (NewState != DISABLE)
ADC_DMACmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+12]
        BEQ      ??ADC_DMACmd_0
//  940   {
//  941     /* Enable the selected ADC DMA request */
//  942     ADCx->CFGR1 |= (uint32_t)ADC_CFGR1_DMAEN;
        MOVS     R2,#+1
        ORRS     R2,R2,R1
        STR      R2,[R0, #+12]
        BX       LR
//  943   }
//  944   else
//  945   {
//  946     /* Disable the selected ADC DMA request */
//  947     ADCx->CFGR1 &= (uint32_t)(~ADC_CFGR1_DMAEN);
??ADC_DMACmd_0:
        MOVS     R2,#+1
        BICS     R1,R1,R2
        STR      R1,[R0, #+12]
//  948   }
//  949 }
        BX       LR               ;; return
//  950 
//  951 /**
//  952   * @brief  Enables or disables the ADC DMA request after last transfer (Single-ADC mode)
//  953   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
//  954   * @param  ADC_DMARequestMode: the ADC channel to configure. 
//  955   *          This parameter can be one of the following values:
//  956   *            @arg ADC_DMAMode_OneShot: DMA One Shot Mode 
//  957   *            @arg ADC_DMAMode_Circular: DMA Circular Mode  
//  958   *  @retval None
//  959   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  960 void ADC_DMARequestModeConfig(ADC_TypeDef* ADCx, uint32_t ADC_DMARequestMode)
//  961 {
//  962   /* Check the parameters */
//  963   assert_param(IS_ADC_ALL_PERIPH(ADCx));
//  964 
//  965   ADCx->CFGR1 &= (uint32_t)~ADC_CFGR1_DMACFG;
ADC_DMARequestModeConfig:
        LDR      R2,[R0, #+12]
        MOVS     R3,#+2
        BICS     R2,R2,R3
        STR      R2,[R0, #+12]
//  966   ADCx->CFGR1 |= (uint32_t)ADC_DMARequestMode;
        LDR      R2,[R0, #+12]
        ORRS     R1,R1,R2
        STR      R1,[R0, #+12]
//  967 }
        BX       LR               ;; return
//  968 
//  969 /**
//  970   * @}
//  971   */
//  972 
//  973 /** @defgroup ADC_Group7 Interrupts and flags management functions
//  974  *  @brief   Interrupts and flags management functions.
//  975  *
//  976 @verbatim   
//  977  ===============================================================================
//  978             ##### Interrupts and flags management functions #####
//  979  ===============================================================================
//  980     [..] This section provides functions allowing to configure the ADC Interrupts 
//  981          and get the status and clear flags and Interrupts pending bits.
//  982   
//  983     [..] The ADC provide 6 Interrupts sources and 11 Flags which can be divided into 
//  984          3 groups:
//  985 
//  986   *** Flags for ADC status ***
//  987   ======================================================
//  988     [..]
//  989         (+)Flags :
//  990            (##) ADC_FLAG_ADRDY : This flag is set after the ADC has been enabled (bit ADEN=1)
//  991                and when the ADC reaches a state where it is ready to accept conversion requests
//  992            (##) ADC_FLAG_ADEN : This flag is set by software to enable the ADC.
//  993                 The ADC will be effectively ready to operate once the ADRDY flag has been set.
//  994            (##) ADC_FLAG_ADDIS : This flag is cleared once the ADC is effectively
//  995                 disabled.
//  996            (##) ADC_FLAG_ADSTART : This flag is cleared after the execution of
//  997                 ADC_StopOfConversion() function, at the same time as the ADSTP bit is
//  998                 cleared by hardware
//  999            (##) ADC_FLAG_ADSTP : This flag is cleared by hardware when the conversion
// 1000                 is effectively discarded and the ADC is ready to accept a new start conversion
// 1001            (##) ADC_FLAG_ADCAL : This flag is set once the calibration is complete.
// 1002 
// 1003         (+)Interrupts 
// 1004            (##) ADC_IT_ADRDY : specifies the interrupt source for ADC ready event.
// 1005 
// 1006   *** Flags and Interrupts for ADC channel conversion ***
// 1007   =====================================================
// 1008     [..]
// 1009         (+)Flags :
// 1010            (##) ADC_FLAG_EOC : This flag is set by hardware at the end of each conversion
// 1011                 of a channel when a new data result is available in the data register
// 1012            (##) ADC_FLAG_EOSEQ : This bit is set by hardware at the end of the conversion
// 1013                 of a sequence of channels selected by ADC_ChannelConfig() function.
// 1014            (##) ADC_FLAG_EOSMP : This bit is set by hardware at the end of the sampling phase.
// 1015            (##) ADC_FLAG_OVR : This flag is set by hardware when an overrun occurs,
// 1016                 meaning that a new conversion has complete while the EOC flag was already set.
// 1017 
// 1018         (+)Interrupts :
// 1019            (##) ADC_IT_EOC : specifies the interrupt source for end of conversion event.
// 1020            (##) ADC_IT_EOSEQ : specifies the interrupt source for end of sequence event.
// 1021            (##) ADC_IT_EOSMP : specifies the interrupt source for end of sampling event.
// 1022            (##) ADC_IT_OVR : specifies the interrupt source for Overrun detection 
// 1023                 event.
// 1024 
// 1025   *** Flags and Interrupts for the Analog Watchdog ***
// 1026   ================================================
// 1027     [..]
// 1028         (+)Flags :
// 1029            (##) ADC_FLAG_AWD: This flag is set by hardware when the converted
// 1030                 voltage crosses the values programmed thrsholds
// 1031 
// 1032         (+)Interrupts :
// 1033            (##) ADC_IT_AWD : specifies the interrupt source for Analog watchdog 
// 1034                 event.
// 1035   
// 1036     [..] The user should identify which mode will be used in his application to 
// 1037          manage the ADC controller events: Polling mode or Interrupt mode.
// 1038   
// 1039     [..] In the Polling Mode it is advised to use the following functions:
// 1040          (+) ADC_GetFlagStatus() : to check if flags events occur.
// 1041          (+) ADC_ClearFlag()     : to clear the flags events.
// 1042   
// 1043     [..] In the Interrupt Mode it is advised to use the following functions:
// 1044          (+) ADC_ITConfig()       : to enable or disable the interrupt source.
// 1045          (+) ADC_GetITStatus()    : to check if Interrupt occurs.
// 1046          (+) ADC_ClearITPendingBit() : to clear the Interrupt pending Bit 
// 1047              (corresponding Flag).
// 1048 
// 1049 @endverbatim
// 1050   * @{
// 1051   */
// 1052 
// 1053 /**
// 1054   * @brief  Enables or disables the specified ADC interrupts.
// 1055   * @param  ADCx: where x can be 1 to select the ADC peripheral.
// 1056   * @param  ADC_IT: specifies the ADC interrupt sources to be enabled or disabled.
// 1057   *          This parameter can be one of the following values:
// 1058   *            @arg ADC_IT_ADRDY: ADC ready interrupt 
// 1059   *            @arg ADC_IT_EOSMP: End of sampling interrupt
// 1060   *            @arg ADC_IT_EOC: End of conversion interrupt 
// 1061   *            @arg ADC_IT_EOSEQ: End of sequence of conversion interrupt
// 1062   *            @arg ADC_IT_OVR: overrun interrupt
// 1063   *            @arg ADC_IT_AWD: Analog watchdog interrupt
// 1064   * @param  NewState: new state of the specified ADC interrupts.
// 1065   *          This parameter can be: ENABLE or DISABLE.
// 1066   * @retval None
// 1067   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1068 void ADC_ITConfig(ADC_TypeDef* ADCx, uint32_t ADC_IT, FunctionalState NewState)
// 1069 {
// 1070   /* Check the parameters */
// 1071   assert_param(IS_ADC_ALL_PERIPH(ADCx));
// 1072   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1073   assert_param(IS_ADC_CONFIG_IT(ADC_IT)); 
// 1074 
// 1075   if (NewState != DISABLE)
ADC_ITConfig:
        CMP      R2,#+0
        LDR      R2,[R0, #+4]
        BEQ      ??ADC_ITConfig_0
// 1076   {
// 1077     /* Enable the selected ADC interrupts */
// 1078     ADCx->IER |= ADC_IT;
        ORRS     R1,R1,R2
        STR      R1,[R0, #+4]
        BX       LR
// 1079   }
// 1080   else
// 1081   {
// 1082     /* Disable the selected ADC interrupts */
// 1083     ADCx->IER &= (~(uint32_t)ADC_IT);
??ADC_ITConfig_0:
        BICS     R2,R2,R1
        STR      R2,[R0, #+4]
// 1084   }
// 1085 }
        BX       LR               ;; return
// 1086 
// 1087 /**
// 1088   * @brief  Checks whether the specified ADC flag is set or not.
// 1089   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
// 1090   * @param  ADC_FLAG: specifies the flag to check. 
// 1091   *          This parameter can be one of the following values:
// 1092   *            @arg ADC_FLAG_AWD: Analog watchdog flag
// 1093   *            @arg ADC_FLAG_OVR: Overrun flag 
// 1094   *            @arg ADC_FLAG_EOSEQ: End of Sequence flag
// 1095   *            @arg ADC_FLAG_EOC: End of conversion flag
// 1096   *            @arg ADC_FLAG_EOSMP: End of sampling flag
// 1097   *            @arg ADC_FLAG_ADRDY: ADC Ready flag
// 1098   *            @arg ADC_FLAG_ADEN: ADC enable flag 
// 1099   *            @arg ADC_FLAG_ADDIS: ADC disable flag 
// 1100   *            @arg ADC_FLAG_ADSTART: ADC start flag 
// 1101   *            @arg ADC_FLAG_ADSTP: ADC stop flag
// 1102   *            @arg ADC_FLAG_ADCAL: ADC Calibration flag
// 1103   * @retval The new state of ADC_FLAG (SET or RESET).
// 1104   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1105 FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint32_t ADC_FLAG)
// 1106 {
ADC_GetFlagStatus:
        MOVS     R2,R0
// 1107   FlagStatus bitstatus = RESET;
        MOVS     R0,#+0
// 1108   uint32_t tmpreg = 0;
// 1109 
// 1110   /* Check the parameters */
// 1111   assert_param(IS_ADC_ALL_PERIPH(ADCx));
// 1112   assert_param(IS_ADC_GET_FLAG(ADC_FLAG));
// 1113 
// 1114   if((uint32_t)(ADC_FLAG & 0x01000000))
        LSLS     R3,R1,#+7
        BPL      ??ADC_GetFlagStatus_0
// 1115   {
// 1116     tmpreg = ADCx->CR & 0xFEFFFFFF;
        LDR      R3,[R2, #+8]
        LDR      R2,??DataTable13_8  ;; 0xfeffffff
        ANDS     R2,R2,R3
        B        ??ADC_GetFlagStatus_1
// 1117   }
// 1118   else
// 1119   {
// 1120     tmpreg = ADCx->ISR;
??ADC_GetFlagStatus_0:
        LDR      R2,[R2, #+0]
// 1121   }
// 1122   
// 1123   /* Check the status of the specified ADC flag */
// 1124   if ((tmpreg & ADC_FLAG) != (uint32_t)RESET)
??ADC_GetFlagStatus_1:
        ANDS     R1,R1,R2
        BEQ      ??ADC_GetFlagStatus_2
// 1125   {
// 1126     /* ADC_FLAG is set */
// 1127     bitstatus = SET;
        MOVS     R0,#+1
// 1128   }
// 1129   else
// 1130   {
// 1131     /* ADC_FLAG is reset */
// 1132     bitstatus = RESET;
// 1133   }
// 1134   /* Return the ADC_FLAG status */
// 1135   return  bitstatus;
??ADC_GetFlagStatus_2:
        BX       LR               ;; return
// 1136 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13:
        DC32     0x40012400

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_1:
        DC32     0xffffd203

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_2:
        DC32     0xffff7fff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_3:
        DC32     0xffffbfff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_4:
        DC32     0xff7fffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_5:
        DC32     0x83ffffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_6:
        DC32     0xffbfffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_7:
        DC32     0x40012708

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_8:
        DC32     0xfeffffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_9:
        DC32     0xffffdfff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_10:
        DC32     0xfffeffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable13_11:
        DC32     0xffffefff
// 1137 
// 1138 /**
// 1139   * @brief  Clears the ADCx's pending flags.
// 1140   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
// 1141   * @param  ADC_FLAG: specifies the flag to clear. 
// 1142   *          This parameter can be any combination of the following values:
// 1143   *            @arg ADC_FLAG_AWD: Analog watchdog flag
// 1144   *            @arg ADC_FLAG_EOC: End of conversion flag
// 1145   *            @arg ADC_FLAG_ADRDY: ADC Ready flag
// 1146   *            @arg ADC_FLAG_EOSMP: End of sampling flag
// 1147   *            @arg ADC_FLAG_EOSEQ: End of Sequence flag
// 1148   *            @arg ADC_FLAG_OVR: Overrun flag 
// 1149   * @retval None
// 1150   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1151 void ADC_ClearFlag(ADC_TypeDef* ADCx, uint32_t ADC_FLAG)
// 1152 {
// 1153   /* Check the parameters */
// 1154   assert_param(IS_ADC_ALL_PERIPH(ADCx));
// 1155   assert_param(IS_ADC_CLEAR_FLAG(ADC_FLAG));
// 1156 
// 1157   /* Clear the selected ADC flags */
// 1158   ADCx->ISR = (uint32_t)ADC_FLAG;
ADC_ClearFlag:
        STR      R1,[R0, #+0]
// 1159 }
        BX       LR               ;; return
// 1160 
// 1161 /**
// 1162   * @brief  Checks whether the specified ADC interrupt has occurred or not.
// 1163   * @param  ADCx: where x can be 1 to select the ADC1 peripheral
// 1164   * @param  ADC_IT: specifies the ADC interrupt source to check.
// 1165   *          This parameter can be one of the following values:
// 1166   *            @arg ADC_IT_ADRDY: ADC ready interrupt 
// 1167   *            @arg ADC_IT_EOSMP: End of sampling interrupt
// 1168   *            @arg ADC_IT_EOC: End of conversion interrupt 
// 1169   *            @arg ADC_IT_EOSEQ: End of sequence of conversion interrupt
// 1170   *            @arg ADC_IT_OVR: overrun interrupt
// 1171   *            @arg ADC_IT_AWD: Analog watchdog interrupt
// 1172   * @retval The new state of ADC_IT (SET or RESET).
// 1173   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1174 ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint32_t ADC_IT)
// 1175 {
ADC_GetITStatus:
        MOVS     R2,R0
// 1176   ITStatus bitstatus = RESET;
        MOVS     R0,#+0
// 1177   uint32_t enablestatus = 0;
// 1178 
// 1179   /* Check the parameters */
// 1180   assert_param(IS_ADC_ALL_PERIPH(ADCx));
// 1181   assert_param(IS_ADC_GET_IT(ADC_IT));
// 1182 
// 1183   /* Get the ADC_IT enable bit status */
// 1184   enablestatus = (uint32_t)(ADCx->IER & ADC_IT); 
        LDR      R3,[R2, #+4]
        ANDS     R3,R3,R1
// 1185 
// 1186   /* Check the status of the specified ADC interrupt */
// 1187   if (((uint32_t)(ADCx->ISR & ADC_IT) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
        LDR      R2,[R2, #+0]
        ANDS     R1,R1,R2
        BEQ      ??ADC_GetITStatus_0
        CMP      R3,#+0
        BEQ      ??ADC_GetITStatus_0
// 1188   {
// 1189     /* ADC_IT is set */
// 1190     bitstatus = SET;
        MOVS     R0,#+1
// 1191   }
// 1192   else
// 1193   {
// 1194     /* ADC_IT is reset */
// 1195     bitstatus = RESET;
// 1196   }
// 1197   /* Return the ADC_IT status */
// 1198   return  bitstatus;
??ADC_GetITStatus_0:
        BX       LR               ;; return
// 1199 }
// 1200 
// 1201 /**
// 1202   * @brief  Clears the ADCx's interrupt pending bits.
// 1203   * @param  ADCx: where x can be 1 to select the ADC1 peripheral.
// 1204   * @param  ADC_IT: specifies the ADC interrupt pending bit to clear.
// 1205   *          This parameter can be one of the following values:
// 1206   *            @arg ADC_IT_ADRDY: ADC ready interrupt
// 1207   *            @arg ADC_IT_EOSMP: End of sampling interrupt
// 1208   *            @arg ADC_IT_EOC: End of conversion interrupt
// 1209   *            @arg ADC_IT_EOSEQ: End of sequence of conversion interrupt
// 1210   *            @arg ADC_IT_OVR: overrun interrupt
// 1211   *            @arg ADC_IT_AWD: Analog watchdog interrupt
// 1212   * @retval None
// 1213   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1214 void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint32_t ADC_IT)
// 1215 {
// 1216   /* Check the parameters */
// 1217   assert_param(IS_ADC_ALL_PERIPH(ADCx));
// 1218   assert_param(IS_ADC_CLEAR_IT(ADC_IT));
// 1219 
// 1220   /* Clear the selected ADC interrupt pending bits */
// 1221   ADCx->ISR = (uint32_t)ADC_IT; 
ADC_ClearITPendingBit:
        STR      R1,[R0, #+0]
// 1222 }
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
// 1223 
// 1224 /**
// 1225   * @}
// 1226   */
// 1227 
// 1228 /**
// 1229   * @}
// 1230   */ 
// 1231 
// 1232 /**
// 1233   * @}
// 1234   */ 
// 1235 
// 1236 /**
// 1237   * @}
// 1238   */ 
// 1239 
// 1240 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
// 610 bytes in section .text
// 
// 610 bytes of CODE memory
//
//Errors: none
//Warnings: none
