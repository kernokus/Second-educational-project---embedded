///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.50.3.4676/W32 for ARM     22/Oct/2019  21:10:52 /
// Copyright 1999-2013 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    rcc.c                                                   /
//    Command line =  F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1. /
//                    5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_ /
//                    rcc.c -D NDEBUG -D STM32F051 -D USE_STDPERIPH_DRIVER    /
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
//    List file    =  F:\ProjectsE\MCU\FiltTest_2\Release\List\stm32f0xx_rcc. /
//                    s                                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME stm32f0xx_rcc

        #define SHT_PROGBITS 0x1

        EXTERN __aeabi_uidiv

        PUBLIC RCC_ADCCLKConfig
        PUBLIC RCC_AHBPeriphClockCmd
        PUBLIC RCC_AHBPeriphResetCmd
        PUBLIC RCC_APB1PeriphClockCmd
        PUBLIC RCC_APB1PeriphResetCmd
        PUBLIC RCC_APB2PeriphClockCmd
        PUBLIC RCC_APB2PeriphResetCmd
        PUBLIC RCC_AdjustHSI14CalibrationValue
        PUBLIC RCC_AdjustHSICalibrationValue
        PUBLIC RCC_BackupResetCmd
        PUBLIC RCC_CECCLKConfig
        PUBLIC RCC_ClearFlag
        PUBLIC RCC_ClearITPendingBit
        PUBLIC RCC_ClockSecuritySystemCmd
        PUBLIC RCC_DeInit
        PUBLIC RCC_GetClocksFreq
        PUBLIC RCC_GetFlagStatus
        PUBLIC RCC_GetITStatus
        PUBLIC RCC_GetSYSCLKSource
        PUBLIC RCC_HCLKConfig
        PUBLIC RCC_HSEConfig
        PUBLIC RCC_HSI14ADCRequestCmd
        PUBLIC RCC_HSI14Cmd
        PUBLIC RCC_HSI48Cmd
        PUBLIC RCC_HSICmd
        PUBLIC RCC_I2CCLKConfig
        PUBLIC RCC_ITConfig
        PUBLIC RCC_LSEConfig
        PUBLIC RCC_LSEDriveConfig
        PUBLIC RCC_LSICmd
        PUBLIC RCC_MCOConfig
        PUBLIC RCC_PCLKConfig
        PUBLIC RCC_PLLCmd
        PUBLIC RCC_PLLConfig
        PUBLIC RCC_PREDIV1Config
        PUBLIC RCC_RTCCLKCmd
        PUBLIC RCC_RTCCLKConfig
        PUBLIC RCC_SYSCLKConfig
        PUBLIC RCC_USARTCLKConfig
        PUBLIC RCC_USBCLKConfig
        PUBLIC RCC_WaitForHSEStartUp

// F:\ProjectsE\MCU\FiltTest_2\STM32F0xx_StdPeriph_Lib_V1.5.0\Libraries\STM32F0xx_StdPeriph_Driver\src\stm32f0xx_rcc.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    stm32f0xx_rcc.c
//    4   * @author  MCD Application Team
//    5   * @version V1.5.0
//    6   * @date    05-December-2014
//    7   * @brief   This file provides firmware functions to manage the following 
//    8   *          functionalities of the Reset and clock control (RCC) peripheral:
//    9   *           + Internal/external clocks, PLL, CSS and MCO configuration
//   10   *           + System, AHB and APB busses clocks configuration
//   11   *           + Peripheral clocks configuration
//   12   *           + Interrupts and flags management
//   13   *
//   14  @verbatim
//   15 
//   16  ===============================================================================
//   17                         ##### RCC specific features #####
//   18  ===============================================================================
//   19     [..] After reset the device is running from HSI (8 MHz) with Flash 0 WS, 
//   20          all peripherals are off except internal SRAM, Flash and SWD.
//   21          (#) There is no prescaler on High speed (AHB) and Low speed (APB) busses;
//   22              all peripherals mapped on these busses are running at HSI speed.
//   23          (#) The clock for all peripherals is switched off, except the SRAM and FLASH.
//   24          (#) All GPIOs are in input floating state, except the SWD pins which
//   25              are assigned to be used for debug purpose.
//   26     [..] Once the device started from reset, the user application has to:
//   27          (#) Configure the clock source to be used to drive the System clock
//   28              (if the application needs higher frequency/performance)
//   29          (#) Configure the System clock frequency and Flash settings
//   30          (#) Configure the AHB and APB busses prescalers
//   31          (#) Enable the clock for the peripheral(s) to be used
//   32          (#) Configure the clock source(s) for peripherals which clocks are not
//   33              derived from the System clock (ADC, CEC, I2C, USART, RTC and IWDG)
//   34 
//   35  @endverbatim
//   36   
//   37   ******************************************************************************
//   38   * @attention
//   39   *
//   40   * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
//   41   *
//   42   * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//   43   * You may not use this file except in compliance with the License.
//   44   * You may obtain a copy of the License at:
//   45   *
//   46   *        http://www.st.com/software_license_agreement_liberty_v2
//   47   *
//   48   * Unless required by applicable law or agreed to in writing, software 
//   49   * distributed under the License is distributed on an "AS IS" BASIS, 
//   50   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   51   * See the License for the specific language governing permissions and
//   52   * limitations under the License.
//   53   *
//   54   ******************************************************************************
//   55   */
//   56 
//   57 /* Includes ------------------------------------------------------------------*/
//   58 #include "stm32f0xx_rcc.h"
//   59 
//   60 /** @addtogroup STM32F0xx_StdPeriph_Driver
//   61   * @{
//   62   */
//   63 
//   64 /** @defgroup RCC 
//   65   * @brief RCC driver modules
//   66   * @{
//   67   */ 
//   68 
//   69 /* Private typedef -----------------------------------------------------------*/
//   70 /* Private define ------------------------------------------------------------*/
//   71 
//   72 /* ---------------------- RCC registers mask -------------------------------- */
//   73 /* RCC Flag Mask */
//   74 #define FLAG_MASK                 ((uint8_t)0x1F)
//   75 
//   76 /* CR register byte 2 (Bits[23:16]) base address */
//   77 #define CR_BYTE2_ADDRESS          ((uint32_t)0x40021002)
//   78 
//   79 /* CFGR register byte 3 (Bits[31:23]) base address */
//   80 #define CFGR_BYTE3_ADDRESS        ((uint32_t)0x40021007)
//   81 
//   82 /* CIR register byte 1 (Bits[15:8]) base address */
//   83 #define CIR_BYTE1_ADDRESS         ((uint32_t)0x40021009)
//   84 
//   85 /* CIR register byte 2 (Bits[23:16]) base address */
//   86 #define CIR_BYTE2_ADDRESS         ((uint32_t)0x4002100A)
//   87 
//   88 /* Private macro -------------------------------------------------------------*/
//   89 /* Private variables ---------------------------------------------------------*/

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   90 static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
APBAHBPrescTable:
        DATA
        DC8 0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9
//   91 
//   92 /* Private function prototypes -----------------------------------------------*/
//   93 /* Private functions ---------------------------------------------------------*/
//   94 
//   95 /** @defgroup RCC_Private_Functions
//   96   * @{
//   97   */
//   98 
//   99 /** @defgroup RCC_Group1 Internal and external clocks, PLL, CSS and MCO configuration functions
//  100  *  @brief   Internal and external clocks, PLL, CSS and MCO configuration functions 
//  101  *
//  102 @verbatim
//  103  ===============================================================================
//  104  ##### Internal-external clocks, PLL, CSS and MCO configuration functions #####
//  105  ===============================================================================
//  106     [..] This section provides functions allowing to configure the internal/external clocks,
//  107          PLL, CSS and MCO.
//  108          (#) HSI (high-speed internal), 8 MHz factory-trimmed RC used directly 
//  109              or through the PLL as System clock source.
//  110              The HSI clock can be used also to clock the USART, I2C and CEC peripherals.
//  111          (#) HSI14 (high-speed internal for ADC), 14 MHz factory-trimmed RC used to clock
//  112              the ADC peripheral.
//  113          (#) LSI (low-speed internal), 40 KHz low consumption RC used as IWDG and/or RTC
//  114              clock source.
//  115          (#) HSE (high-speed external), 4 to 32 MHz crystal oscillator used directly or
//  116              through the PLL as System clock source. Can be used also as RTC clock source.
//  117          (#) LSE (low-speed external), 32 KHz oscillator used as RTC clock source. 
//  118              LSE can be used also to clock the USART and CEC peripherals.   
//  119          (#) PLL (clocked by HSI or HSE), for System clock.
//  120          (#) CSS (Clock security system), once enabled and if a HSE clock failure occurs 
//  121              (HSE used directly or through PLL as System clock source), the System clock
//  122              is automatically switched to HSI and an interrupt is generated if enabled. 
//  123              The interrupt is linked to the Cortex-M0 NMI (Non-Maskable Interrupt) 
//  124              exception vector.   
//  125          (#) MCO (microcontroller clock output), used to output SYSCLK, HSI, HSI14, LSI,
//  126              HSE, LSE or PLL (divided by 2) clock on PA8 pin.
//  127 
//  128 @endverbatim
//  129   * @{
//  130   */
//  131 
//  132 /**
//  133   * @brief  Resets the RCC clock configuration to the default reset state.
//  134   * @note   The default reset state of the clock configuration is given below:
//  135   * @note      HSI ON and used as system clock source 
//  136   * @note      HSI14, HSE and PLL OFF
//  137   * @note      AHB, APB prescaler set to 1.
//  138   * @note      CSS and MCO OFF
//  139   * @note      All interrupts disabled
//  140   * @note   However, this function doesn't modify the configuration of the
//  141   * @note      Peripheral clocks
//  142   * @note      LSI, LSE and RTC clocks
//  143   * @param  None
//  144   * @retval None
//  145   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  146 void RCC_DeInit(void)
//  147 {
//  148   /* Set HSION bit */
//  149   RCC->CR |= (uint32_t)0x00000001;
RCC_DeInit:
        LDR      R0,??DataTable24  ;; 0x40021000
        LDR      R1,[R0, #+0]
        MOVS     R2,#+1
        ORRS     R2,R2,R1
        STR      R2,[R0, #+0]
//  150 
//  151 #if defined (STM32F051)
//  152   /* Reset SW[1:0], HPRE[3:0], PPRE[2:0] and MCOSEL[2:0] bits */
//  153   RCC->CFGR &= (uint32_t)0xF8FFB80C;
        LDR      R1,[R0, #+4]
        LDR      R2,??DataTable24_1  ;; 0xf8ffb80c
        ANDS     R2,R2,R1
        STR      R2,[R0, #+4]
//  154 #else
//  155   /* Reset SW[1:0], HPRE[3:0], PPRE[2:0], ADCPRE, MCOSEL[2:0], MCOPRE[2:0] and PLLNODIV bits */
//  156   RCC->CFGR &= (uint32_t)0x08FFB80C;
//  157 #endif /* STM32F051 */
//  158   
//  159   /* Reset HSEON, CSSON and PLLON bits */
//  160   RCC->CR &= (uint32_t)0xFEF6FFFF;
        LDR      R1,[R0, #+0]
        LDR      R2,??DataTable24_2  ;; 0xfef6ffff
        ANDS     R2,R2,R1
        STR      R2,[R0, #+0]
//  161 
//  162   /* Reset HSEBYP bit */
//  163   RCC->CR &= (uint32_t)0xFFFBFFFF;
        LDR      R1,[R0, #+0]
        LDR      R2,??DataTable24_3  ;; 0xfffbffff
        ANDS     R2,R2,R1
        STR      R2,[R0, #+0]
//  164 
//  165   /* Reset PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
//  166   RCC->CFGR &= (uint32_t)0xFFC0FFFF;
        LDR      R1,[R0, #+4]
        LDR      R2,??DataTable24_4  ;; 0xffc0ffff
        ANDS     R2,R2,R1
        STR      R2,[R0, #+4]
//  167 
//  168   /* Reset PREDIV1[3:0] bits */
//  169   RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;
        LDR      R1,[R0, #+44]
        MOVS     R2,#+15
        BICS     R1,R1,R2
        STR      R1,[R0, #+44]
//  170 
//  171   /* Reset USARTSW[1:0], I2CSW, CECSW and ADCSW bits */
//  172   RCC->CFGR3 &= (uint32_t)0xFFF0FEAC;
        LDR      R1,[R0, #+48]
        LDR      R2,??DataTable24_5  ;; 0xfff0feac
        ANDS     R2,R2,R1
        STR      R2,[R0, #+48]
//  173   
//  174   /* Reset HSI14 bit */
//  175   RCC->CR2 &= (uint32_t)0xFFFFFFFE;
        LDR      R1,[R0, #+52]
        MOVS     R2,#+1
        BICS     R1,R1,R2
        STR      R1,[R0, #+52]
//  176 
//  177   /* Disable all interrupts */
//  178   RCC->CIR = 0x00000000;
        MOVS     R1,#+0
        STR      R1,[R0, #+8]
//  179 }
        BX       LR               ;; return
//  180 
//  181 /**
//  182   * @brief  Configures the External High Speed oscillator (HSE).
//  183   * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
//  184   *         software should wait on HSERDY flag to be set indicating that HSE clock
//  185   *         is stable and can be used to clock the PLL and/or system clock.
//  186   * @note   HSE state can not be changed if it is used directly or through the
//  187   *         PLL as system clock. In this case, you have to select another source
//  188   *         of the system clock then change the HSE state (ex. disable it).
//  189   * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
//  190   * @note   This function resets the CSSON bit, so if the Clock security system(CSS)
//  191   *         was previously enabled you have to enable it again after calling this
//  192   *         function.
//  193   * @param  RCC_HSE: specifies the new state of the HSE.
//  194   *          This parameter can be one of the following values:
//  195   *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
//  196   *                              6 HSE oscillator clock cycles.
//  197   *            @arg RCC_HSE_ON: turn ON the HSE oscillator
//  198   *            @arg RCC_HSE_Bypass: HSE oscillator bypassed with external clock
//  199   * @retval None
//  200   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  201 void RCC_HSEConfig(uint8_t RCC_HSE)
//  202 {
//  203   /* Check the parameters */
//  204   assert_param(IS_RCC_HSE(RCC_HSE));
//  205 
//  206   /* Reset HSEON and HSEBYP bits before configuring the HSE ------------------*/
//  207   *(__IO uint8_t *) CR_BYTE2_ADDRESS = RCC_HSE_OFF;
RCC_HSEConfig:
        LDR      R1,??DataTable24_6  ;; 0x40021002
        MOVS     R2,#+0
        STRB     R2,[R1, #+0]
//  208 
//  209   /* Set the new HSE configuration -------------------------------------------*/
//  210   *(__IO uint8_t *) CR_BYTE2_ADDRESS = RCC_HSE;
        STRB     R0,[R1, #+0]
//  211 
//  212 }
        BX       LR               ;; return
//  213 
//  214 /**
//  215   * @brief  Waits for HSE start-up.
//  216   * @note   This function waits on HSERDY flag to be set and return SUCCESS if 
//  217   *         this flag is set, otherwise returns ERROR if the timeout is reached 
//  218   *         and this flag is not set. The timeout value is defined by the constant
//  219   *         HSE_STARTUP_TIMEOUT in stm32f0xx.h file. You can tailor it depending
//  220   *         on the HSE crystal used in your application.
//  221   * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
//  222   * @param  None
//  223   * @retval An ErrorStatus enumeration value:
//  224   *          - SUCCESS: HSE oscillator is stable and ready to use
//  225   *          - ERROR: HSE oscillator not yet ready
//  226   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  227 ErrorStatus RCC_WaitForHSEStartUp(void)
//  228 {
RCC_WaitForHSEStartUp:
        PUSH     {R4}
        SUB      SP,SP,#+8
//  229   __IO uint32_t StartUpCounter = 0;
        MOVS     R0,#+0
        STR      R0,[SP, #+0]
//  230   ErrorStatus status = ERROR;
//  231   FlagStatus HSEStatus = RESET;
        MOVS     R1,#+160
        LSLS     R1,R1,#+7        ;; #+20480
        LDR      R2,??DataTable24  ;; 0x40021000
//  232   
//  233   /* Wait till HSE is ready and if timeout is reached exit */
//  234   do
//  235   {
//  236     HSEStatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
??RCC_WaitForHSEStartUp_0:
        LDR      R3,[R2, #+0]
//  237     StartUpCounter++;  
        LDR      R4,[SP, #+0]
        ADDS     R4,R4,#+1
        STR      R4,[SP, #+0]
//  238   } while((StartUpCounter != HSE_STARTUP_TIMEOUT) && (HSEStatus == RESET));
        LDR      R4,[SP, #+0]
        CMP      R4,R1
        BEQ      ??RCC_WaitForHSEStartUp_1
        LSLS     R3,R3,#+14
        BPL      ??RCC_WaitForHSEStartUp_0
//  239   
//  240   if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
??RCC_WaitForHSEStartUp_1:
        LDR      R1,[R2, #+0]
        LSLS     R1,R1,#+14
        BPL      ??RCC_WaitForHSEStartUp_2
//  241   {
//  242     status = SUCCESS;
        MOVS     R0,#+1
//  243   }
//  244   else
//  245   {
//  246     status = ERROR;
//  247   }  
//  248   return (status);
??RCC_WaitForHSEStartUp_2:
        POP      {R1,R2,R4}
        BX       LR               ;; return
//  249 }
//  250 
//  251 /**
//  252   * @brief  Adjusts the Internal High Speed oscillator (HSI) calibration value.
//  253   * @note   The calibration is used to compensate for the variations in voltage
//  254   *         and temperature that influence the frequency of the internal HSI RC.
//  255   *         Refer to the Application Note AN4067 for more details on how to  
//  256   *         calibrate the HSI.
//  257   * @param  HSICalibrationValue: specifies the HSI calibration trimming value.
//  258   *          This parameter must be a number between 0 and 0x1F.
//  259   * @retval None
//  260   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  261 void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue)
//  262 {
//  263   uint32_t tmpreg = 0;
//  264   
//  265   /* Check the parameters */
//  266   assert_param(IS_RCC_HSI_CALIBRATION_VALUE(HSICalibrationValue));
//  267   
//  268   tmpreg = RCC->CR;
RCC_AdjustHSICalibrationValue:
        LDR      R1,??DataTable24  ;; 0x40021000
        B.N      ?Subroutine1
//  269   
//  270   /* Clear HSITRIM[4:0] bits */
//  271   tmpreg &= ~RCC_CR_HSITRIM;
//  272   
//  273   /* Set the HSITRIM[4:0] bits according to HSICalibrationValue value */
//  274   tmpreg |= (uint32_t)HSICalibrationValue << 3;
//  275 
//  276   /* Store the new value */
//  277   RCC->CR = tmpreg;
//  278 }
//  279 
//  280 /**
//  281   * @brief  Enables or disables the Internal High Speed oscillator (HSI).
//  282   * @note   After enabling the HSI, the application software should wait on 
//  283   *         HSIRDY flag to be set indicating that HSI clock is stable and can
//  284   *         be used to clock the PLL and/or system clock.
//  285   * @note   HSI can not be stopped if it is used directly or through the PLL
//  286   *         as system clock. In this case, you have to select another source 
//  287   *         of the system clock then stop the HSI.
//  288   * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
//  289   * @param  NewState: new state of the HSI.
//  290   *          This parameter can be: ENABLE or DISABLE.
//  291   * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
//  292   *         clock cycles.
//  293   * @retval None
//  294   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  295 void RCC_HSICmd(FunctionalState NewState)
//  296 {
//  297   /* Check the parameters */
//  298   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  299   
//  300   if (NewState != DISABLE)
RCC_HSICmd:
        LDR      R1,??DataTable24  ;; 0x40021000
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_HSICmd_0
//  301   {
//  302     RCC->CR |= RCC_CR_HSION;
        MOVS     R2,#+1
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
        BX       LR
//  303   }
//  304   else
//  305   {
//  306     RCC->CR &= ~RCC_CR_HSION;
??RCC_HSICmd_0:
        MOVS     R2,#+1
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
//  307   }
//  308 }
        BX       LR               ;; return
//  309 
//  310 /**
//  311   * @brief  Adjusts the Internal High Speed oscillator for ADC (HSI14) 
//  312   *         calibration value.
//  313   * @note   The calibration is used to compensate for the variations in voltage
//  314   *         and temperature that influence the frequency of the internal HSI RC.
//  315   *         Refer to the Application Note AN4067  for more details on how to  
//  316   *         calibrate the HSI14.
//  317   * @param  HSI14CalibrationValue: specifies the HSI14 calibration trimming value.
//  318   *          This parameter must be a number between 0 and 0x1F.
//  319   * @retval None
//  320   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  321 void RCC_AdjustHSI14CalibrationValue(uint8_t HSI14CalibrationValue)
//  322 {
//  323   uint32_t tmpreg = 0;
//  324   
//  325   /* Check the parameters */
//  326   assert_param(IS_RCC_HSI14_CALIBRATION_VALUE(HSI14CalibrationValue));
//  327   
//  328   tmpreg = RCC->CR2;
RCC_AdjustHSI14CalibrationValue:
        LDR      R1,??DataTable24_7  ;; 0x40021034
        REQUIRE ?Subroutine1
        ;; // Fall through to label ?Subroutine1
//  329   
//  330   /* Clear HSI14TRIM[4:0] bits */
//  331   tmpreg &= ~RCC_CR2_HSI14TRIM;
//  332   
//  333   /* Set the HSITRIM14[4:0] bits according to HSI14CalibrationValue value */
//  334   tmpreg |= (uint32_t)HSI14CalibrationValue << 3;
//  335 
//  336   /* Store the new value */
//  337   RCC->CR2 = tmpreg;
//  338 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine1:
        LDR      R2,[R1, #+0]
        MOVS     R3,#+248
        BICS     R2,R2,R3
        LSLS     R0,R0,#+3
        REQUIRE ??Subroutine2_0
        ;; // Fall through to label ??Subroutine2_0

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
??Subroutine2_0:
        ORRS     R0,R0,R2
        STR      R0,[R1, #+0]
        BX       LR               ;; return
//  339 
//  340 /**
//  341   * @brief  Enables or disables the Internal High Speed oscillator for ADC (HSI14).
//  342   * @note   After enabling the HSI14, the application software should wait on 
//  343   *         HSIRDY flag to be set indicating that HSI clock is stable and can
//  344   *         be used to clock the ADC.
//  345   * @note   The HSI14 is stopped by hardware when entering STOP and STANDBY modes.
//  346   * @param  NewState: new state of the HSI14.
//  347   *          This parameter can be: ENABLE or DISABLE.
//  348   * @note   When the HSI14 is stopped, HSI14RDY flag goes low after 6 HSI14 oscillator
//  349   *         clock cycles.
//  350   * @retval None
//  351   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  352 void RCC_HSI14Cmd(FunctionalState NewState)
//  353 {
//  354   /* Check the parameters */
//  355   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  356   
//  357   if (NewState != DISABLE)
RCC_HSI14Cmd:
        LDR      R1,??DataTable24_7  ;; 0x40021034
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_HSI14Cmd_0
//  358   {
//  359     RCC->CR2 |= RCC_CR2_HSI14ON;
        MOVS     R2,#+1
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
        BX       LR
//  360   }
//  361   else
//  362   {
//  363     RCC->CR2 &= ~RCC_CR2_HSI14ON;
??RCC_HSI14Cmd_0:
        MOVS     R2,#+1
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
//  364   }
//  365 }
        BX       LR               ;; return
//  366 
//  367 /**
//  368   * @brief  Enables or disables the Internal High Speed oscillator request from ADC.
//  369   * @param  NewState: new state of the HSI14 ADC request.
//  370   *          This parameter can be: ENABLE or DISABLE.
//  371   * @retval None
//  372   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  373 void RCC_HSI14ADCRequestCmd(FunctionalState NewState)
//  374 {
//  375   /* Check the parameters */
//  376   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  377   
//  378   if (NewState != DISABLE)
RCC_HSI14ADCRequestCmd:
        LDR      R1,??DataTable24_7  ;; 0x40021034
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_HSI14ADCRequestCmd_0
//  379   {
//  380     RCC->CR2 &= ~RCC_CR2_HSI14DIS;
        MOVS     R2,#+4
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
        BX       LR
//  381   }
//  382   else
//  383   {
//  384     RCC->CR2 |= RCC_CR2_HSI14DIS;
??RCC_HSI14ADCRequestCmd_0:
        MOVS     R2,#+4
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
//  385   }
//  386 }
        BX       LR               ;; return
//  387 
//  388 /**
//  389   * @brief  Configures the External Low Speed oscillator (LSE).
//  390   * @note   As the LSE is in the Backup domain and write access is denied to this
//  391   *         domain after reset, you have to enable write access using 
//  392   *         PWR_BackupAccessCmd(ENABLE) function before to configure the LSE
//  393   *         (to be done once after reset).
//  394   * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_Bypass), the application
//  395   *         software should wait on LSERDY flag to be set indicating that LSE clock
//  396   *         is stable and can be used to clock the RTC.
//  397   * @param  RCC_LSE: specifies the new state of the LSE.
//  398   *          This parameter can be one of the following values:
//  399   *            @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
//  400   *                              6 LSE oscillator clock cycles.
//  401   *            @arg RCC_LSE_ON: turn ON the LSE oscillator
//  402   *            @arg RCC_LSE_Bypass: LSE oscillator bypassed with external clock
//  403   * @retval None
//  404   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  405 void RCC_LSEConfig(uint32_t RCC_LSE)
//  406 {
//  407   /* Check the parameters */
//  408   assert_param(IS_RCC_LSE(RCC_LSE));
//  409 
//  410   /* Reset LSEON and LSEBYP bits before configuring the LSE ------------------*/
//  411   /* Reset LSEON bit */
//  412   RCC->BDCR &= ~(RCC_BDCR_LSEON);
RCC_LSEConfig:
        LDR      R1,??DataTable24_8  ;; 0x40021020
        LDR      R2,[R1, #+0]
        MOVS     R3,#+1
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
//  413 
//  414   /* Reset LSEBYP bit */
//  415   RCC->BDCR &= ~(RCC_BDCR_LSEBYP);
        LDR      R2,[R1, #+0]
        MOVS     R3,#+4
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
//  416 
//  417   /* Configure LSE */
//  418   RCC->BDCR |= RCC_LSE;
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
//  419 }
//  420 
//  421 /**
//  422   * @brief  Configures the External Low Speed oscillator (LSE) drive capability.
//  423   * @param  RCC_LSEDrive: specifies the new state of the LSE drive capability.
//  424   *          This parameter can be one of the following values:
//  425   *            @arg RCC_LSEDrive_Low: LSE oscillator low drive capability.
//  426   *            @arg RCC_LSEDrive_MediumLow: LSE oscillator medium low drive capability.
//  427   *            @arg RCC_LSEDrive_MediumHigh: LSE oscillator medium high drive capability.
//  428   *            @arg RCC_LSEDrive_High: LSE oscillator high drive capability.
//  429   * @retval None
//  430   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  431 void RCC_LSEDriveConfig(uint32_t RCC_LSEDrive)
//  432 {
//  433   /* Check the parameters */
//  434   assert_param(IS_RCC_LSE_DRIVE(RCC_LSEDrive));
//  435   
//  436   /* Clear LSEDRV[1:0] bits */
//  437   RCC->BDCR &= ~(RCC_BDCR_LSEDRV);
RCC_LSEDriveConfig:
        LDR      R1,??DataTable24_8  ;; 0x40021020
        LDR      R2,[R1, #+0]
        MOVS     R3,#+24
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
//  438 
//  439   /* Set the LSE Drive */
//  440   RCC->BDCR |= RCC_LSEDrive;
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
//  441 }
//  442 
//  443 /**
//  444   * @brief  Enables or disables the Internal Low Speed oscillator (LSI).
//  445   * @note   After enabling the LSI, the application software should wait on 
//  446   *         LSIRDY flag to be set indicating that LSI clock is stable and can
//  447   *         be used to clock the IWDG and/or the RTC.
//  448   * @note   LSI can not be disabled if the IWDG is running.
//  449   * @param  NewState: new state of the LSI.
//  450   *          This parameter can be: ENABLE or DISABLE.
//  451   * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
//  452   *         clock cycles.
//  453   * @retval None
//  454   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  455 void RCC_LSICmd(FunctionalState NewState)
//  456 {
//  457   /* Check the parameters */
//  458   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  459   
//  460   if (NewState != DISABLE)
RCC_LSICmd:
        LDR      R1,??DataTable26  ;; 0x40021024
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_LSICmd_0
//  461   {
//  462     RCC->CSR |= RCC_CSR_LSION;
        MOVS     R2,#+1
        ORRS     R2,R2,R0
        STR      R2,[R1, #+0]
        BX       LR
//  463   }
//  464   else
//  465   {
//  466     RCC->CSR &= ~RCC_CSR_LSION;
??RCC_LSICmd_0:
        MOVS     R2,#+1
        BICS     R0,R0,R2
        STR      R0,[R1, #+0]
//  467   }
//  468 }
        BX       LR               ;; return
//  469 
//  470 /**
//  471   * @brief  Configures the PLL clock source and multiplication factor.
//  472   * @note   This function must be used only when the PLL is disabled.
//  473   *
//  474   * @param  RCC_PLLSource: specifies the PLL entry clock source.
//  475   *          This parameter can be one of the following values:
//  476   *            @arg RCC_PLLSource_HSI_Div2: HSI oscillator clock selected as PLL clock source
//  477   *            @arg RCC_PLLSource_PREDIV1: PREDIV1 clock selected as PLL clock entry
//  478   *            @arg RCC_PLLSource_HSI48 HSI48 oscillator clock selected as PLL clock source, applicable only for STM32F072 devices
//  479   *            @arg RCC_PLLSource_HSI: HSI clock selected as PLL clock entry, applicable only for STM32F072 devices
//  480   * @note   The minimum input clock frequency for PLL is 2 MHz (when using HSE as
//  481   *         PLL source).
//  482   *
//  483   * @param  RCC_PLLMul: specifies the PLL multiplication factor, which drive the PLLVCO clock
//  484   *          This parameter can be RCC_PLLMul_x where x:[2,16] 
//  485   *
//  486   * @retval None
//  487   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  488 void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul)
//  489 {
RCC_PLLConfig:
        PUSH     {R4}
//  490   /* Check the parameters */
//  491   assert_param(IS_RCC_PLL_SOURCE(RCC_PLLSource));
//  492   assert_param(IS_RCC_PLL_MUL(RCC_PLLMul));
//  493 
//  494   /* Clear PLL Source [16] and Multiplier [21:18] bits */
//  495   RCC->CFGR &= ~(RCC_CFGR_PLLMULL | RCC_CFGR_PLLSRC);
        LDR      R2,??DataTable24_9  ;; 0x40021004
        LDR      R3,[R2, #+0]
        LDR      R4,??DataTable27  ;; 0xffc27fff
        ANDS     R4,R4,R3
        STR      R4,[R2, #+0]
//  496 
//  497   /* Set the PLL Source and Multiplier */
//  498   RCC->CFGR |= (uint32_t)(RCC_PLLSource | RCC_PLLMul);
        LDR      R3,[R2, #+0]
        ORRS     R1,R1,R0
        ORRS     R1,R1,R3
        STR      R1,[R2, #+0]
//  499 }
        POP      {R4}
        BX       LR               ;; return
//  500 
//  501 /**
//  502   * @brief  Enables or disables the PLL.
//  503   * @note   After enabling the PLL, the application software should wait on 
//  504   *         PLLRDY flag to be set indicating that PLL clock is stable and can
//  505   *         be used as system clock source.
//  506   * @note   The PLL can not be disabled if it is used as system clock source
//  507   * @note   The PLL is disabled by hardware when entering STOP and STANDBY modes.
//  508   * @param  NewState: new state of the PLL.
//  509   *          This parameter can be: ENABLE or DISABLE.
//  510   * @retval None
//  511   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  512 void RCC_PLLCmd(FunctionalState NewState)
//  513 {
//  514   /* Check the parameters */
//  515   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  516   
//  517   if (NewState != DISABLE)
RCC_PLLCmd:
        LDR      R1,??DataTable24  ;; 0x40021000
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_PLLCmd_0
//  518   {
//  519     RCC->CR |= RCC_CR_PLLON;
        MOVS     R2,#+128
        LSLS     R2,R2,#+17       ;; #+16777216
        ORRS     R2,R2,R0
        B        ??RCC_PLLCmd_1
//  520   }
//  521   else
//  522   {
//  523     RCC->CR &= ~RCC_CR_PLLON;
??RCC_PLLCmd_0:
        LDR      R2,??DataTable28  ;; 0xfeffffff
        ANDS     R2,R2,R0
??RCC_PLLCmd_1:
        STR      R2,[R1, #+0]
//  524   }
//  525 }
        BX       LR               ;; return
//  526 
//  527 /**
//  528   * @brief  Enables or disables the Internal High Speed oscillator for USB (HSI48).
//  529   *         This function is only applicable for STM32F072 devices.  
//  530   * @note   After enabling the HSI48, the application software should wait on 
//  531   *         HSI48RDY flag to be set indicating that HSI48 clock is stable and can
//  532   *         be used to clock the USB.
//  533   * @note   The HSI48 is stopped by hardware when entering STOP and STANDBY modes.
//  534   * @param  NewState: new state of the HSI48.
//  535   *          This parameter can be: ENABLE or DISABLE.
//  536   * @retval None
//  537   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  538 void RCC_HSI48Cmd(FunctionalState NewState)
//  539 {
//  540   /* Check the parameters */
//  541   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  542   
//  543   if (NewState != DISABLE)
RCC_HSI48Cmd:
        LDR      R1,??DataTable24_7  ;; 0x40021034
        B.N      ?Subroutine0
//  544   {
//  545     RCC->CR2 |= RCC_CR2_HSI48ON;
//  546   }
//  547   else
//  548   {
//  549     RCC->CR2 &= ~RCC_CR2_HSI48ON;
//  550   }
//  551 }
//  552 
//  553 /**
//  554   * @brief  Configures the PREDIV1 division factor.
//  555   * @note   This function must be used only when the PLL is disabled.
//  556   * @param  RCC_PREDIV1_Div: specifies the PREDIV1 clock division factor.
//  557   *          This parameter can be RCC_PREDIV1_Divx where x:[1,16]
//  558   * @retval None
//  559   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  560 void RCC_PREDIV1Config(uint32_t RCC_PREDIV1_Div)
//  561 {
//  562   uint32_t tmpreg = 0;
//  563   
//  564   /* Check the parameters */
//  565   assert_param(IS_RCC_PREDIV1(RCC_PREDIV1_Div));
//  566 
//  567   tmpreg = RCC->CFGR2;
RCC_PREDIV1Config:
        LDR      R1,??DataTable29  ;; 0x4002102c
        LDR      R2,[R1, #+0]
//  568   /* Clear PREDIV1[3:0] bits */
//  569   tmpreg &= ~(RCC_CFGR2_PREDIV1);
//  570   /* Set the PREDIV1 division factor */
//  571   tmpreg |= RCC_PREDIV1_Div;
//  572   /* Store the new value */
//  573   RCC->CFGR2 = tmpreg;
        MOVS     R3,#+15
        BICS     R2,R2,R3
        B.N      ??Subroutine2_0
//  574 }
//  575 
//  576 /**
//  577   * @brief  Enables or disables the Clock Security System.
//  578   * @note   If a failure is detected on the HSE oscillator clock, this oscillator
//  579   *         is automatically disabled and an interrupt is generated to inform the
//  580   *         software about the failure (Clock Security System Interrupt, CSSI),
//  581   *         allowing the MCU to perform rescue operations. The CSSI is linked to 
//  582   *         the Cortex-M0 NMI (Non-Maskable Interrupt) exception vector.
//  583   * @param  NewState: new state of the Clock Security System.
//  584   *          This parameter can be: ENABLE or DISABLE.
//  585   * @retval None
//  586   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  587 void RCC_ClockSecuritySystemCmd(FunctionalState NewState)
//  588 {
//  589   /* Check the parameters */
//  590   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  591   
//  592   if (NewState != DISABLE)
RCC_ClockSecuritySystemCmd:
        LDR      R1,??DataTable24  ;; 0x40021000
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_ClockSecuritySystemCmd_0
//  593   {
//  594     RCC->CR |= RCC_CR_CSSON;
        MOVS     R2,#+128
        LSLS     R2,R2,#+12       ;; #+524288
        ORRS     R2,R2,R0
        B        ??RCC_ClockSecuritySystemCmd_1
//  595   }
//  596   else
//  597   {
//  598     RCC->CR &= ~RCC_CR_CSSON;
??RCC_ClockSecuritySystemCmd_0:
        LDR      R2,??DataTable30  ;; 0xfff7ffff
        ANDS     R2,R2,R0
??RCC_ClockSecuritySystemCmd_1:
        STR      R2,[R1, #+0]
//  599   }
//  600 }
        BX       LR               ;; return
//  601 
//  602 #ifdef STM32F051
//  603 /**
//  604   * @brief  Selects the clock source to output on MCO pin (PA8).
//  605   * @note   PA8 should be configured in alternate function mode.
//  606   * @param  RCC_MCOSource: specifies the clock source to output.
//  607   *          This parameter can be one of the following values:
//  608   *            @arg RCC_MCOSource_NoClock: No clock selected.
//  609   *            @arg RCC_MCOSource_HSI14: HSI14 oscillator clock selected.
//  610   *            @arg RCC_MCOSource_LSI: LSI oscillator clock selected.
//  611   *            @arg RCC_MCOSource_LSE: LSE oscillator clock selected.
//  612   *            @arg RCC_MCOSource_SYSCLK: System clock selected.
//  613   *            @arg RCC_MCOSource_HSI: HSI oscillator clock selected.
//  614   *            @arg RCC_MCOSource_HSE: HSE oscillator clock selected.
//  615   *            @arg RCC_MCOSource_PLLCLK_Div2: PLL clock divided by 2 selected.
//  616   * @retval None
//  617   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  618 void RCC_MCOConfig(uint8_t RCC_MCOSource)
//  619 {
//  620   /* Check the parameters */
//  621   assert_param(IS_RCC_MCO_SOURCE(RCC_MCOSource));
//  622 
//  623   /* Select MCO clock source and prescaler */
//  624   *(__IO uint8_t *) CFGR_BYTE3_ADDRESS =  RCC_MCOSource;
RCC_MCOConfig:
        LDR      R1,??DataTable31  ;; 0x40021007
        STRB     R0,[R1, #+0]
//  625 }
        BX       LR               ;; return
//  626 #else
//  627 
//  628 /**
//  629   * @brief  Selects the clock source to output on MCO pin (PA8) and the corresponding
//  630   *         prescsaler.
//  631   * @note   PA8 should be configured in alternate function mode.
//  632   * @param  RCC_MCOSource: specifies the clock source to output.
//  633   *          This parameter can be one of the following values:
//  634   *            @arg RCC_MCOSource_NoClock: No clock selected.
//  635   *            @arg RCC_MCOSource_HSI14: HSI14 oscillator clock selected.
//  636   *            @arg RCC_MCOSource_LSI: LSI oscillator clock selected.
//  637   *            @arg RCC_MCOSource_LSE: LSE oscillator clock selected.
//  638   *            @arg RCC_MCOSource_SYSCLK: System clock selected.
//  639   *            @arg RCC_MCOSource_HSI: HSI oscillator clock selected.
//  640   *            @arg RCC_MCOSource_HSE: HSE oscillator clock selected.
//  641   *            @arg RCC_MCOSource_PLLCLK_Div2: PLL clock divided by 2 selected.
//  642   *            @arg RCC_MCOSource_PLLCLK: PLL clock selected.
//  643   *            @arg RCC_MCOSource_HSI48: HSI48 clock selected.
//  644   * @param  RCC_MCOPrescaler: specifies the prescaler on MCO pin.
//  645   *          This parameter can be one of the following values:
//  646   *            @arg RCC_MCOPrescaler_1: MCO clock is divided by 1.
//  647   *            @arg RCC_MCOPrescaler_2: MCO clock is divided by 2.
//  648   *            @arg RCC_MCOPrescaler_4: MCO clock is divided by 4.
//  649   *            @arg RCC_MCOPrescaler_8: MCO clock is divided by 8.
//  650   *            @arg RCC_MCOPrescaler_16: MCO clock is divided by 16.
//  651   *            @arg RCC_MCOPrescaler_32: MCO clock is divided by 32.
//  652   *            @arg RCC_MCOPrescaler_64: MCO clock is divided by 64.
//  653   *            @arg RCC_MCOPrescaler_128: MCO clock is divided by 128.    
//  654   * @retval None
//  655   */
//  656 void RCC_MCOConfig(uint8_t RCC_MCOSource, uint32_t RCC_MCOPrescaler)
//  657 {
//  658   uint32_t tmpreg = 0;
//  659   
//  660   /* Check the parameters */
//  661   assert_param(IS_RCC_MCO_SOURCE(RCC_MCOSource));
//  662   assert_param(IS_RCC_MCO_PRESCALER(RCC_MCOPrescaler));
//  663     
//  664   /* Get CFGR value */  
//  665   tmpreg = RCC->CFGR;
//  666   /* Clear MCOPRE[2:0] bits */
//  667   tmpreg &= ~(RCC_CFGR_MCO_PRE | RCC_CFGR_MCO | RCC_CFGR_PLLNODIV);
//  668   /* Set the RCC_MCOSource and RCC_MCOPrescaler */
//  669   tmpreg |= (RCC_MCOPrescaler | ((uint32_t)RCC_MCOSource<<24));
//  670   /* Store the new value */
//  671   RCC->CFGR = tmpreg;
//  672 }
//  673 #endif /* STM32F072 */
//  674 
//  675 /**
//  676   * @}
//  677   */
//  678 
//  679 /** @defgroup RCC_Group2 System AHB and APB busses clocks configuration functions
//  680  *  @brief   System, AHB and APB busses clocks configuration functions
//  681  *
//  682 @verbatim
//  683  ===============================================================================
//  684      ##### System, AHB and APB busses clocks configuration functions #####
//  685  ===============================================================================
//  686 
//  687     [..] This section provide functions allowing to configure the System, AHB and 
//  688          APB busses clocks.
//  689          (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
//  690              HSE and PLL.
//  691              The AHB clock (HCLK) is derived from System clock through configurable prescaler
//  692              and used to clock the CPU, memory and peripherals mapped on AHB bus (DMA and GPIO).
//  693              and APB (PCLK) clocks are derived from AHB clock through 
//  694              configurable prescalers and used to clock the peripherals mapped on these busses.
//  695              You can use "RCC_GetClocksFreq()" function to retrieve the frequencies of these clocks.
//  696 
//  697          -@- All the peripheral clocks are derived from the System clock (SYSCLK) except:
//  698              (+@) The ADC clock which is derived from HSI14 or APB (APB divided by a
//  699                   programmable prescaler: 2 or 4).
//  700              (+@) The CEC clock which is derived from LSE or HSI divided by 244.
//  701              (+@) The I2C clock which is derived from HSI or system clock (SYSCLK).
//  702              (+@) The USART clock which is derived from HSI, system clock (SYSCLK), APB or LSE.
//  703              (+@) The RTC/LCD clock which is derived from the LSE, LSI or 2 MHz HSE_RTC (HSE
//  704                   divided by a programmable prescaler).
//  705                   The System clock (SYSCLK) frequency must be higher or equal to the RTC/LCD
//  706                   clock frequency.
//  707              (+@) IWDG clock which is always the LSI clock.
//  708        
//  709          (#) The maximum frequency of the SYSCLK, HCLK and PCLK is 48 MHz.
//  710              Depending on the maximum frequency, the FLASH wait states (WS) should be 
//  711              adapted accordingly:
//  712         +--------------------------------------------- +
//  713         |  Wait states  |   HCLK clock frequency (MHz) |
//  714         |---------------|------------------------------|
//  715         |0WS(1CPU cycle)|       0 < HCLK <= 24         |
//  716         |---------------|------------------------------|
//  717         |1WS(2CPU cycle)|       24 < HCLK <= 48        |
//  718         +----------------------------------------------+
//  719 
//  720          (#) After reset, the System clock source is the HSI (8 MHz) with 0 WS and 
//  721              prefetch is disabled.
//  722   
//  723     [..] It is recommended to use the following software sequences to tune the number
//  724          of wait states needed to access the Flash memory with the CPU frequency (HCLK).
//  725          (+) Increasing the CPU frequency
//  726          (++) Program the Flash Prefetch buffer, using "FLASH_PrefetchBufferCmd(ENABLE)" 
//  727               function
//  728          (++) Check that Flash Prefetch buffer activation is taken into account by 
//  729               reading FLASH_ACR using the FLASH_GetPrefetchBufferStatus() function
//  730          (++) Program Flash WS to 1, using "FLASH_SetLatency(FLASH_Latency_1)" function
//  731          (++) Check that the new number of WS is taken into account by reading FLASH_ACR
//  732          (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
//  733          (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
//  734          (++) Check that the new CPU clock source is taken into account by reading 
//  735               the clock source status, using "RCC_GetSYSCLKSource()" function 
//  736          (+) Decreasing the CPU frequency
//  737          (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
//  738          (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
//  739          (++) Check that the new CPU clock source is taken into account by reading 
//  740               the clock source status, using "RCC_GetSYSCLKSource()" function
//  741          (++) Program the new number of WS, using "FLASH_SetLatency()" function
//  742          (++) Check that the new number of WS is taken into account by reading FLASH_ACR
//  743          (++) Disable the Flash Prefetch buffer using "FLASH_PrefetchBufferCmd(DISABLE)" 
//  744               function
//  745          (++) Check that Flash Prefetch buffer deactivation is taken into account by reading FLASH_ACR
//  746               using the FLASH_GetPrefetchBufferStatus() function.
//  747 
//  748 @endverbatim
//  749   * @{
//  750   */
//  751 
//  752 /**
//  753   * @brief  Configures the system clock (SYSCLK).
//  754   * @note   The HSI is used (enabled by hardware) as system clock source after
//  755   *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
//  756   *         of failure of the HSE used directly or indirectly as system clock
//  757   *         (if the Clock Security System CSS is enabled).
//  758   * @note   A switch from one clock source to another occurs only if the target
//  759   *         clock source is ready (clock stable after startup delay or PLL locked). 
//  760   *         If a clock source which is not yet ready is selected, the switch will
//  761   *         occur when the clock source will be ready. 
//  762   *         You can use RCC_GetSYSCLKSource() function to know which clock is
//  763   *         currently used as system clock source.  
//  764   * @param  RCC_SYSCLKSource: specifies the clock source used as system clock source 
//  765   *          This parameter can be one of the following values:
//  766   *            @arg RCC_SYSCLKSource_HSI:    HSI selected as system clock source
//  767   *            @arg RCC_SYSCLKSource_HSE:    HSE selected as system clock source
//  768   *            @arg RCC_SYSCLKSource_PLLCLK: PLL selected as system clock source
//  769   *            @arg RCC_SYSCLKSource_HSI48:  HSI48 selected as system clock source, applicable only for STM32F072 devices  
//  770   * @retval None
//  771   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  772 void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource)
//  773 {
//  774   uint32_t tmpreg = 0;
//  775   
//  776   /* Check the parameters */
//  777   assert_param(IS_RCC_SYSCLK_SOURCE(RCC_SYSCLKSource));
//  778   
//  779   tmpreg = RCC->CFGR;
RCC_SYSCLKConfig:
        LDR      R1,??DataTable24_9  ;; 0x40021004
        LDR      R2,[R1, #+0]
//  780   
//  781   /* Clear SW[1:0] bits */
//  782   tmpreg &= ~RCC_CFGR_SW;
//  783   
//  784   /* Set SW[1:0] bits according to RCC_SYSCLKSource value */
//  785   tmpreg |= RCC_SYSCLKSource;
//  786   
//  787   /* Store the new value */
//  788   RCC->CFGR = tmpreg;
        MOVS     R3,#+3
        BICS     R2,R2,R3
        B.N      ??Subroutine2_0
//  789 }
//  790 
//  791 /**
//  792   * @brief  Returns the clock source used as system clock.
//  793   * @param  None
//  794   * @retval The clock source used as system clock. The returned value can be one 
//  795   *         of the following values:
//  796   *           - 0x00: HSI used as system clock
//  797   *           - 0x04: HSE used as system clock  
//  798   *           - 0x08: PLL used as system clock
//  799   *           - 0x0C: HSI48 used as system clock, applicable only for STM32F072 devices  
//  800   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  801 uint8_t RCC_GetSYSCLKSource(void)
//  802 {
//  803   return ((uint8_t)(RCC->CFGR & RCC_CFGR_SWS));
RCC_GetSYSCLKSource:
        LDR      R0,??DataTable24_9  ;; 0x40021004
        LDR      R1,[R0, #+0]
        MOVS     R0,#+12
        ANDS     R0,R0,R1
        BX       LR               ;; return
//  804 }
//  805 
//  806 /**
//  807   * @brief  Configures the AHB clock (HCLK).
//  808   * @param  RCC_SYSCLK: defines the AHB clock divider. This clock is derived from 
//  809   *         the system clock (SYSCLK).
//  810   *          This parameter can be one of the following values:
//  811   *            @arg RCC_SYSCLK_Div1:   AHB clock = SYSCLK
//  812   *            @arg RCC_SYSCLK_Div2:   AHB clock = SYSCLK/2
//  813   *            @arg RCC_SYSCLK_Div4:   AHB clock = SYSCLK/4
//  814   *            @arg RCC_SYSCLK_Div8:   AHB clock = SYSCLK/8
//  815   *            @arg RCC_SYSCLK_Div16:  AHB clock = SYSCLK/16
//  816   *            @arg RCC_SYSCLK_Div64:  AHB clock = SYSCLK/64
//  817   *            @arg RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
//  818   *            @arg RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
//  819   *            @arg RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
//  820   * @retval None
//  821   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  822 void RCC_HCLKConfig(uint32_t RCC_SYSCLK)
//  823 {
//  824   uint32_t tmpreg = 0;
//  825   
//  826   /* Check the parameters */
//  827   assert_param(IS_RCC_HCLK(RCC_SYSCLK));
//  828   
//  829   tmpreg = RCC->CFGR;
RCC_HCLKConfig:
        LDR      R1,??DataTable24_9  ;; 0x40021004
        LDR      R2,[R1, #+0]
//  830   
//  831   /* Clear HPRE[3:0] bits */
//  832   tmpreg &= ~RCC_CFGR_HPRE;
//  833   
//  834   /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
//  835   tmpreg |= RCC_SYSCLK;
//  836   
//  837   /* Store the new value */
//  838   RCC->CFGR = tmpreg;
        MOVS     R3,#+240
        BICS     R2,R2,R3
        B.N      ??Subroutine2_0
//  839 }
//  840 
//  841 /**
//  842   * @brief  Configures the APB clock (PCLK).
//  843   * @param  RCC_HCLK: defines the APB clock divider. This clock is derived from 
//  844   *         the AHB clock (HCLK).
//  845   *          This parameter can be one of the following values:
//  846   *            @arg RCC_HCLK_Div1: APB clock = HCLK
//  847   *            @arg RCC_HCLK_Div2: APB clock = HCLK/2
//  848   *            @arg RCC_HCLK_Div4: APB clock = HCLK/4
//  849   *            @arg RCC_HCLK_Div8: APB clock = HCLK/8
//  850   *            @arg RCC_HCLK_Div16: APB clock = HCLK/16
//  851   * @retval None
//  852   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  853 void RCC_PCLKConfig(uint32_t RCC_HCLK)
//  854 {
//  855   uint32_t tmpreg = 0;
//  856   
//  857   /* Check the parameters */
//  858   assert_param(IS_RCC_PCLK(RCC_HCLK));
//  859   
//  860   tmpreg = RCC->CFGR;
RCC_PCLKConfig:
        LDR      R1,??DataTable24_9  ;; 0x40021004
        LDR      R2,[R1, #+0]
//  861   
//  862   /* Clear PPRE[2:0] bits */
//  863   tmpreg &= ~RCC_CFGR_PPRE;
//  864   
//  865   /* Set PPRE[2:0] bits according to RCC_HCLK value */
//  866   tmpreg |= RCC_HCLK;
//  867   
//  868   /* Store the new value */
//  869   RCC->CFGR = tmpreg;
        LDR      R3,??DataTable33  ;; 0xfffff8ff
        ANDS     R3,R3,R2
        ORRS     R0,R0,R3
        STR      R0,[R1, #+0]
//  870 }
        BX       LR               ;; return
//  871 
//  872 /**
//  873   * @brief  Configures the ADC clock (ADCCLK).
//  874   * @note   This function is obsolete.
//  875   *         For proper ADC clock selection, refer to ADC_ClockModeConfig() in the ADC driver
//  876   * @param  RCC_ADCCLK: defines the ADC clock source. This clock is derived 
//  877   *         from the HSI14 or APB clock (PCLK).
//  878   *          This parameter can be one of the following values:
//  879   *             @arg RCC_ADCCLK_HSI14: ADC clock = HSI14 (14MHz)
//  880   *             @arg RCC_ADCCLK_PCLK_Div2: ADC clock = PCLK/2
//  881   *             @arg RCC_ADCCLK_PCLK_Div4: ADC clock = PCLK/4  
//  882   * @retval None
//  883   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  884 void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK)
//  885 { 
//  886   /* Check the parameters */
//  887   assert_param(IS_RCC_ADCCLK(RCC_ADCCLK));
//  888 
//  889   /* Clear ADCPRE bit */
//  890   RCC->CFGR &= ~RCC_CFGR_ADCPRE;
RCC_ADCCLKConfig:
        LDR      R1,??DataTable24_9  ;; 0x40021004
        LDR      R2,[R1, #+0]
        LDR      R3,??DataTable32  ;; 0xffffbfff
        ANDS     R3,R3,R2
        STR      R3,[R1, #+0]
//  891   /* Set ADCPRE bits according to RCC_PCLK value */
//  892   RCC->CFGR |= RCC_ADCCLK & 0xFFFF;
        LDR      R2,[R1, #+0]
        UXTH     R3,R0
        ORRS     R3,R3,R2
        STR      R3,[R1, #+0]
//  893 
//  894   /* Clear ADCSW bit */
//  895   RCC->CFGR3 &= ~RCC_CFGR3_ADCSW; 
        LDR      R2,[R1, #+44]
        LDR      R3,??DataTable34  ;; 0xfffffeff
        ANDS     R3,R3,R2
        STR      R3,[R1, #+44]
//  896   /* Set ADCSW bits according to RCC_ADCCLK value */
//  897   RCC->CFGR3 |= RCC_ADCCLK >> 16;  
        LDR      R2,[R1, #+44]
        LSRS     R0,R0,#+16
        ORRS     R0,R0,R2
        STR      R0,[R1, #+44]
//  898 }
        BX       LR               ;; return
//  899 
//  900 /**
//  901   * @brief  Configures the CEC clock (CECCLK).
//  902   * @param  RCC_CECCLK: defines the CEC clock source. This clock is derived 
//  903   *         from the HSI or LSE clock.
//  904   *          This parameter can be one of the following values:
//  905   *             @arg RCC_CECCLK_HSI_Div244: CEC clock = HSI/244 (32768Hz)
//  906   *             @arg RCC_CECCLK_LSE: CEC clock = LSE
//  907   * @retval None
//  908   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  909 void RCC_CECCLKConfig(uint32_t RCC_CECCLK)
//  910 { 
//  911   /* Check the parameters */
//  912   assert_param(IS_RCC_CECCLK(RCC_CECCLK));
//  913 
//  914   /* Clear CECSW bit */
//  915   RCC->CFGR3 &= ~RCC_CFGR3_CECSW;
RCC_CECCLKConfig:
        LDR      R1,??DataTable34_1  ;; 0x40021030
        LDR      R2,[R1, #+0]
        MOVS     R3,#+64
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
//  916   /* Set CECSW bits according to RCC_CECCLK value */
//  917   RCC->CFGR3 |= RCC_CECCLK;
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
//  918 }
//  919 
//  920 /**
//  921   * @brief  Configures the I2C1 clock (I2C1CLK).
//  922   * @param  RCC_I2CCLK: defines the I2C1 clock source. This clock is derived 
//  923   *         from the HSI or System clock.
//  924   *          This parameter can be one of the following values:
//  925   *             @arg RCC_I2C1CLK_HSI: I2C1 clock = HSI
//  926   *             @arg RCC_I2C1CLK_SYSCLK: I2C1 clock = System Clock
//  927   * @retval None
//  928   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  929 void RCC_I2CCLKConfig(uint32_t RCC_I2CCLK)
//  930 { 
//  931   /* Check the parameters */
//  932   assert_param(IS_RCC_I2CCLK(RCC_I2CCLK));
//  933 
//  934   /* Clear I2CSW bit */
//  935   RCC->CFGR3 &= ~RCC_CFGR3_I2C1SW;
RCC_I2CCLKConfig:
        LDR      R1,??DataTable35  ;; 0x40021030
        LDR      R2,[R1, #+0]
        MOVS     R3,#+16
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
//  936   /* Set I2CSW bits according to RCC_I2CCLK value */
//  937   RCC->CFGR3 |= RCC_I2CCLK;
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
//  938 }
//  939 
//  940 /**
//  941   * @brief  Configures the USART1 clock (USART1CLK).
//  942   * @param  RCC_USARTCLK: defines the USART clock source. This clock is derived 
//  943   *         from the HSI or System clock.
//  944   *          This parameter can be one of the following values:
//  945   *             @arg RCC_USART1CLK_PCLK: USART1 clock = APB Clock (PCLK)
//  946   *             @arg RCC_USART1CLK_SYSCLK: USART1 clock = System Clock
//  947   *             @arg RCC_USART1CLK_LSE: USART1 clock = LSE Clock
//  948   *             @arg RCC_USART1CLK_HSI: USART1 clock = HSI Clock
//  949   *             @arg RCC_USART2CLK_PCLK: USART2 clock = APB Clock (PCLK), applicable only for STM32F072 and STM32F091 devices
//  950   *             @arg RCC_USART2CLK_SYSCLK: USART2 clock = System Clock, applicable only for STM32F072 and STM32F091 devices
//  951   *             @arg RCC_USART2CLK_LSE: USART2 clock = LSE Clock, applicable only for STM32F072 and STM32F091 devices
//  952   *             @arg RCC_USART2CLK_HSI: USART2 clock = HSI Clock, applicable only for STM32F072 and STM32F091 devices  
//  953   *             @arg RCC_USART3CLK_PCLK: USART3 clock = APB Clock (PCLK), applicable only for STM32F091 devices
//  954   *             @arg RCC_USART3CLK_SYSCLK: USART3 clock = System Clock, applicable only for STM32F091 devices
//  955   *             @arg RCC_USART3CLK_LSE: USART3 clock = LSE Clock, applicable only for STM32F091 devices
//  956   *             @arg RCC_USART3CLK_HSI: USART3 clock = HSI Clock, applicable only for STM32F091 devices   
//  957   * @retval None
//  958   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  959 void RCC_USARTCLKConfig(uint32_t RCC_USARTCLK)
//  960 { 
//  961   uint32_t tmp = 0;
//  962   
//  963   /* Check the parameters */
//  964   assert_param(IS_RCC_USARTCLK(RCC_USARTCLK));
//  965 
//  966   /* Get USART index */
//  967   tmp = (RCC_USARTCLK >> 28);
RCC_USARTCLKConfig:
        LSRS     R2,R0,#+28
//  968 
//  969   /* Clear USARTSW[1:0] bit */
//  970   if (tmp == (uint32_t)0x00000001)
        LDR      R1,??DataTable34_1  ;; 0x40021030
        CMP      R2,#+1
        BNE      ??RCC_USARTCLKConfig_0
//  971   {
//  972     /* Clear USART1SW[1:0] bit */  
//  973     RCC->CFGR3 &= ~RCC_CFGR3_USART1SW;
        LDR      R2,[R1, #+0]
        MOVS     R3,#+3
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
        B        ??RCC_USARTCLKConfig_1
//  974   }
//  975   else if (tmp == (uint32_t)0x00000002)
??RCC_USARTCLKConfig_0:
        CMP      R2,#+2
        LDR      R2,[R1, #+0]
        BNE      ??RCC_USARTCLKConfig_2
//  976   {
//  977     /* Clear USART2SW[1:0] bit */
//  978     RCC->CFGR3 &= ~RCC_CFGR3_USART2SW;
        LDR      R3,??DataTable35_1  ;; 0xfffcffff
        B        ??RCC_USARTCLKConfig_3
//  979   }
//  980   else 
//  981   {
//  982     /* Clear USART3SW[1:0] bit */
//  983     RCC->CFGR3 &= ~RCC_CFGR3_USART3SW;
??RCC_USARTCLKConfig_2:
        LDR      R3,??DataTable36  ;; 0xfff3ffff
??RCC_USARTCLKConfig_3:
        ANDS     R3,R3,R2
        STR      R3,[R1, #+0]
//  984   }
//  985 
//  986   /* Set USARTxSW bits according to RCC_USARTCLK value */
//  987   RCC->CFGR3 |= RCC_USARTCLK;
??RCC_USARTCLKConfig_1:
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
//  988 }
//  989 
//  990 /**
//  991   * @brief  Configures the USB clock (USBCLK).
//  992   *         This function is only applicable for STM32F072 devices.  
//  993   * @param  RCC_USBCLK: defines the USB clock source. This clock is derived 
//  994   *         from the HSI48 or system clock.
//  995   *          This parameter can be one of the following values:
//  996   *             @arg RCC_USBCLK_HSI48: USB clock = HSI48
//  997   *             @arg RCC_USBCLK_PLLCLK: USB clock = PLL clock
//  998   * @retval None
//  999   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1000 void RCC_USBCLKConfig(uint32_t RCC_USBCLK)
// 1001 { 
// 1002   /* Check the parameters */
// 1003   assert_param(IS_RCC_USBCLK(RCC_USBCLK));
// 1004 
// 1005   /* Clear USBSW bit */
// 1006   RCC->CFGR3 &= ~RCC_CFGR3_USBSW;
RCC_USBCLKConfig:
        LDR      R1,??DataTable35  ;; 0x40021030
        LDR      R2,[R1, #+0]
        MOVS     R3,#+128
        BICS     R2,R2,R3
        STR      R2,[R1, #+0]
// 1007   /* Set USBSW bits according to RCC_USBCLK value */
// 1008   RCC->CFGR3 |= RCC_USBCLK;
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
// 1009 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24:
        DC32     0x40021000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_1:
        DC32     0xf8ffb80c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_2:
        DC32     0xfef6ffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_3:
        DC32     0xfffbffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_4:
        DC32     0xffc0ffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_5:
        DC32     0xfff0feac

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_6:
        DC32     0x40021002

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_7:
        DC32     0x40021034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_8:
        DC32     0x40021020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_9:
        DC32     0x40021004
// 1010 
// 1011 /**
// 1012   * @brief  Returns the frequencies of the System, AHB and APB busses clocks.
// 1013   * @note    The frequency returned by this function is not the real frequency
// 1014   *           in the chip. It is calculated based on the predefined constant and
// 1015   *           the source selected by RCC_SYSCLKConfig():
// 1016   *                                              
// 1017   * @note     If SYSCLK source is HSI, function returns constant HSI_VALUE(*)
// 1018   *                                              
// 1019   * @note     If SYSCLK source is HSE, function returns constant HSE_VALUE(**)
// 1020   *                          
// 1021   * @note     If SYSCLK source is PLL, function returns constant HSE_VALUE(**) 
// 1022   *             or HSI_VALUE(*) multiplied by the PLL factors.
// 1023   *               
// 1024   * @note     If SYSCLK source is HSI48, function returns constant HSI48_VALUE(***) 
// 1025   *             
// 1026   * @note     (*) HSI_VALUE is a constant defined in stm32f0xx.h file (default value
// 1027   *               8 MHz) but the real value may vary depending on the variations
// 1028   *               in voltage and temperature, refer to RCC_AdjustHSICalibrationValue().   
// 1029   *    
// 1030   * @note     (**) HSE_VALUE is a constant defined in stm32f0xx.h file (default value
// 1031   *                8 MHz), user has to ensure that HSE_VALUE is same as the real
// 1032   *                frequency of the crystal used. Otherwise, this function may
// 1033   *                return wrong result.
// 1034   *
// 1035   * @note     (***) HSI48_VALUE is a constant defined in stm32f0xx.h file (default value
// 1036   *                 48 MHz) but the real value may vary depending on the variations
// 1037   *                 in voltage and temperature.
// 1038   *                                   
// 1039   * @note   The result of this function could be not correct when using fractional
// 1040   *         value for HSE crystal.   
// 1041   *             
// 1042   * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold 
// 1043   *         the clocks frequencies. 
// 1044   *     
// 1045   * @note   This function can be used by the user application to compute the 
// 1046   *         baudrate for the communication peripherals or configure other parameters.
// 1047   * @note   Each time SYSCLK, HCLK and/or PCLK clock changes, this function
// 1048   *         must be called to update the structure's field. Otherwise, any
// 1049   *         configuration based on this function will be incorrect.
// 1050   *    
// 1051   * @retval None
// 1052   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1053 void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
// 1054 {
RCC_GetClocksFreq:
        PUSH     {R4-R6,LR}
        MOVS     R5,R0
// 1055   uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0, presc = 0, pllclk = 0;
        MOVS     R0,#+0
// 1056 
// 1057   /* Get SYSCLK source -------------------------------------------------------*/
// 1058   tmp = RCC->CFGR & RCC_CFGR_SWS;
        LDR      R6,??DataTable39  ;; 0x40021004
        LDR      R1,[R6, #+0]
// 1059   
// 1060   switch (tmp)
        MOVS     R2,#+12
        ANDS     R2,R2,R1
        CMP      R2,#+8
        BEQ      ??RCC_GetClocksFreq_0
        CMP      R2,#+12
        BEQ      ??RCC_GetClocksFreq_1
        B        ??RCC_GetClocksFreq_2
// 1061   {
// 1062     case 0x00:  /* HSI used as system clock */
// 1063       RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
// 1064       break;
// 1065     case 0x04:  /* HSE used as system clock */
// 1066       RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
// 1067       break;
// 1068     case 0x08:  /* PLL used as system clock */
// 1069       /* Get PLL clock source and multiplication factor ----------------------*/
// 1070       pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
??RCC_GetClocksFreq_0:
        LDR      R0,[R6, #+0]
// 1071       pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
        LDR      R2,[R6, #+0]
// 1072       pllmull = ( pllmull >> 18) + 2;
        MOVS     R1,#+15
        LSRS     R4,R0,#+18
        ANDS     R4,R4,R1
        ADDS     R4,R4,#+2
// 1073       
// 1074       if (pllsource == 0x00)
        MOVS     R0,#+192
        LSLS     R0,R0,#+9        ;; #+98304
        ANDS     R0,R0,R2
        BNE      ??RCC_GetClocksFreq_3
// 1075       {
// 1076         /* HSI oscillator clock divided by 2 selected as PLL clock entry */
// 1077         pllclk = (HSI_VALUE >> 1) * pllmull;
        MOVS     R0,R4
        LDR      R1,??DataTable39_1  ;; 0x3d0900
        MULS     R0,R1,R0
        B        ??RCC_GetClocksFreq_4
// 1078       }
// 1079       else
// 1080       {
// 1081         prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
??RCC_GetClocksFreq_3:
        LDR      R2,[R6, #+40]
// 1082         /* HSE oscillator clock selected as PREDIV1 clock entry */
// 1083         pllclk = (HSE_VALUE / prediv1factor) * pllmull; 
        LDR      R0,??DataTable39_2  ;; 0x7a1200
        ANDS     R1,R1,R2
        ADDS     R1,R1,#+1
        BL       __aeabi_uidiv
        MULS     R0,R4,R0
// 1084       }
// 1085       RCC_Clocks->SYSCLK_Frequency = pllclk;      
??RCC_GetClocksFreq_4:
        MOVS     R1,R0
// 1086       break;
        B        ??RCC_GetClocksFreq_5
// 1087     case 0x0C:  /* HSI48 used as system clock */
// 1088       RCC_Clocks->SYSCLK_Frequency = HSI48_VALUE;
??RCC_GetClocksFreq_1:
        LDR      R1,??DataTable39_3  ;; 0x2dc6c00
// 1089       break;
        B        ??RCC_GetClocksFreq_5
// 1090     default: /* HSI used as system clock */
// 1091       RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
??RCC_GetClocksFreq_2:
        LDR      R1,??DataTable39_2  ;; 0x7a1200
// 1092       break;
??RCC_GetClocksFreq_5:
        STR      R1,[R5, #+0]
// 1093   }
// 1094   /* Compute HCLK, PCLK clocks frequencies -----------------------------------*/
// 1095   /* Get HCLK prescaler */
// 1096   tmp = RCC->CFGR & RCC_CFGR_HPRE;
        LDR      R4,[R6, #+0]
// 1097   tmp = tmp >> 4;
// 1098   presc = APBAHBPrescTable[tmp]; 
// 1099   /* HCLK clock frequency */
// 1100   RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;
        LDR      R2,??DataTable39_4
        MOVS     R1,#+15
        LDR      R3,[R5, #+0]
        LSRS     R4,R4,#+4
        ANDS     R1,R1,R4
        LDRB     R1,[R2, R1]
        LSRS     R3,R3,R1
        STR      R3,[R5, #+4]
// 1101 
// 1102   /* Get PCLK prescaler */
// 1103   tmp = RCC->CFGR & RCC_CFGR_PPRE;
        LDR      R1,[R6, #+0]
// 1104   tmp = tmp >> 8;
// 1105   presc = APBAHBPrescTable[tmp];
// 1106   /* PCLK clock frequency */
// 1107   RCC_Clocks->PCLK_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
        LSLS     R1,R1,#+21
        LSRS     R1,R1,#+29
        LDRB     R1,[R2, R1]
        LSRS     R3,R3,R1
        STR      R3,[R5, #+8]
// 1108 
// 1109   /* ADCCLK clock frequency */
// 1110   if((RCC->CFGR3 & RCC_CFGR3_ADCSW) != RCC_CFGR3_ADCSW)
        LDR      R1,[R6, #+44]
        LSLS     R1,R1,#+23
        BMI      ??RCC_GetClocksFreq_6
// 1111   {
// 1112     /* ADC Clock is HSI14 Osc. */
// 1113     RCC_Clocks->ADCCLK_Frequency = HSI14_VALUE;
        LDR      R1,??DataTable39_5  ;; 0xd59f80
        B        ??RCC_GetClocksFreq_7
// 1114   }
// 1115   else
// 1116   {
// 1117     if((RCC->CFGR & RCC_CFGR_ADCPRE) != RCC_CFGR_ADCPRE)
??RCC_GetClocksFreq_6:
        LDR      R1,[R6, #+0]
        LSLS     R1,R1,#+17
        BMI      ??RCC_GetClocksFreq_8
// 1118     {
// 1119       /* ADC Clock is derived from PCLK/2 */
// 1120       RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK_Frequency >> 1;
        LSRS     R1,R3,#+1
        B        ??RCC_GetClocksFreq_7
// 1121     }
// 1122     else
// 1123     {
// 1124       /* ADC Clock is derived from PCLK/4 */
// 1125       RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK_Frequency >> 2;
??RCC_GetClocksFreq_8:
        LSRS     R1,R3,#+2
??RCC_GetClocksFreq_7:
        STR      R1,[R5, #+12]
// 1126     }
// 1127     
// 1128   }
// 1129 
// 1130   /* CECCLK clock frequency */
// 1131   if((RCC->CFGR3 & RCC_CFGR3_CECSW) != RCC_CFGR3_CECSW)
        LDR      R1,[R6, #+44]
        LSLS     R1,R1,#+25
        BMI      ??RCC_GetClocksFreq_9
// 1132   {
// 1133     /* CEC Clock is HSI/244 */
// 1134     RCC_Clocks->CECCLK_Frequency = HSI_VALUE / 244;
        LDR      R1,??DataTable39_6  ;; 0x8012
        B        ??RCC_GetClocksFreq_10
// 1135   }
// 1136   else
// 1137   {
// 1138     /* CECC Clock is LSE Osc. */
// 1139     RCC_Clocks->CECCLK_Frequency = LSE_VALUE;
??RCC_GetClocksFreq_9:
        MOVS     R1,#+128
        LSLS     R1,R1,#+8        ;; #+32768
??RCC_GetClocksFreq_10:
        STR      R1,[R5, #+16]
// 1140   }
// 1141 
// 1142   /* I2C1CLK clock frequency */
// 1143   if((RCC->CFGR3 & RCC_CFGR3_I2C1SW) != RCC_CFGR3_I2C1SW)
        LDR      R1,[R6, #+44]
        LSLS     R1,R1,#+27
        BMI      ??RCC_GetClocksFreq_11
// 1144   {
// 1145     /* I2C1 Clock is HSI Osc. */
// 1146     RCC_Clocks->I2C1CLK_Frequency = HSI_VALUE;
        LDR      R1,??DataTable39_2  ;; 0x7a1200
        B        ??RCC_GetClocksFreq_12
// 1147   }
// 1148   else
// 1149   {
// 1150     /* I2C1 Clock is System Clock */
// 1151     RCC_Clocks->I2C1CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
??RCC_GetClocksFreq_11:
        LDR      R1,[R5, #+0]
??RCC_GetClocksFreq_12:
        STR      R1,[R5, #+20]
// 1152   }
// 1153 
// 1154   /* USART1CLK clock frequency */
// 1155   if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == 0x0)
        MOVS     R1,#+3
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        BNE      ??RCC_GetClocksFreq_13
// 1156   {
// 1157     /* USART1 Clock is PCLK */
// 1158     RCC_Clocks->USART1CLK_Frequency = RCC_Clocks->PCLK_Frequency;
        LDR      R1,[R5, #+8]
        B        ??RCC_GetClocksFreq_14
// 1159   }
// 1160   else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW_0)
??RCC_GetClocksFreq_13:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        CMP      R2,#+1
        BNE      ??RCC_GetClocksFreq_15
// 1161   {
// 1162     /* USART1 Clock is System Clock */
// 1163     RCC_Clocks->USART1CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
        LDR      R1,[R5, #+0]
        B        ??RCC_GetClocksFreq_14
// 1164   }
// 1165   else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW_1)
??RCC_GetClocksFreq_15:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        CMP      R2,#+2
        BNE      ??RCC_GetClocksFreq_16
// 1166   {
// 1167     /* USART1 Clock is LSE Osc. */
// 1168     RCC_Clocks->USART1CLK_Frequency = LSE_VALUE;
        MOVS     R1,#+128
        LSLS     R1,R1,#+8        ;; #+32768
        B        ??RCC_GetClocksFreq_14
// 1169   }
// 1170   else if((RCC->CFGR3 & RCC_CFGR3_USART1SW) == RCC_CFGR3_USART1SW)
??RCC_GetClocksFreq_16:
        LDR      R2,[R6, #+44]
        ANDS     R1,R1,R2
        CMP      R1,#+3
        BNE      ??RCC_GetClocksFreq_17
// 1171   {
// 1172     /* USART1 Clock is HSI Osc. */
// 1173     RCC_Clocks->USART1CLK_Frequency = HSI_VALUE;
        LDR      R1,??DataTable39_2  ;; 0x7a1200
??RCC_GetClocksFreq_14:
        STR      R1,[R5, #+24]
// 1174   }
// 1175   
// 1176   /* USART2CLK clock frequency */
// 1177   if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == 0x0)
??RCC_GetClocksFreq_17:
        MOVS     R1,#+192
        LSLS     R1,R1,#+10       ;; #+196608
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        BNE      ??RCC_GetClocksFreq_18
// 1178   {
// 1179     /* USART Clock is PCLK */
// 1180     RCC_Clocks->USART2CLK_Frequency = RCC_Clocks->PCLK_Frequency;
        LDR      R1,[R5, #+8]
        B        ??RCC_GetClocksFreq_19
// 1181   }
// 1182   else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW_0)
??RCC_GetClocksFreq_18:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        MOVS     R3,#+128
        LSLS     R3,R3,#+9        ;; #+65536
        CMP      R2,R3
        BNE      ??RCC_GetClocksFreq_20
// 1183   {
// 1184     /* USART Clock is System Clock */
// 1185     RCC_Clocks->USART2CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
        LDR      R1,[R5, #+0]
        B        ??RCC_GetClocksFreq_19
// 1186   }
// 1187   else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW_1)
??RCC_GetClocksFreq_20:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        LSLS     R3,R3,#+1
        CMP      R2,R3
        BNE      ??RCC_GetClocksFreq_21
// 1188   {
// 1189     /* USART Clock is LSE Osc. */
// 1190     RCC_Clocks->USART2CLK_Frequency = LSE_VALUE;
        LSRS     R1,R3,#+2
        B        ??RCC_GetClocksFreq_19
// 1191   }
// 1192   else if((RCC->CFGR3 & RCC_CFGR3_USART2SW) == RCC_CFGR3_USART2SW)
??RCC_GetClocksFreq_21:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        CMP      R2,R1
        BNE      ??RCC_GetClocksFreq_22
// 1193   {
// 1194     /* USART Clock is HSI Osc. */
// 1195     RCC_Clocks->USART2CLK_Frequency = HSI_VALUE;
        LDR      R1,??DataTable39_2  ;; 0x7a1200
??RCC_GetClocksFreq_19:
        STR      R1,[R5, #+28]
// 1196   }
// 1197   
// 1198   /* USART3CLK clock frequency */
// 1199   if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == 0x0)
??RCC_GetClocksFreq_22:
        MOVS     R1,#+192
        LSLS     R1,R1,#+12       ;; #+786432
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        BNE      ??RCC_GetClocksFreq_23
// 1200   {
// 1201     /* USART Clock is PCLK */
// 1202     RCC_Clocks->USART3CLK_Frequency = RCC_Clocks->PCLK_Frequency;
        LDR      R1,[R5, #+8]
        B        ??RCC_GetClocksFreq_24
// 1203   }
// 1204   else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW_0)
??RCC_GetClocksFreq_23:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        MOVS     R3,#+128
        LSLS     R3,R3,#+11       ;; #+262144
        CMP      R2,R3
        BNE      ??RCC_GetClocksFreq_25
// 1205   {
// 1206     /* USART Clock is System Clock */
// 1207     RCC_Clocks->USART3CLK_Frequency = RCC_Clocks->SYSCLK_Frequency;
        LDR      R1,[R5, #+0]
        B        ??RCC_GetClocksFreq_24
// 1208   }
// 1209   else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW_1)
??RCC_GetClocksFreq_25:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        LSLS     R3,R3,#+1
        CMP      R2,R3
        BNE      ??RCC_GetClocksFreq_26
// 1210   {
// 1211     /* USART Clock is LSE Osc. */
// 1212     RCC_Clocks->USART3CLK_Frequency = LSE_VALUE;
        LSRS     R1,R3,#+4
        B        ??RCC_GetClocksFreq_24
// 1213   }
// 1214   else if((RCC->CFGR3 & RCC_CFGR3_USART3SW) == RCC_CFGR3_USART3SW)
??RCC_GetClocksFreq_26:
        LDR      R2,[R6, #+44]
        ANDS     R2,R2,R1
        CMP      R2,R1
        BNE      ??RCC_GetClocksFreq_27
// 1215   {
// 1216     /* USART Clock is HSI Osc. */
// 1217     RCC_Clocks->USART3CLK_Frequency = HSI_VALUE;
        LDR      R1,??DataTable39_2  ;; 0x7a1200
??RCC_GetClocksFreq_24:
        STR      R1,[R5, #+32]
// 1218   }
// 1219   
// 1220   /* USBCLK clock frequency */
// 1221   if((RCC->CFGR3 & RCC_CFGR3_USBSW) != RCC_CFGR3_USBSW)
??RCC_GetClocksFreq_27:
        LDR      R1,[R6, #+44]
        LSLS     R1,R1,#+24
        BMI      ??RCC_GetClocksFreq_28
// 1222   {
// 1223     /* USB Clock is HSI48 */
// 1224     RCC_Clocks->USBCLK_Frequency = HSI48_VALUE;
        LDR      R0,??DataTable39_3  ;; 0x2dc6c00
// 1225   }
// 1226   else
// 1227   {
// 1228     /* USB Clock is PLL clock */
// 1229     RCC_Clocks->USBCLK_Frequency = pllclk;
??RCC_GetClocksFreq_28:
        STR      R0,[R5, #+36]
// 1230   }   
// 1231 }
        POP      {R4-R6,PC}       ;; return
// 1232 
// 1233 /**
// 1234   * @}
// 1235   */
// 1236 
// 1237 /** @defgroup RCC_Group3 Peripheral clocks configuration functions
// 1238  *  @brief   Peripheral clocks configuration functions 
// 1239  *
// 1240 @verbatim
// 1241  ===============================================================================
// 1242              #####Peripheral clocks configuration functions #####
// 1243  ===============================================================================  
// 1244 
// 1245     [..] This section provide functions allowing to configure the Peripheral clocks. 
// 1246          (#) The RTC clock which is derived from the LSE, LSI or  HSE_Div32 (HSE
// 1247              divided by 32).
// 1248          (#) After restart from Reset or wakeup from STANDBY, all peripherals are off
// 1249              except internal SRAM, Flash and SWD. Before to start using a peripheral you
// 1250              have to enable its interface clock. You can do this using RCC_AHBPeriphClockCmd(),
// 1251              RCC_APB2PeriphClockCmd() and RCC_APB1PeriphClockCmd() functions.
// 1252          (#) To reset the peripherals configuration (to the default state after device reset)
// 1253              you can use RCC_AHBPeriphResetCmd(), RCC_APB2PeriphResetCmd() and 
// 1254              RCC_APB1PeriphResetCmd() functions.
// 1255 
// 1256 @endverbatim
// 1257   * @{
// 1258   */
// 1259 
// 1260 /**
// 1261   * @brief  Configures the RTC clock (RTCCLK).
// 1262   * @note   As the RTC clock configuration bits are in the Backup domain and write
// 1263   *         access is denied to this domain after reset, you have to enable write
// 1264   *         access using PWR_BackupAccessCmd(ENABLE) function before to configure
// 1265   *         the RTC clock source (to be done once after reset).    
// 1266   * @note   Once the RTC clock is configured it can't be changed unless the RTC
// 1267   *         is reset using RCC_BackupResetCmd function, or by a Power On Reset (POR)
// 1268   *             
// 1269   * @param  RCC_RTCCLKSource: specifies the RTC clock source.
// 1270   *          This parameter can be one of the following values:
// 1271   *            @arg RCC_RTCCLKSource_LSE: LSE selected as RTC clock
// 1272   *            @arg RCC_RTCCLKSource_LSI: LSI selected as RTC clock
// 1273   *            @arg RCC_RTCCLKSource_HSE_Div32: HSE divided by 32 selected as RTC clock
// 1274   *       
// 1275   * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
// 1276   *         work in STOP and STANDBY modes, and can be used as wakeup source.
// 1277   *         However, when the HSE clock is used as RTC clock source, the RTC
// 1278   *         cannot be used in STOP and STANDBY modes.
// 1279   *             
// 1280   * @note   The maximum input clock frequency for RTC is 2MHz (when using HSE as
// 1281   *         RTC clock source).
// 1282   *                          
// 1283   * @retval None
// 1284   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1285 void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource)
// 1286 {
// 1287   /* Check the parameters */
// 1288   assert_param(IS_RCC_RTCCLK_SOURCE(RCC_RTCCLKSource));
// 1289   
// 1290   /* Select the RTC clock source */
// 1291   RCC->BDCR |= RCC_RTCCLKSource;
RCC_RTCCLKConfig:
        LDR      R1,??DataTable39_7  ;; 0x40021020
        LDR      R2,[R1, #+0]
        B.N      ??Subroutine2_0
// 1292 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable26:
        DC32     0x40021024
// 1293 
// 1294 /**
// 1295   * @brief  Enables or disables the RTC clock.
// 1296   * @note   This function must be used only after the RTC clock source was selected
// 1297   *         using the RCC_RTCCLKConfig function.
// 1298   * @param  NewState: new state of the RTC clock.
// 1299   *          This parameter can be: ENABLE or DISABLE.
// 1300   * @retval None
// 1301   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1302 void RCC_RTCCLKCmd(FunctionalState NewState)
// 1303 {
// 1304   /* Check the parameters */
// 1305   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1306   
// 1307   if (NewState != DISABLE)
RCC_RTCCLKCmd:
        LDR      R1,??DataTable39_7  ;; 0x40021020
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??RCC_RTCCLKCmd_0
// 1308   {
// 1309     RCC->BDCR |= RCC_BDCR_RTCEN;
        MOVS     R2,#+128
        LSLS     R2,R2,#+8        ;; #+32768
        ORRS     R2,R2,R0
        B        ??RCC_RTCCLKCmd_1
// 1310   }
// 1311   else
// 1312   {
// 1313     RCC->BDCR &= ~RCC_BDCR_RTCEN;
??RCC_RTCCLKCmd_0:
        LDR      R2,??DataTable39_8  ;; 0xffff7fff
        ANDS     R2,R2,R0
??RCC_RTCCLKCmd_1:
        STR      R2,[R1, #+0]
// 1314   }
// 1315 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27:
        DC32     0xffc27fff
// 1316 
// 1317 /**
// 1318   * @brief  Forces or releases the Backup domain reset.
// 1319   * @note   This function resets the RTC peripheral (including the backup registers)
// 1320   *         and the RTC clock source selection in RCC_BDCR register.
// 1321   * @param  NewState: new state of the Backup domain reset.
// 1322   *          This parameter can be: ENABLE or DISABLE.
// 1323   * @retval None
// 1324   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1325 void RCC_BackupResetCmd(FunctionalState NewState)
// 1326 {
// 1327   /* Check the parameters */
// 1328   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1329   
// 1330   if (NewState != DISABLE)
RCC_BackupResetCmd:
        LDR      R1,??DataTable39_7  ;; 0x40021020
        REQUIRE ?Subroutine0
        ;; // Fall through to label ?Subroutine0
// 1331   {
// 1332     RCC->BDCR |= RCC_BDCR_BDRST;
// 1333   }
// 1334   else
// 1335   {
// 1336     RCC->BDCR &= ~RCC_BDCR_BDRST;
// 1337   }
// 1338 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine0:
        CMP      R0,#+0
        LDR      R0,[R1, #+0]
        BEQ      ??Subroutine0_0
        MOVS     R2,#+128
        LSLS     R2,R2,#+9        ;; #+65536
        ORRS     R2,R2,R0
        B        ??Subroutine0_1
??Subroutine0_0:
        LDR      R2,??DataTable39_9  ;; 0xfffeffff
        ANDS     R2,R2,R0
??Subroutine0_1:
        STR      R2,[R1, #+0]
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable28:
        DC32     0xfeffffff
// 1339 
// 1340 /**
// 1341   * @brief  Enables or disables the AHB peripheral clock.
// 1342   * @note   After reset, the peripheral clock (used for registers read/write access)
// 1343   *         is disabled and the application software has to enable this clock before 
// 1344   *         using it.    
// 1345   * @param  RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
// 1346   *          This parameter can be any combination of the following values:
// 1347   *             @arg RCC_AHBPeriph_GPIOA: GPIOA clock
// 1348   *             @arg RCC_AHBPeriph_GPIOB: GPIOB clock
// 1349   *             @arg RCC_AHBPeriph_GPIOC: GPIOC clock
// 1350   *             @arg RCC_AHBPeriph_GPIOD: GPIOD clock
// 1351   *             @arg RCC_AHBPeriph_GPIOE: GPIOE clock, applicable only for STM32F072 devices  
// 1352   *             @arg RCC_AHBPeriph_GPIOF: GPIOF clock
// 1353   *             @arg RCC_AHBPeriph_TS:    TS clock
// 1354   *             @arg RCC_AHBPeriph_CRC:   CRC clock
// 1355   *             @arg RCC_AHBPeriph_FLITF: (has effect only when the Flash memory is in power down mode)  
// 1356   *             @arg RCC_AHBPeriph_SRAM:  SRAM clock
// 1357   *             @arg RCC_AHBPeriph_DMA1:  DMA1 clock
// 1358   *             @arg RCC_AHBPeriph_DMA2:  DMA2 clock  
// 1359   * @param  NewState: new state of the specified peripheral clock.
// 1360   *          This parameter can be: ENABLE or DISABLE.
// 1361   * @retval None
// 1362   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1363 void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
// 1364 {
// 1365   /* Check the parameters */
// 1366   assert_param(IS_RCC_AHB_PERIPH(RCC_AHBPeriph));
// 1367   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1368   
// 1369   if (NewState != DISABLE)
RCC_AHBPeriphClockCmd:
        LDR      R2,??DataTable39_10  ;; 0x40021014
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_AHBPeriphClockCmd_0
// 1370   {
// 1371     RCC->AHBENR |= RCC_AHBPeriph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1372   }
// 1373   else
// 1374   {
// 1375     RCC->AHBENR &= ~RCC_AHBPeriph;
??RCC_AHBPeriphClockCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1376   }
// 1377 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable29:
        DC32     0x4002102c
// 1378 
// 1379 /**
// 1380   * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
// 1381   * @note   After reset, the peripheral clock (used for registers read/write access)
// 1382   *         is disabled and the application software has to enable this clock before 
// 1383   *         using it.
// 1384   * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
// 1385   *          This parameter can be any combination of the following values:
// 1386   *             @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
// 1387   *             @arg RCC_APB2Periph_USART6: USART6 clock  
// 1388   *             @arg RCC_APB2Periph_USART7: USART7 clock
// 1389   *             @arg RCC_APB2Periph_USART8: USART8 clock   
// 1390   *             @arg RCC_APB2Periph_ADC1:   ADC1 clock
// 1391   *             @arg RCC_APB2Periph_TIM1:   TIM1 clock
// 1392   *             @arg RCC_APB2Periph_SPI1:   SPI1 clock
// 1393   *             @arg RCC_APB2Periph_USART1: USART1 clock   
// 1394   *             @arg RCC_APB2Periph_TIM15:  TIM15 clock
// 1395   *             @arg RCC_APB2Periph_TIM16:  TIM16 clock
// 1396   *             @arg RCC_APB2Periph_TIM17:  TIM17 clock
// 1397   *             @arg RCC_APB2Periph_DBGMCU: DBGMCU clock
// 1398   * @param  NewState: new state of the specified peripheral clock.
// 1399   *          This parameter can be: ENABLE or DISABLE.
// 1400   * @retval None
// 1401   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1402 void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
// 1403 {
// 1404   /* Check the parameters */
// 1405   assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
// 1406   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1407 
// 1408   if (NewState != DISABLE)
RCC_APB2PeriphClockCmd:
        LDR      R2,??DataTable39_11  ;; 0x40021018
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_APB2PeriphClockCmd_0
// 1409   {
// 1410     RCC->APB2ENR |= RCC_APB2Periph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1411   }
// 1412   else
// 1413   {
// 1414     RCC->APB2ENR &= ~RCC_APB2Periph;
??RCC_APB2PeriphClockCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1415   }
// 1416 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable30:
        DC32     0xfff7ffff
// 1417 
// 1418 /**
// 1419   * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
// 1420   * @note   After reset, the peripheral clock (used for registers read/write access)
// 1421   *         is disabled and the application software has to enable this clock before 
// 1422   *         using it.
// 1423   * @param  RCC_APB1Periph: specifies the APB1 peripheral to gates its clock.
// 1424   *          This parameter can be any combination of the following values:
// 1425   *           @arg RCC_APB1Periph_TIM2:   TIM2 clock, applicable only for STM32F051 and STM32F072 devices
// 1426   *           @arg RCC_APB1Periph_TIM3:   TIM3 clock
// 1427   *           @arg RCC_APB1Periph_TIM6:   TIM6 clock
// 1428   *           @arg RCC_APB1Periph_TIM7:   TIM7 clock, applicable only for STM32F072 devices   
// 1429   *           @arg RCC_APB1Periph_TIM14:  TIM14 clock
// 1430   *           @arg RCC_APB1Periph_WWDG:   WWDG clock
// 1431   *           @arg RCC_APB1Periph_SPI2:   SPI2 clock
// 1432   *           @arg RCC_APB1Periph_USART2: USART2 clock
// 1433   *           @arg RCC_APB1Periph_USART3: USART3 clock, applicable only for STM32F072 and STM32F091 devices 
// 1434   *           @arg RCC_APB1Periph_USART4: USART4 clock, applicable only for STM32F072 and STM32F091 devices
// 1435   *           @arg RCC_APB1Periph_USART5: USART5 clock, applicable only for STM32F091 devices         
// 1436   *           @arg RCC_APB1Periph_I2C1:   I2C1 clock
// 1437   *           @arg RCC_APB1Periph_I2C2:   I2C2 clock
// 1438   *           @arg RCC_APB1Periph_USB:    USB clock, applicable only for STM32F042 and STM32F072 devices 
// 1439   *           @arg RCC_APB1Periph_CAN:    CAN clock, applicable only for STM32F042 and STM32F072 devices 
// 1440   *           @arg RCC_APB1Periph_CRS:    CRS clock , applicable only for STM32F042 and STM32F072 devices      
// 1441   *           @arg RCC_APB1Periph_PWR:    PWR clock
// 1442   *           @arg RCC_APB1Periph_DAC:    DAC clock, applicable only for STM32F051 and STM32F072 devices 
// 1443   *           @arg RCC_APB1Periph_CEC:    CEC clock, applicable only for STM32F051, STM32F042 and STM32F072 devices                               
// 1444   * @param  NewState: new state of the specified peripheral clock.
// 1445   *          This parameter can be: ENABLE or DISABLE.
// 1446   * @retval None
// 1447   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1448 void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
// 1449 {
// 1450   /* Check the parameters */
// 1451   assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
// 1452   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1453 
// 1454   if (NewState != DISABLE)
RCC_APB1PeriphClockCmd:
        LDR      R2,??DataTable39_12  ;; 0x4002101c
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_APB1PeriphClockCmd_0
// 1455   {
// 1456     RCC->APB1ENR |= RCC_APB1Periph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1457   }
// 1458   else
// 1459   {
// 1460     RCC->APB1ENR &= ~RCC_APB1Periph;
??RCC_APB1PeriphClockCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1461   }
// 1462 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable31:
        DC32     0x40021007
// 1463 
// 1464 /**
// 1465   * @brief  Forces or releases AHB peripheral reset.
// 1466   * @param  RCC_AHBPeriph: specifies the AHB peripheral to reset.
// 1467   *          This parameter can be any combination of the following values:
// 1468   *             @arg RCC_AHBPeriph_GPIOA: GPIOA clock
// 1469   *             @arg RCC_AHBPeriph_GPIOB: GPIOB clock
// 1470   *             @arg RCC_AHBPeriph_GPIOC: GPIOC clock
// 1471   *             @arg RCC_AHBPeriph_GPIOD: GPIOD clock
// 1472   *             @arg RCC_AHBPeriph_GPIOE: GPIOE clock, applicable only for STM32F072 devices  
// 1473   *             @arg RCC_AHBPeriph_GPIOF: GPIOF clock
// 1474   *             @arg RCC_AHBPeriph_TS:    TS clock
// 1475   * @param  NewState: new state of the specified peripheral reset.
// 1476   *          This parameter can be: ENABLE or DISABLE.
// 1477   * @retval None
// 1478   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1479 void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
// 1480 {
// 1481   /* Check the parameters */
// 1482   assert_param(IS_RCC_AHB_RST_PERIPH(RCC_AHBPeriph));
// 1483   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1484 
// 1485   if (NewState != DISABLE)
RCC_AHBPeriphResetCmd:
        LDR      R2,??DataTable39_13  ;; 0x40021028
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_AHBPeriphResetCmd_0
// 1486   {
// 1487     RCC->AHBRSTR |= RCC_AHBPeriph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1488   }
// 1489   else
// 1490   {
// 1491     RCC->AHBRSTR &= ~RCC_AHBPeriph;
??RCC_AHBPeriphResetCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1492   }
// 1493 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable32:
        DC32     0xffffbfff
// 1494 
// 1495 /**
// 1496   * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
// 1497   * @param  RCC_APB2Periph: specifies the APB2 peripheral to reset.
// 1498   *          This parameter can be any combination of the following values:
// 1499   *             @arg RCC_APB2Periph_SYSCFG: SYSCFG clock
// 1500   *             @arg RCC_APB2Periph_USART6: USART6 clock  
// 1501   *             @arg RCC_APB2Periph_USART7: USART7 clock
// 1502   *             @arg RCC_APB2Periph_USART8: USART8 clock   
// 1503   *             @arg RCC_APB2Periph_ADC1:   ADC1 clock
// 1504   *             @arg RCC_APB2Periph_TIM1:   TIM1 clock
// 1505   *             @arg RCC_APB2Periph_SPI1:   SPI1 clock
// 1506   *             @arg RCC_APB2Periph_USART1: USART1 clock
// 1507   *             @arg RCC_APB2Periph_TIM15:  TIM15 clock
// 1508   *             @arg RCC_APB2Periph_TIM16:  TIM16 clock
// 1509   *             @arg RCC_APB2Periph_TIM17:  TIM17 clock
// 1510   *             @arg RCC_APB2Periph_DBGMCU: DBGMCU clock
// 1511   * @param  NewState: new state of the specified peripheral reset.
// 1512   *          This parameter can be: ENABLE or DISABLE.
// 1513   * @retval None
// 1514   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1515 void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
// 1516 {
// 1517   /* Check the parameters */
// 1518   assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
// 1519   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1520 
// 1521   if (NewState != DISABLE)
RCC_APB2PeriphResetCmd:
        LDR      R2,??DataTable39_14  ;; 0x4002100c
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_APB2PeriphResetCmd_0
// 1522   {
// 1523     RCC->APB2RSTR |= RCC_APB2Periph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1524   }
// 1525   else
// 1526   {
// 1527     RCC->APB2RSTR &= ~RCC_APB2Periph;
??RCC_APB2PeriphResetCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1528   }
// 1529 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33:
        DC32     0xfffff8ff
// 1530 
// 1531 /**
// 1532   * @brief  Forces or releases Low Speed APB (APB1) peripheral reset.
// 1533   * @param  RCC_APB1Periph: specifies the APB1 peripheral to reset.
// 1534   *          This parameter can be any combination of the following values:
// 1535   *           @arg RCC_APB1Periph_TIM2:   TIM2 clock, applicable only for STM32F051 and STM32F072 devices
// 1536   *           @arg RCC_APB1Periph_TIM3:   TIM3 clock
// 1537   *           @arg RCC_APB1Periph_TIM6:   TIM6 clock
// 1538   *           @arg RCC_APB1Periph_TIM7:   TIM7 clock, applicable only for STM32F072 devices   
// 1539   *           @arg RCC_APB1Periph_TIM14:  TIM14 clock
// 1540   *           @arg RCC_APB1Periph_WWDG:   WWDG clock
// 1541   *           @arg RCC_APB1Periph_SPI2:   SPI2 clock
// 1542   *           @arg RCC_APB1Periph_USART2: USART2 clock
// 1543   *           @arg RCC_APB1Periph_USART3: USART3 clock, applicable only for STM32F072 and STM32F091 devices 
// 1544   *           @arg RCC_APB1Periph_USART4: USART4 clock, applicable only for STM32F072 and STM32F091 devices
// 1545   *           @arg RCC_APB1Periph_USART5: USART5 clock, applicable only for STM32F091 devices         
// 1546   *           @arg RCC_APB1Periph_I2C1:   I2C1 clock
// 1547   *           @arg RCC_APB1Periph_I2C2:   I2C2 clock
// 1548   *           @arg RCC_APB1Periph_USB:    USB clock, applicable only for STM32F042 and STM32F072 devices 
// 1549   *           @arg RCC_APB1Periph_CAN:    CAN clock, applicable only for STM32F042 and STM32F072 devices 
// 1550   *           @arg RCC_APB1Periph_CRS:    CRS clock , applicable only for STM32F042 and STM32F072 devices      
// 1551   *           @arg RCC_APB1Periph_PWR:    PWR clock
// 1552   *           @arg RCC_APB1Periph_DAC:    DAC clock, applicable only for STM32F051 and STM32F072 devices 
// 1553   *           @arg RCC_APB1Periph_CEC:    CEC clock, applicable only for STM32F051, STM32F042 and STM32F072 devices    
// 1554   * @param  NewState: new state of the specified peripheral clock.
// 1555   *          This parameter can be: ENABLE or DISABLE.
// 1556   * @retval None
// 1557   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1558 void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
// 1559 {
// 1560   /* Check the parameters */
// 1561   assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
// 1562   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1563 
// 1564   if (NewState != DISABLE)
RCC_APB1PeriphResetCmd:
        LDR      R2,??DataTable39_15  ;; 0x40021010
        CMP      R1,#+0
        LDR      R1,[R2, #+0]
        BEQ      ??RCC_APB1PeriphResetCmd_0
// 1565   {
// 1566     RCC->APB1RSTR |= RCC_APB1Periph;
        ORRS     R0,R0,R1
        STR      R0,[R2, #+0]
        BX       LR
// 1567   }
// 1568   else
// 1569   {
// 1570     RCC->APB1RSTR &= ~RCC_APB1Periph;
??RCC_APB1PeriphResetCmd_0:
        BICS     R1,R1,R0
        STR      R1,[R2, #+0]
// 1571   }
// 1572 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34:
        DC32     0xfffffeff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34_1:
        DC32     0x40021030
// 1573 
// 1574 /**
// 1575   * @}
// 1576   */
// 1577 
// 1578 /** @defgroup RCC_Group4 Interrupts and flags management functions
// 1579  *  @brief   Interrupts and flags management functions 
// 1580  *
// 1581 @verbatim
// 1582  ===============================================================================
// 1583              ##### Interrupts and flags management functions #####
// 1584  ===============================================================================
// 1585 @endverbatim
// 1586   * @{
// 1587   */
// 1588 
// 1589 /**
// 1590   * @brief  Enables or disables the specified RCC interrupts.
// 1591   * @note   The CSS interrupt doesn't have an enable bit; once the CSS is enabled
// 1592   *         and if the HSE clock fails, the CSS interrupt occurs and an NMI is
// 1593   *         automatically generated. The NMI will be executed indefinitely, and 
// 1594   *         since NMI has higher priority than any other IRQ (and main program)
// 1595   *         the application will be stacked in the NMI ISR unless the CSS interrupt
// 1596   *         pending bit is cleared.
// 1597   * @param  RCC_IT: specifies the RCC interrupt sources to be enabled or disabled.
// 1598   *          This parameter can be any combination of the following values:
// 1599   *              @arg RCC_IT_LSIRDY: LSI ready interrupt
// 1600   *              @arg RCC_IT_LSERDY: LSE ready interrupt
// 1601   *              @arg RCC_IT_HSIRDY: HSI ready interrupt
// 1602   *              @arg RCC_IT_HSERDY: HSE ready interrupt
// 1603   *              @arg RCC_IT_PLLRDY: PLL ready interrupt
// 1604   *              @arg RCC_IT_HSI14RDY: HSI14 ready interrupt
// 1605   *              @arg RCC_IT_HSI48RDY: HSI48 ready interrupt, applicable only for STM32F072 devices  
// 1606   * @param  NewState: new state of the specified RCC interrupts.
// 1607   *          This parameter can be: ENABLE or DISABLE.
// 1608   * @retval None
// 1609   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1610 void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState)
// 1611 {
// 1612   /* Check the parameters */
// 1613   assert_param(IS_RCC_IT(RCC_IT));
// 1614   assert_param(IS_FUNCTIONAL_STATE(NewState));
// 1615   
// 1616   if (NewState != DISABLE)
RCC_ITConfig:
        LDR      R2,??DataTable39_16  ;; 0x40021009
        CMP      R1,#+0
        LDRB     R1,[R2, #+0]
        BEQ      ??RCC_ITConfig_0
// 1617   {
// 1618     /* Perform Byte access to RCC_CIR[13:8] bits to enable the selected interrupts */
// 1619     *(__IO uint8_t *) CIR_BYTE1_ADDRESS |= RCC_IT;
        ORRS     R0,R0,R1
        STRB     R0,[R2, #+0]
        BX       LR
// 1620   }
// 1621   else
// 1622   {
// 1623     /* Perform Byte access to RCC_CIR[13:8] bits to disable the selected interrupts */
// 1624     *(__IO uint8_t *) CIR_BYTE1_ADDRESS &= (uint8_t)~RCC_IT;
??RCC_ITConfig_0:
        BICS     R1,R1,R0
        STRB     R1,[R2, #+0]
// 1625   }
// 1626 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable35:
        DC32     0x40021030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable35_1:
        DC32     0xfffcffff
// 1627 
// 1628 /**
// 1629   * @brief  Checks whether the specified RCC flag is set or not.
// 1630   * @param  RCC_FLAG: specifies the flag to check.
// 1631   *          This parameter can be one of the following values:
// 1632   *             @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready  
// 1633   *             @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
// 1634   *             @arg RCC_FLAG_PLLRDY: PLL clock ready
// 1635   *             @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
// 1636   *             @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
// 1637   *             @arg RCC_FLAG_OBLRST: Option Byte Loader (OBL) reset 
// 1638   *             @arg RCC_FLAG_PINRST: Pin reset
// 1639   *             @arg RCC_FLAG_V18PWRRSTF:  V1.8 power domain reset  
// 1640   *             @arg RCC_FLAG_PORRST: POR/PDR reset
// 1641   *             @arg RCC_FLAG_SFTRST: Software reset
// 1642   *             @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
// 1643   *             @arg RCC_FLAG_WWDGRST: Window Watchdog reset
// 1644   *             @arg RCC_FLAG_LPWRRST: Low Power reset
// 1645   *             @arg RCC_FLAG_HSI14RDY: HSI14 oscillator clock ready
// 1646   *             @arg RCC_FLAG_HSI48RDY: HSI48 oscillator clock ready, applicable only for STM32F072 devices    
// 1647   * @retval The new state of RCC_FLAG (SET or RESET).
// 1648   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1649 FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
// 1650 {
// 1651   uint32_t tmp = 0;
// 1652   uint32_t statusreg = 0;
// 1653   FlagStatus bitstatus = RESET;
// 1654 
// 1655   /* Check the parameters */
// 1656   assert_param(IS_RCC_FLAG(RCC_FLAG));
// 1657 
// 1658   /* Get the RCC register index */
// 1659   tmp = RCC_FLAG >> 5;
RCC_GetFlagStatus:
        LSRS     R1,R0,#+5
// 1660 
// 1661   if (tmp == 0)               /* The flag to check is in CR register */
        LDR      R2,??DataTable39_17  ;; 0x40021000
        CMP      R1,#+0
        BNE      ??RCC_GetFlagStatus_0
// 1662   {
// 1663     statusreg = RCC->CR;
        LDR      R1,[R2, #+0]
        B        ??RCC_GetFlagStatus_1
// 1664   }
// 1665   else if (tmp == 1)          /* The flag to check is in BDCR register */
??RCC_GetFlagStatus_0:
        CMP      R1,#+1
        BNE      ??RCC_GetFlagStatus_2
// 1666   {
// 1667     statusreg = RCC->BDCR;
        LDR      R1,[R2, #+32]
        B        ??RCC_GetFlagStatus_1
// 1668   }
// 1669   else if (tmp == 2)          /* The flag to check is in CSR register */
??RCC_GetFlagStatus_2:
        CMP      R1,#+2
        BNE      ??RCC_GetFlagStatus_3
// 1670   {
// 1671     statusreg = RCC->CSR;
        LDR      R1,[R2, #+36]
        B        ??RCC_GetFlagStatus_1
// 1672   }
// 1673   else                        /* The flag to check is in CR2 register */
// 1674   {
// 1675     statusreg = RCC->CR2;
??RCC_GetFlagStatus_3:
        LDR      R1,[R2, #+52]
// 1676   }    
// 1677 
// 1678   /* Get the flag position */
// 1679   tmp = RCC_FLAG & FLAG_MASK;
// 1680 
// 1681   if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
// 1682   {
// 1683     bitstatus = SET;
// 1684   }
// 1685   else
// 1686   {
// 1687     bitstatus = RESET;
// 1688   }
// 1689   /* Return the flag status */
// 1690   return bitstatus;
??RCC_GetFlagStatus_1:
        LSLS     R0,R0,#+27
        LSRS     R0,R0,#+27
        LSRS     R1,R1,R0
        LSLS     R0,R1,#+31
        LSRS     R0,R0,#+31
        BX       LR               ;; return
// 1691 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable36:
        DC32     0xfff3ffff
// 1692 
// 1693 /**
// 1694   * @brief  Clears the RCC reset flags.
// 1695   *         The reset flags are: RCC_FLAG_OBLRST, RCC_FLAG_PINRST, RCC_FLAG_V18PWRRSTF,
// 1696   *         RCC_FLAG_PORRST, RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST,
// 1697   *         RCC_FLAG_LPWRRST.
// 1698   * @param  None
// 1699   * @retval None
// 1700   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1701 void RCC_ClearFlag(void)
// 1702 {
// 1703   /* Set RMVF bit to clear the reset flags */
// 1704   RCC->CSR |= RCC_CSR_RMVF;
RCC_ClearFlag:
        LDR      R0,??DataTable39_18  ;; 0x40021024
        LDR      R1,[R0, #+0]
        MOVS     R2,#+128
        LSLS     R2,R2,#+17       ;; #+16777216
        ORRS     R2,R2,R1
        STR      R2,[R0, #+0]
// 1705 }
        BX       LR               ;; return
// 1706 
// 1707 /**
// 1708   * @brief  Checks whether the specified RCC interrupt has occurred or not.
// 1709   * @param  RCC_IT: specifies the RCC interrupt source to check.
// 1710   *          This parameter can be one of the following values:
// 1711   *             @arg RCC_IT_LSIRDY: LSI ready interrupt
// 1712   *             @arg RCC_IT_LSERDY: LSE ready interrupt
// 1713   *             @arg RCC_IT_HSIRDY: HSI ready interrupt
// 1714   *             @arg RCC_IT_HSERDY: HSE ready interrupt
// 1715   *             @arg RCC_IT_PLLRDY: PLL ready interrupt
// 1716   *             @arg RCC_IT_HSI14RDY: HSI14 ready interrupt
// 1717   *             @arg RCC_IT_HSI48RDY: HSI48 ready interrupt, applicable only for STM32F072 devices    
// 1718   *             @arg RCC_IT_CSS: Clock Security System interrupt
// 1719   * @retval The new state of RCC_IT (SET or RESET).
// 1720   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1721 ITStatus RCC_GetITStatus(uint8_t RCC_IT)
// 1722 {
RCC_GetITStatus:
        MOVS     R1,R0
// 1723   ITStatus bitstatus = RESET;
        MOVS     R0,#+0
// 1724   
// 1725   /* Check the parameters */
// 1726   assert_param(IS_RCC_GET_IT(RCC_IT));
// 1727   
// 1728   /* Check the status of the specified RCC interrupt */
// 1729   if ((RCC->CIR & RCC_IT) != (uint32_t)RESET)
        LDR      R2,??DataTable39_19  ;; 0x40021008
        LDR      R2,[R2, #+0]
        ANDS     R1,R1,R2
        BEQ      ??RCC_GetITStatus_0
// 1730   {
// 1731     bitstatus = SET;
        MOVS     R0,#+1
// 1732   }
// 1733   else
// 1734   {
// 1735     bitstatus = RESET;
// 1736   }
// 1737   /* Return the RCC_IT status */
// 1738   return  bitstatus;
??RCC_GetITStatus_0:
        BX       LR               ;; return
// 1739 }
// 1740 
// 1741 /**
// 1742   * @brief  Clears the RCC's interrupt pending bits.
// 1743   * @param  RCC_IT: specifies the interrupt pending bit to clear.
// 1744   *          This parameter can be any combination of the following values:
// 1745   *             @arg RCC_IT_LSIRDY: LSI ready interrupt
// 1746   *             @arg RCC_IT_LSERDY: LSE ready interrupt
// 1747   *             @arg RCC_IT_HSIRDY: HSI ready interrupt
// 1748   *             @arg RCC_IT_HSERDY: HSE ready interrupt
// 1749   *             @arg RCC_IT_PLLRDY: PLL ready interrupt
// 1750   *             @arg RCC_IT_HSI48RDY: HSI48 ready interrupt, applicable only for STM32F072 devices 
// 1751   *             @arg RCC_IT_HSI14RDY: HSI14 ready interrupt
// 1752   *             @arg RCC_IT_CSS: Clock Security System interrupt
// 1753   * @retval None
// 1754   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1755 void RCC_ClearITPendingBit(uint8_t RCC_IT)
// 1756 {
// 1757   /* Check the parameters */
// 1758   assert_param(IS_RCC_CLEAR_IT(RCC_IT));
// 1759   
// 1760   /* Perform Byte access to RCC_CIR[23:16] bits to clear the selected interrupt
// 1761      pending bits */
// 1762   *(__IO uint8_t *) CIR_BYTE2_ADDRESS = RCC_IT;
RCC_ClearITPendingBit:
        LDR      R1,??DataTable39_20  ;; 0x4002100a
        STRB     R0,[R1, #+0]
// 1763 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39:
        DC32     0x40021004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_1:
        DC32     0x3d0900

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_2:
        DC32     0x7a1200

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_3:
        DC32     0x2dc6c00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_4:
        DC32     APBAHBPrescTable

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_5:
        DC32     0xd59f80

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_6:
        DC32     0x8012

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_7:
        DC32     0x40021020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_8:
        DC32     0xffff7fff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_9:
        DC32     0xfffeffff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_10:
        DC32     0x40021014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_11:
        DC32     0x40021018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_12:
        DC32     0x4002101c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_13:
        DC32     0x40021028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_14:
        DC32     0x4002100c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_15:
        DC32     0x40021010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_16:
        DC32     0x40021009

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_17:
        DC32     0x40021000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_18:
        DC32     0x40021024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_19:
        DC32     0x40021008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_20:
        DC32     0x4002100a

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
// 1764 
// 1765 /**
// 1766   * @}
// 1767   */
// 1768 
// 1769 /**
// 1770   * @}
// 1771   */
// 1772 
// 1773 /**
// 1774   * @}
// 1775   */
// 1776 
// 1777 /**
// 1778   * @}
// 1779   */
// 1780 
// 1781 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
// 
//    16 bytes in section .data
// 1 314 bytes in section .text
// 
// 1 314 bytes of CODE memory
//    16 bytes of DATA memory
//
//Errors: none
//Warnings: none
