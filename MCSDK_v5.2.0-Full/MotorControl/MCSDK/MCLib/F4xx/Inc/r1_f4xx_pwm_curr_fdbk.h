/**
  ******************************************************************************
  * @file    r1_f4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_f4xx_pwm_curr_fdbk component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * @ingroup r1_f4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R1_F4XX_PWMNCURRFDBK_H
#define __R1_F4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r1_f4XX_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------- */
/**
  * @brief  R1_F4_Params_t structure definition
  */
typedef struct
{
  uint8_t  bFreqRatio;            /*!< It is used in case of dual MC to
                                       synchronize times. It must be equal
                                       to the ratio between the two PWM
                                       frequencies (higher/lower).
                                       Supported values are 1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;      /*!< When bFreqRatio is greather than 1
                                       this param is used to indicate if this
                                       instance is the one with the highest
                                       frequency. Allowed value are: HIGHER_FREQ
                                       or LOWER_FREQ */

  /* Current reading A/D Conversions initialization --------------------------*/
  ADC_TypeDef * ADCx_Inj ;         /*!< ADC Pperipheral used for phase current sampling */
 
  uint8_t hIChannel;              /*!< ADC channel used for conversion of
                                       current. It must be equal to
                                       ADC_CHANNEL_x x= 0, ..., 15*/

  /* PWM generation parameters --------------------------------------------------*/
  TIM_TypeDef * TIMx;              /*!< Timer used for PWM generation. It should be
                                       TIM1 or TIM8*/
  TIM_TypeDef * TIMx_2;          /*!< Auxiliary timer used for single shunt */

  uint16_t hDeadTime;             /*!< Dead time in number of TIM clock
                                       cycles. If CHxN are enabled, it must
                                       contain the dead time to be generated
                                       by the microcontroller, otherwise it
                                       expresses the maximum dead time
                                       generated by driving network*/
  uint8_t  bRepetitionCounter;    /*!< It expresses the number of PWM
                                       periods to be elapsed before compare
                                       registers are updated again. In
                                       particular:
                                       RepetitionCounter= (2* PWM periods) -1*/
  uint16_t hTafter;               /*!< It is the sum of dead time plus rise time
                                       express in number of TIM clocks.*/
  uint16_t hTbefore;              /*!< It is the value of sampling time
                                       expressed in numbers of TIM clocks.*/
  uint16_t hTMin;                 /*!< It is the sum of dead time plus rise time
                                       plus sampling time express in numbers of
                                       TIM clocks.*/
  uint16_t hHTMin;                /*!< It is the half of hTMin value.*/
  uint16_t hTSample;              /*!< It is the sampling time express in
                                       numbers of TIM clocks.*/
  uint16_t hMaxTrTs;              /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/

  /* PWM Driving signals initialization ----------------------------------------*/
  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/
  GPIO_TypeDef * pwm_en_u_port;
  uint32_t      pwm_en_u_pin;
  GPIO_TypeDef * pwm_en_v_port;
  uint32_t      pwm_en_v_pin;
  GPIO_TypeDef * pwm_en_w_port;
  uint32_t      pwm_en_w_pin;

  /* PWM Driving signals initialization ----------------------------------------*/
  FunctionalState EmergencyStop;  /*!< It enable/disable the management of
                                       an emergency input instantaneously
                                       stopping PWM generation. It must be
                                       either equal to ENABLE or DISABLE */
} R1_F4_Params_t;


/**
  * @brief  Public R1_F4XX class definition
  */
typedef struct
{
  PWMC_Handle_t _Super;       /*!< base component handler   */
  uint16_t Half_PWMPeriod;    /* Half PWM Period in timer clock counts */
  uint16_t hDmaBuff[2];       /*!< Buffer used for PWM distortion points*/
  uint32_t hCCDmaBuffCh4[4];  /*!< Buffer used for dual ADC sampling points*/
  uint16_t hCntSmp1;          /*!< First sampling point express in timer counts*/
  uint16_t hCntSmp2;          /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;           /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;           /*!< Current sampled in the second sampling point*/
  int16_t hCurrAOld;          /*!< Previous measured value of phase A current*/
  int16_t hCurrBOld;          /*!< Previous measured value of phase B current*/
  int16_t hCurrCOld;          /*!< Previous measured value of phase C current*/
  uint8_t bInverted_pwm;      /*!< This value indicates the type of the previous
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint8_t bInverted_pwm_new;  /*!< This value indicates the type of the current
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint16_t hPreloadCCMR2Set;  /*!< Preload value for TIMx->CCMR2 register used to
                                    set the mode of TIMx CH4*/
  uint8_t bDMATot;            /*!< Value to indicate the total number of expected
                                   DMA TC events*/
  uint8_t bDMACur;            /*!< Current number of DMA TC events occurred */
  uint16_t hFlags;            /*!< Flags
                                   EOFOC: Flag to indicate end of FOC duty available
                                   STBD3: Flag to indicate which phase has been distorted
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be
                                          performed or not (charge of bootstrap
                                          capacitor phase)
                                   SOFOC: This flag will be reset to zero at the begin of FOC
                                          and will be set in the UP IRQ. If at the end of
                                          FOC it is set the software error must be generated*/
  void * pDrive;               /*!< Pointer to drive object related to PWMnCurr object.
                                   It is set in the init function and returned by the MC TIMx update
                                   IRQ handler */
  uint32_t wPhaseOffset;      /*!< Offset of Phase current sensing network  */
  volatile uint8_t  bIndex;   /*!< Number of conversions performed during the
                                   calibration phase*/
  volatile uint32_t * pTIMx_2_CCR; /*!< Pointer to the used CCR of the auxiliary timer */
  bool OverCurrentFlag;     /*!< This flag is set when an overcurrent occurs.*/

  R1_F4_Params_t const * pParams_str;
} PWMC_R1_F4_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * It initializes TIM, ADC, GPIO, DMA and NVIC for single shunt current
  * reading configuration using STM32 F4XX family.
  */
void R1F4XX_Init( PWMC_R1_F4_Handle_t * pHandle );

/**
  * It stores into the component's handle the voltage present on the
  * current feedback analog channel when no current is flowin into the
  * motor
  */
void R1F4XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/**
  * It computes and return latest converted motor phase currents motor
  */
void R1F4XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R1F4XX_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit, disables the single shunt distortion and reset the TIM status
  */
void R1F4XX_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * This function enables the update event and the single shunt distortion
  */
void R1F4XX_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * Implementation of the single shunt algorithm to setup the
  * TIM1 register and DMA buffers values for the next PWM period.
  */
uint16_t R1F4XX_CalcDutyCycles( PWMC_Handle_t * pHdl );

/**
  * Execute a regular conversion.
  * The function is not re-entrant (can't executed twice at the same time)
  * It returns 0xFFFF in case of conversion error.
  */
uint16_t R1F4XX_ExecRegularConv( PWMC_Handle_t * pHdl, uint8_t bChannel );

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADC1. It must be called once for each channel utilized by user
  */
void R1F4XX_ADC_SetSamplingTime( PWMC_Handle_t * pHdl, ADConv_t ADConv_struct );

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t R1F4XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );

/**
  * It is used to set the PWM mode for R/L detection.
  */
void R1F4XX_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/**
  * It is used to disable the PWM mode for R/L detection.
  */
void R1F4XX_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/**
  * It is used to set the PWM dutycycle for R/L detection.
  */
uint16_t R1F4XX_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/**
  * It computes and return latest converted motor phase currents motor
  * during RL detection phase
  */
void R1F4XX_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers.
  * This function is specific for RL detection phase.
  */
void R1F4XX_RLTurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * It enables PWM generation on the proper Timer peripheral
  * This function is specific for RL detection phase.
  */
void R1F4XX_RLSwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit, disables the single shunt distortion and reset the TIM status
  */
void R1F4XX_RLSwitchOffPWM( PWMC_Handle_t * pHdl );

/**
 * @brief  It contains the TIM1 Update event interrupt
 */
void * R1F4XX_TIM1_UP_IRQHandler( PWMC_R1_F4_Handle_t * pHdl );

/**
 * @brief  It contains the TIM8 Update event interrupt
 */
void * R1F4XX_TIM8_UP_IRQHandler( PWMC_R1_F4_Handle_t * pHdl );

/**
 * @brief  It contains the Break event interrupt
 */
void * R1F4XX_BRK_IRQHandler( PWMC_R1_F4_Handle_t * pHdl );

/**
 * @brief  It contains the DMA transfer complete event interrupt
 */
void * R1F4XX_DMA_TC_IRQHandler( PWMC_R1_F4_Handle_t * pHdl );


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__R1_F4XX_PWMNCURRFDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
