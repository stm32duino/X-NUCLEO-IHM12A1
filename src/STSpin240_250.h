 /**
 ******************************************************************************
 * @file    STSpin240_250.h
 * @author  SRA
 * @version V1.0.0
 * @date    July 29th, 2019
 * @brief   This file contains the class of a STSpin240_250 Motor Control component.
 * @note    (C) COPYRIGHT 2019 STMicroelectronics
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __STSPIN240_250_CLASS_H
#define __STSPIN240_250_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/        
#include "Arduino.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/        
#include "STSpin240_250_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "BDCMotor.h"

/* Defines -------------------------------------------------------------------*/

/* MCU wait time in ms after power bridges are enabled */
#define BRIDGE_TURN_ON_DELAY    (1)

/* MCU wait time in ms after exit standby mode */
#define EXIT_STANDBY_MODE_DELAY    (1)

/* Classes -------------------------------------------------------------------*/


/**
 * @brief Class representing a STSpin240_250 component.
 */
class STSpin240_250 : public BDCMotor
{
public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_and_enable_pin   pin name of the EN pin of the component.
     * @param standby_reset_pin     pin name of the STBY\RST pin of the component.
     * @param dirA_pin              pin name for the direction pin for bridge A.
     * @param dirB_pin              pin name for the direction pin for bridge B.
     * @param pwmA_pin              pin name for the PWM input for bridge A.
     * @param pwmB_pin              pin name for the PWM input for bridge B.
     * @param pwmRef_pin            pin name for the REF pin of the component.
     */
    STSpin240_250(uint8_t flag_and_enable_pin, uint8_t standby_reset_pin, uint8_t dirA_pin, uint8_t dirB_pin, uint8_t pwmA_pin, uint8_t pwmB_pin, uint8_t pwmRef_pin) :
        BDCMotor(),
        flag_and_enable(flag_and_enable_pin),
        standby_reset(standby_reset_pin),
        dirA(dirA_pin),
        dirB(dirB_pin),
        pwmA(pwmA_pin),
        pwmB(pwmB_pin),
        pwmRef(pwmRef_pin)
    {
        TIM_TypeDef *pwmA_instance;
        TIM_TypeDef *pwmB_instance;
        TIM_TypeDef *pwmRef_instance;

        pinMode(flag_and_enable, OUTPUT);
        pinMode(standby_reset, OUTPUT);
        pinMode(dirA, OUTPUT);
        pinMode(dirB, OUTPUT);

        pwmA_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmA), PinMap_PWM);
        pwmA_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmA), PinMap_PWM));

        pwmB_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmB), PinMap_PWM);
        pwmB_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmB), PinMap_PWM));

        pwmRef_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmRef), PinMap_PWM);
        pwmRef_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmRef), PinMap_PWM));

        pwmA_timer = new HardwareTimer(pwmA_instance);
        pwmB_timer = new HardwareTimer(pwmB_instance);
        pwmRef_timer = new HardwareTimer(pwmRef_instance);

        errorHandlerCallback = 0;
        memset(&device_prm, 0, sizeof(deviceParams_t));
        deviceInstance = numberOfDevices++;
    }
    
    /**
     * @brief Destructor.
     */
    virtual ~STSpin240_250(void)
	{
       free(pwmA_timer);
       free(pwmB_timer);
       free(pwmRef_timer);
	}
    

    /*** Public Component Related Methods ***/

    /**
     * @brief Public functions inherited from the Component Class
     */

    /**
     * @brief  Initialize the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init = NULL)
    {
        return (int) STSpin240_250_Init((void *) init);
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int read_id(uint8_t *id = NULL)
    {
        return (int) STSpin240_250_ReadId((uint8_t *) id);
    }

    /**
     * @brief Public functions inherited from the BCDMotor Class
     */

    /**
     * @brief  Disabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval None.
     */
    virtual void disable_bridge(unsigned int bridgeId) 
    {
        STSpin240_250_DisableBridge(bridgeId);
    }
     
    /**
     * @brief  Enabling the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B
     * @retval None.
     */
    virtual void enable_bridge(unsigned int bridgeId)
    {
        STSpin240_250_EnableBridge(bridgeId);
    }
     
    /**
     * @brief  Getting the PWM frequency of the specified bridge;
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The frequency in Hz of the specified bridge input PWM.
     */
    virtual unsigned int get_bridge_input_pwm_freq(unsigned int bridgeId)
    {
        return (unsigned int) STSpin240_250_GetBridgeInputPwmFreq(bridgeId);
    }
    
    /**
     * @brief  Getting the bridge status.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @retval The status.
     */
    virtual unsigned int get_bridge_status(unsigned int bridgeId)
    {
        (void)bridgeId;
        return (unsigned int) STSpin240_250_GetBridgeStatus();
    }

    /**
     * @brief  Getting the device State. 
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1). 
     * @retval The device state (STEADY or INACTIVE)
     */
    virtual unsigned int get_device_state(unsigned int motorId)
    {
        return (motorState_t) STSpin240_250_GetDeviceState(motorId);
    }
   
    /**
     * @brief  Getting the current speed in % of the specified motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1). 
     * @retval The current speed in %.
     */
    virtual unsigned int get_speed(unsigned int motorId)
    {
        return (unsigned int) STSpin240_250_GetCurrentSpeed(motorId);
    }
    
    /**
     * @brief  Stopping the motor and disabling the power bridge immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1). 
     * @retval None.
     */
    virtual void hard_hiz(unsigned int motorId)
    {
       STSpin240_250_HardHiz(motorId);
    }
    
    /**
     * @brief  Stopping the motor immediately.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1). 
     * @retval None.
     */
    virtual void hard_stop(unsigned int motorId)
    {
       STSpin240_250_HardStop(motorId);
    }

    /**
     * @brief  Running the motor.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(unsigned int motorId, direction_t direction) 
    {
       STSpin240_250_Run(motorId, (motorDir_t) direction);
    }
    
    /**
     * @brief  Setting the PWM frequency of the specified bridge.
     * @param  bridgeId from 0 for bridge A to 1 for bridge B.
     * @param  frequency of the PWM in Hz
     * @retval None.
     */
    virtual void set_bridge_input_pwm_freq(unsigned int bridgeId, unsigned int frequency)
    {
        STSpin240_250_SetBridgeInputPwmFreq(bridgeId, frequency);
    }
        
    /**
     * @brief  Setting the dual bridge configuration mode.
     * @param  configuration. The bridge configuration.
     * @retval None.
     */
    virtual void set_dual_full_bridge_config(unsigned int configuration)
    {
        STSpin240_250_SetDualFullBridgeconfig(configuration);
    }

    /**
     * @brief  Setting the speed in %.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param  speed The new speed in %.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_speed(unsigned int motorId, unsigned int speed)
    {
        return (bool) STSpin240_250_SetMaxSpeed(motorId, speed);
    }

    /**
     * @brief Public functions NOT inherited
     */
     
    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void attach_error_handler(void (*fptr)(uint16_t error))
    {
        STSpin240_250_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }
    
    /**
     * @brief  Getting the motor current direction.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @retval direction The direction of rotation.
     */
    virtual direction_t get_direction(unsigned int motorId)
    {
        return (direction_t) STSpin240_250_GetDirection(motorId);
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual unsigned int get_fw_version(void)
    {
        return (unsigned int) STSpin240_250_GetFwVersion();
    }

    /**
     * @brief  Getting the duty cycle of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @retval duty cycle in % (from 0 to 100)
     */
    virtual unsigned int get_ref_pwm_dc(unsigned int refId)
    {
        return (unsigned int) STSpin240_250_GetRefPwmDc(refId);
    }
    
    /**
     * @brief  Getting the frequency of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @retval frequency in Hz.
     */
    virtual unsigned int get_ref_pwm_freq(unsigned int refId)
    {
        return (unsigned int) STSpin240_250_GetRefPwmFreq(refId);
    }
    
    /**
     * @brief  Releasing the reset (exiting standby mode).
     * @param  None.
     * @retval None.
     */
    virtual void release_reset(void)
    {
       STSpin240_250_ReleaseReset();
    }
    
    /**
     * @brief  Reseting (entering standby mode).
     * @param  None.
     * @retval None.
     */
    virtual void reset(void)
    {
       STSpin240_250_Reset();
    }
        
    /**
     * @brief  Setting the direction of rotation of the firmware.
     * @param  motorId from 0 to (MAX_NUMBER_OF_BRUSH_DC_MOTORS - 1).
     * @param direction The direction of rotation.
     * @retval None
     */
    virtual void set_direction(unsigned int motorId, direction_t direction)
    {
       STSpin240_250_SetDirection(motorId, (motorDir_t) direction);
    }

    /**
     * @brief  Setting the duty cycle of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @param  newDc new duty cycle from 0 to 100
     * @retval None.
     */
    virtual void set_ref_pwm_dc(unsigned int refId, unsigned int newDc)
    {
        STSpin240_250_SetRefPwmDc(refId, newDc);
    }
    
    /**
     * @brief  Setting the frequency of the PWM used for REF.
     * @param  refId Id of the reference PWM signal.
     * @param  frequency in Hz.
     * @retval None.
     */
    virtual void set_ref_pwm_freq(unsigned int refId, unsigned int frequency)
    {
        STSpin240_250_SetRefPwmFreq(refId, frequency);
    }

    /**
     * @brief Public static functions
     */    
    static uint8_t get_nb_devices(void)
    {
        return numberOfDevices;
    }

    /*** Public Interrupt Related Methods ***/

    /**
     * @brief  Attaching an interrupt handler to the FLAG interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_flag_irq(void (*fptr)(void))
    {
        int_cb = fptr;
    }
    
protected:

    /*** Protected Component Related Methods ***/
    
    status_t STSpin240_250_Init(void *init);
    status_t STSpin240_250_ReadId(uint8_t *id);
    void STSpin240_250_AttachErrorHandler(void (*callback)(uint16_t error));
    void STSpin240_250_DisableBridge(uint8_t bridgeId);
    void STSpin240_250_EnableBridge(uint8_t bridgeId);
    void STSpin240_250_ErrorHandler(uint16_t error);
    uint32_t STSpin240_250_GetBridgeInputPwmFreq(uint8_t bridgeId);
    uint16_t STSpin240_250_GetBridgeStatus(void);
    uint16_t STSpin240_250_GetCurrentSpeed(uint8_t motorId);
    motorState_t STSpin240_250_GetDeviceState(uint8_t motorId); 
    motorDir_t STSpin240_250_GetDirection(uint8_t motorId); 
    uint32_t STSpin240_250_GetFwVersion(void);  
    uint8_t STSpin240_250_GetRefPwmDc(uint8_t refId);    
    uint32_t STSpin240_250_GetRefPwmFreq(uint8_t refId);
    void STSpin240_250_HardHiz(uint8_t motorId); 
    void STSpin240_250_HardStop(uint8_t motorId); 
    void STSpin240_250_ReleaseReset(void);  
    void STSpin240_250_Reset(void);
    void STSpin240_250_Run(uint8_t motorId, motorDir_t direction); 
    void STSpin240_250_SetBridgeInputPwmFreq(uint8_t bridgeId, uint32_t newFreq); 
    void STSpin240_250_SetDirection(uint8_t motorId, motorDir_t dir); 
    void STSpin240_250_SetDualFullBridgeconfig(uint8_t enable);         
    bool STSpin240_250_SetMaxSpeed(uint8_t motorId,uint16_t newMaxSpeed);
    void STSpin240_250_SetRefPwmDc(uint8_t refId, uint8_t newDc);   
    void STSpin240_250_SetRefPwmFreq(uint8_t refId, uint32_t newFreq);  

    /**
     * @brief Functions to initialize the registers
     */
    void STSpin240_250_SetDeviceParamsToGivenValues(STSpin240_250_init_t *pInitPrm);
    void STSpin240_250_SetDeviceParamsToPredefinedValues(void);
    
    /**
     * @brief  Get the state of the standby/reset pin
     */
    uint8_t STSpin240_250_GetResetState(void);
   
    /*** Component's I/O Methods ***/

    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
    void STSpin240_250_Board_Delay(uint32_t ms_delay)
    {
        delay(ms_delay);
    }

    /**
     * @brief  Disable bridge.
     * @param  None.
     * @retval None.
     */  
    void STSpin240_250_Board_DisableBridge(void) 
    {
        detachInterrupt(flag_and_enable);
        pinMode(flag_and_enable, OUTPUT);
        digitalWrite(flag_and_enable, 0);
    }

    /**
     * @brief  Enable bridge.
     * @param  addDelay if different from 0, a delay is added after bridge activation..
     * @retval None.
     */  
    void STSpin240_250_Board_EnableBridge(uint8_t addDelay) 
    {
        pinMode(flag_and_enable, OUTPUT);
        digitalWrite(flag_and_enable, 1);
        if (addDelay)
        {
            STSpin240_250_Board_Delay(BRIDGE_TURN_ON_DELAY);
        }
        pinMode(flag_and_enable, INPUT);
        attachInterrupt(flag_and_enable, int_cb, FALLING);
    }  

    /** 
     * @brief  Get the status of the reset Pin.
     * @param  None.
     * @retval None.
     */  
    uint16_t STSpin240_250_Board_GetResetPinState(void) 
    {
        return((uint16_t)digitalRead(flag_and_enable));
    }

    /**
     * @brief  Get the status of the flag and enable Pin.
     * @param  None.
     * @retval None.
     */  
    uint16_t STSpin240_250_Board_GetFaultPinState(void) 
    {
        return((uint16_t)digitalRead(flag_and_enable));
    }
    
     /**
     * @brief  Deinitialising the PWM.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM.
     * @retval None.
     */
     void STSpin240_250_Board_PwmDeInit(uint8_t pwmId)
     {
         (void)pwmId;
     }
        
     /**
     * @brief  Initialising the PWM.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM.
     * @param  onlyChannel.
     * @retval None.
     */
    void STSpin240_250_Board_PwmInit(uint8_t pwmId, uint8_t onlyChannel) 
    {
         (void)pwmId;
         (void)onlyChannel;		 
    }

    /**
     * @brief  Setting the frequency of PWM.
     *         The frequency of bridge A and B controls directly the speed of the device.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM.
     * @param  newFreq frequency to apply in Hz.
     * @param  duty Duty cycle to use from 0 to 100.
     * @retval None.
     */
    void STSpin240_250_Board_PwmSetFreq(int8_t pwmId, uint32_t newFreq, uint8_t duty)
    {	
        switch (pwmId)
        {
            case 0:
            default:
                /* Setting the period and the duty-cycle of PWM A. */
                pwmA_timer->setPWM(pwmA_channel, pwmA, newFreq, duty);
            break;
            case 1:
                /* Setting the period and the duty-cycle of PWM B. */
                pwmB_timer->setPWM(pwmB_channel, pwmB, newFreq, duty);
            break;
            case 2:
                /* Setting the period and the duty-cycle of PWM Ref. */
                pwmRef_timer->setPWM(pwmRef_channel, pwmRef, newFreq, duty);
            break;           
        }
    }

    /**
     * @brief  Stopping the PWM.
     * @param  pwmId 0 for bridge A PWM, 1 for bridge B PWM, 2 for REF PWM.
     * @retval None.
     */
    void STSpin240_250_Board_PwmStop(uint8_t pwmId)
    {
        switch (pwmId)
        {
            case 0:
            default:
                pwmA_timer->setPWM(pwmA_channel, pwmA, device_prm.bridgePwmFreq[0], 0);
            break;
            case 1:
                pwmB_timer->setPWM(pwmB_channel, pwmB, device_prm.bridgePwmFreq[1], 0);
            break;
            case 2:
                pwmRef_timer->setPWM(pwmRef_channel, pwmRef, device_prm.refPwmFreq, 0);
            break;
        }
    }

    /**
     * @brief  Putting the device in standby mode.
     * @param  None.
     * @retval None.
     */
    void STSpin240_250_Board_ReleaseReset(void)
    {
        digitalWrite(standby_reset, 1);
        STSpin240_250_Board_Delay(EXIT_STANDBY_MODE_DELAY);
    }

    /**
     * @brief  Putting the device in reset mode.
     * @param  None.
     * @retval None.
     */
    void STSpin240_250_Board_Reset(void)
    {
        digitalWrite(standby_reset, 0);
    }

    /**
     * @brief  Setting the direction of rotation.
     * @param  bridgeId 0 for bridge A, 1 for bridge B.
     * @param  gpioState direction of rotation: "1" for forward, "0" for backward.
     * @retval None.
     */
    void STSpin240_250_Board_SetDirectionGpio(uint8_t bridgeId, uint8_t gpioState)
    {
      if (bridgeId == 0)
      {
        digitalWrite(dirA, gpioState);
      }
      else
      {
        digitalWrite(dirB, gpioState);
      }
    }

    /*** Component's Instance Variables ***/

    /* Flag Interrupt. */
    uint8_t flag_and_enable;
    void (*int_cb)(void);

    /* Standby/reset pin. */
    uint8_t standby_reset;

    /* Direction pin of bridge A. */
    uint8_t dirA;

    /* Direction pin of bridge B. */
    uint8_t dirB;

    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwmA_timer;
    uint32_t pwmA_channel;
    uint8_t pwmA;
    
    /* Pulse Width Modulation pin for bridge A input. */
    HardwareTimer *pwmB_timer;
    uint32_t pwmB_channel;
    uint8_t pwmB;

    /* Pulse Width Modulation pin for Ref signal. */
    HardwareTimer *pwmRef_timer;
    uint32_t pwmRef_channel;
    uint8_t pwmRef;

    /* ACTION 12 -------------------------------------------------------------*
     * Declare here identity related variables, if needed.                    *
     * Note that there should be only a unique identifier for each component, *
     * which should be the "who_am_i" parameter.                              *
     *------------------------------------------------------------------------*/
    /* Identity */
    uint8_t who_am_i;

    /* ACTION 13 -------------------------------------------------------------*
     * Declare here the component's static and non-static data, one variable  *
     * per line.                                                              *
     *                                                                        *
     * Example:                                                               *
     *   float measure;                                                       *
     *   int instance_id;                                                     *
     *   static int number_of_instances;                                      *
     *------------------------------------------------------------------------*/
    /* Data. */
    void (*errorHandlerCallback)(uint16_t error);
    deviceParams_t device_prm;
    uint8_t deviceInstance;

    
    /* Static data. */
    static uint8_t numberOfDevices;
    static uint8_t arrayNbMaxMotorsByConfig[2];
    
public:

    /* Static data. */
    
};

#endif // __STSPIN240_250_CLASS_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
