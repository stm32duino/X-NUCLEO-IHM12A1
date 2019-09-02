/**
 ******************************************************************************
 * @file    X_NUCLEO_IHM12A1_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 July 2019
 * @brief   Arduino test application for the STMicroelectronics X-NUCLEO-IHM12A1
 *          Motor Control Expansion Board: control of 2 DC Brush motors.
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

/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Component specific header files. */
#include "STSpin240_250.h"

/* Definitions ---------------------------------------------------------------*/

#define SerialPort Serial

/* Variables -----------------------------------------------------------------*/

/* Initialization parameters of the motor connected to the expansion board. */
STSpin240_250_init_t init_s =
{
  20000, /* Frequency of PWM of Input Bridge A in Hz up to 100000Hz             */
  20000, /* Frequency of PWM of Input Bridge B in Hz up to 100000Hz             */
  20000, /* Frequency of PWM used for Ref pin in Hz up to 100000Hz              */
  50,    /* Duty cycle of PWM used for Ref pin (from 0 to 100)                  */
  TRUE   /* Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc) */
};

/* Motor Control Component. */
STSpin240_250 *motor;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  /* Printing to the console. */
  char report[256];
  sprintf(report, "Error 0x%x detected\r\n\n", error);
  SerialPort.print(report);
  
  /* Infinite loop */
  while (true)
  {
  }    
}

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->attach_flag_irq(&my_flag_irq_handler);
 *           + motor->enable_flag_irq();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void my_flag_irq_handler(void)
{
  /* Code to be customised */
  /************************/

  SerialPort.print("    WARNING: \"FLAG\" interrupt triggered.\r\n");

  /* Get the state of bridge A */
  uint16_t bridgeState  = motor->get_bridge_status(0);
  
  if (bridgeState == 0)
  {
    if (motor->get_device_state(0) != INACTIVE || motor->get_device_state(1) != INACTIVE)
	{
      /* Bridges were disabled due to overcurrent or over temperature */
      /* When  motor was running */
      my_error_handler(0xBAD0);
    }
  }
}

/* setup ----------------------------------------------------------------------*/

void setup()
{
  /* Initialize Serial Port */
  SerialPort.begin(115200);

  /* Printing to the console. */
  SerialPort.println("STARTING DEMO PROGRAM");

  /* Initialization of the motor driver*/
  /* Please, be careful that pins can change if you change the Nucleo board */
  /* Give a look at the DataSheet of the Nucleo to find a correct configuration */
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A0);

  if (motor->init(&init_s) != COMPONENT_OK) exit(EXIT_FAILURE);

  /* Set dual bridge enabled as two motors are used*/
  motor->set_dual_full_bridge_config(1);

  /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);
    
  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);

  /* Printing to the console. */
  SerialPort.print("Motor Control Application Example for 2 brush DC motors\r\n");

  /* Set PWM Frequency of Ref to 15000 Hz */ 
  motor->set_ref_pwm_freq(0, 15000); 

  /* Set PWM duty cycle of Ref to 60% */ 
  motor->set_ref_pwm_dc(0, 60); 
  
  /* Set PWM Frequency of bridge A inputs to 10000 Hz */ 
  motor->set_bridge_input_pwm_freq(0,10000); 
  
  /* Set PWM Frequency of bridge B inputs to 10000 Hz */ 
  motor->set_bridge_input_pwm_freq(1,10000);
}

/* loop ----------------------------------------------------------------------*/

void loop()
{
  uint8_t demoStep = 0;

  while (demoStep <= 12)
  {
    switch (demoStep)
    {
      case 0:
      {
        SerialPort.print("STEP 0: Motor(0) FWD Speed=100%% - Motor(1) Inactive\r\n");
        /* Set speed of motor 0 to 100 % */
        motor->set_speed(0,100); 
        /* start motor 0 to run forward*/
        /* if chip is in standby mode */
        /* it is automatically awakened */
        motor->run(0, BDCMotor::FWD);
        break;
      }
      case 1:
      {
        SerialPort.print("STEP 1: Motor(0) FWD Speed=75%% - Motor(1) BWD Speed=100%%\r\n");
        /* Set speed of motor 0 to 75 % */
        motor->set_speed(0,75); 
        /* Set speed of motor 1 to 100 % */
        motor->set_speed(1,100); 
        /* start motor 1 to run backward */
        motor->run(1, BDCMotor::BWD);
        break;
      }
      case 2:
      {
        SerialPort.print("STEP 2: Motor(0) FWD Speed=50%% - Motor(1) BWD Speed=75%%\r\n");
        /* Set speed of motor 0 to 50 % */
        motor->set_speed(0,50);   
        /* Set speed of motor 1 to 75% */
        motor->set_speed(1,75); 
        break;
      }
      case 3:
      {
        SerialPort.print("STEP 3: Motor(0) FWD Speed=25%% - Motor(1) BWD Speed=50%%\r\n");
        /* Set speed of motor 0 to 25 % */
        motor->set_speed(0,25);  
        /* Set speed of motor 1 to 50% */
        motor->set_speed(1,50);          
        break;
      } 
      case 4:
      {
        SerialPort.print("STEP 4: Motor(0) Stopped - Motor(1) BWD Speed=25%%\r\n");
        /* Stop Motor 0 */
        motor->hard_stop(0);   
        /* Set speed of motor 1 to 25% */
        motor->set_speed(1,25);      
        break;
      }        
      case 5:
      {
        SerialPort.print("STEP 5: Motor(0) BWD Speed=25%% - Motor(1) Stopped\r\n");
        /* Set speed of motor 0 to 25 % */
        motor->set_speed(0,25); 
        /* start motor 0 to run backward */
        motor->run(0, BDCMotor::BWD);
        /* Stop Motor 1 */
        motor->hard_stop(1);   
        break;
      }
      case 6:
      {
        SerialPort.print("STEP 6: Motor(0) BWD Speed=50%% - Motor(1) FWD Speed=25%%\r\n");
        /* Set speed of motor 0 to 50 % */
        motor->set_speed(0,50); 
        /* Set speed of motor 1 to 25 % */
        motor->set_speed(1,25); 
        /* start motor 1 to run backward */
        motor->run(1, BDCMotor::FWD);
        break;
      }
      case 7:
      {
        SerialPort.print("STEP 7: Motor(0) BWD Speed=75%% - Motor(1) FWD Speed=50%%\r\n");
        /* Set speed of motor 0 to 75 % */
        motor->set_speed(0,75);   
        /* Set speed of motor 1 to 50 % */
        motor->set_speed(1,50);             
        break;
      }
      case 8:
      {
        SerialPort.print("STEP 8: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=75%%\r\n");
        /* Set speed of motor 0 to 100 % */
        motor->set_speed(0,100);   
        /* Set speed of motor 1 to 75 % */
        motor->set_speed(1,75);   
        break;
      } 
      case 9:
      {
        SerialPort.print("STEP 9: Motor(0) BWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
        /* Set speed of motor 1 to 100 % */
        motor->set_speed(1,100);    
        break;
      }  
      case 10:
      {
        SerialPort.print("STEP 10: Stop both motors and disable bridges\r\n");
        /* Stop both motors and disable bridge */
        motor->hard_hiz(0);    
        motor->hard_hiz(1);    
        break;
      }         
      case 11:
      {
        SerialPort.print("STEP 11: Motor(0) FWD Speed=100%% - Motor(1) FWD Speed=100%%\r\n");
        /* Start both motors to go forward*/
        motor->run(0,BDCMotor::FWD);    
        motor->run(1,BDCMotor::FWD);    
        break;
      }               
      case 12:
      default:
      {
        SerialPort.print("STEP 12: Stop both motors and enter standby mode\r\n");
        /* Stop both motors and put chip in standby mode */
        motor->reset();    
        break;
      }
    }
  
    /* Wait for 2 seconds */  
    delay(2000);

    /* Increment demostep*/  
    demoStep++;
  }
}
