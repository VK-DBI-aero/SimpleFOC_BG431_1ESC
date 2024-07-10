
#include <Arduino.h>
#include <SimpleFOC.h>
#include <string>
#include "stm32g4xx_hal.h"

float target = 0.0;
int spinIt = 1;
int spinItSpeed = 1500;
float radPerSec = 0;
int mxRads = 0;
int outIt = 0;

BLDCMotor motor = BLDCMotor(7, .12, 1400);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);


void serialControl(){
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      float num = received_chars.toFloat();
      if(received_chars == "s\n"){
        radPerSec = 0;
        mxRads = 0;
        received_chars = "";
        Serial.println("Stopping Motor.");
        break;
      }
      else if(received_chars == "g\n"){
        radPerSec = 0;
        mxRads = 300;
        received_chars = "";
        Serial.println("Starting Motor.");
        break;
      }
      else{
        target = num;
        Serial.print("PID.P = ");
        Serial.println(target);
        received_chars = "";
      }    
    }
  }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Initialize the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // Initialize the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO pins for motor phases as analog input
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2; // Example pins
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



void setup()
{

  //HAL_Init();
  //SystemClock_Config();
  MX_GPIO_Init();
  //MX_ADC1_Init();


  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);

  motor.voltage_limit = 14.8;
  motor.velocity_limit = 2212;

  motor.controller = MotionControlType::velocity_openloop;
  // set FOC modulation type to sinusoidal
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.PID_velocity.P = 0;
  motor.PID_velocity.I = 0;
  motor.LPF_velocity.Tf = 0;



  motor.init();

  motor.initFOC();

  
  Serial.begin(115200);
  delay(1000);
\
}


void loop()
{

  motor.loopFOC();
  motor.move(radPerSec);
  if (spinIt%spinItSpeed == 0 && radPerSec<mxRads){
    radPerSec++;
    spinIt = 1;
  }
  else{
    spinIt++;
  }

  motor.PID_velocity.P = target;
  
  serialControl();


}

