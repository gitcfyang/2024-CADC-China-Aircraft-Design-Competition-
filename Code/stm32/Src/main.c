/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"
#include "pid.h"
#include "usbd_cdc_if.h"

pid_struct_t motor_pid_Single;//定义单个pid结构体
pid_Cascade_t motor_pid_Cas;//定义串级pid结构体

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t  CAN_ID;  //ID

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

typedef struct
{
    int16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  motor_temperature;
    int16_t  last_ecd;
} Motor_Info;
Motor_Info motor_info;

int16_t motor1Target = 4000;
int16_t current_target;
int32_t motor2Target = 0;

uint8_t direction_decide = 0;
uint32_t timer1 = 0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;

uint8_t oncerun1 = 1;

uint8_t receivebuff[11];
uint8_t transmitbuff[4];

uint16_t previous_position = 4000;
uint16_t current_position;
int8_t circle_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t canCRC_ATM(uint8_t *buf,uint8_t len);
void CanTransfer(uint8_t *buf,uint8_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis);
void Gohome(int16_t setv);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	pid_init(&motor_pid_Cas.inner, 1, 0.00001, 0, 3000, 3000);
    pid_init(&motor_pid_Cas.outer, 20, 0.02, 200, 3000, 3000);
	
    transmitbuff[0] = 0x55;
	transmitbuff[1] = 0x11;
	transmitbuff[3] = 0xFF;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  
  HAL_CAN_Start(&hcan1);
  
  //Activate the nofication
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  
  HAL_Delay(3000);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
	  if (HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6) == GPIO_PIN_RESET)
	  {
		  direction_decide = 0;//决定不抓
		  motor1Target = 4000;
		  motor2Target = 0;
		  
		  flag2 = 0;
		  oncerun1 = 1;
		  
		  if (flag1 == 0) // 如果还未开始计时
          {
              timer1 = HAL_GetTick(); // 记录当前时间
              flag1 = 1; // 设置标志为已计时
			  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);//爪子放开
			  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 800);
			  
          }
          else if (HAL_GetTick() - timer1 > 800) // 检查是否经过0.8秒
          {
              motor2Target = 0x10000;//应是计算出的250mm距离
			
          }
		  
		  
	  }else
	  {  
		  direction_decide = 1;//决定抓
		  flag1 = 0;

		  //以下东西仅运动一次（非阻塞模式下）
		  if (flag2 ==0)
		  {
			  timer1 = HAL_GetTick();
		      flag2 = 1;
		  }else if(HAL_GetTick() - timer1 <= 500)
		  {
			  if (oncerun1)
			  {
				  oncerun1 = 0;
			      int16_t ror = rand() % 360;
			      int16_t lenen = rand() % 250;
				
			      motor1Target = (int16_t)(ror * 22.7527f);
		          motor2Target = (int32_t)(lenen * 273.06667f);
			  }
			  //等待500ms
		  }else if (HAL_GetTick() - timer1 <= 1500)
		  {
			 
		  }else if (HAL_GetTick() - timer1 <= 2500)//延迟3000ms，开始舵机1运动
	      {
		      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1300);
		  }else if (HAL_GetTick() - timer1 <= 3300)//延迟1000ms，开始舵机2运动
		  {
			  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1900);
		  }else if (HAL_GetTick() - timer1 <= 4100)//延迟800ms，舵机1反转
		  {
			  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 800);
		  }else
		  {
			  motor2Target = 0;
		  }

		  
	  }
	  if (abs(motor_info.rotor_angle - motor1Target) > 200)
	  {
		  int16_t delta = motor_info.rotor_angle - motor1Target;
		  current_target = motor_info.rotor_angle + ((delta < 0) - (delta > 0))*200;
	  }else
	  {
		  current_target = motor1Target;
	  }
	  
	  int16_t set_voltage = pid_CascadeCalc(&motor_pid_Cas,current_target,motor_info.rotor_angle,motor_info.rotor_speed);
	  Gohome(set_voltage);
	  positionMode3Run(1,200,200,motor2Target);
	  
	  
	  if (HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7) == GPIO_PIN_RESET)
	  {
		  transmitbuff[2] = 0x00;
	  }else
	  {
		  transmitbuff[2] = 0x01;
	  }
	  CDC_Transmit_FS(transmitbuff, 4);
	  HAL_Delay(10);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//CAN发出标准帧
void CanTransfer(uint8_t *buf,uint8_t len)
{
    /* Init Transmit frame*/
    TxHeader.StdId   = CAN_ID; 					
    TxHeader.IDE     = CAN_ID_STD;  
    TxHeader.RTR     = CAN_RTR_DATA;    
    TxHeader.DLC     = len;  
		
	 memcpy(TxData,buf,len-1);		
	 TxData[len-1] = canCRC_ATM(buf,len-1);
	
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox);
}

//计算校验和
uint8_t canCRC_ATM(uint8_t *buf,uint8_t len) //CRC_SUM8
{
	uint32_t i;
	uint8_t check_sum;
	uint32_t sum = 0;
	
	for(i=0;i<len;i++)
	{
		sum += buf[i];
	}
	sum += CAN_ID;
	check_sum = sum & 0xFF;
	return check_sum;
}

/*
功能：串口发送位置模式3运行指令
输入：slaveAddr 从机地址
      speed     运行速度
      acc       加速度
      absAxis   绝对坐标
*/
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis)
{
    CAN_ID = slaveAddr;				//ID
	
  TxData[0] = 0xF5;       //功能码
  TxData[1] = (speed>>8)&0x00FF; //速度高8位
  TxData[2] = speed&0x00FF;     //速度低8位
  TxData[3] = acc;            //加速度
  TxData[4] = (absAxis >> 16)&0xFF;  //绝对坐标 bit23 - bit16
  TxData[5] = (absAxis >> 8)&0xFF;   //绝对坐标 bit15 - bit8
  TxData[6] = (absAxis >> 0)&0xFF;   //绝对坐标 bit7 - bit0
	
	CanTransfer(TxData,8);
}

//CAN接收数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	if(hcan->Instance==CAN1)
    {
        /*
          hcan
          CAN_RX_FIFO0:  [ sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  ]
          RxMessage:
          aData:the payload of the Rx frame will be stored
        */
        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxData)==HAL_OK)
        {

            motor_info.rotor_angle       = ((RxData[0] << 8) | RxData[1]);
            motor_info.rotor_speed       = ((RxData[2] << 8) | RxData[3]);
            motor_info.torque_current    = ((RxData[4] << 8) | RxData[5]);
            motor_info.motor_temperature =   RxData[6];
			
			current_position = motor_info.rotor_angle;
			
			int16_t delta = current_position - previous_position;
			
			if (delta > 4095)
			{
			    circle_count -= 1;
			}else if(delta < -4095)
			{
				circle_count += 1;
			}
			motor_info.rotor_angle = circle_count * 8191 + current_position;;
			
			previous_position = current_position;
            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // The FIFO0 receive interrupt function was enabled again
        }
    }
}


void Gohome(int16_t setv)
{
	CAN_ID = 0x1FF;
    TxData[0] = (setv>>8)&0x00FF; //速度高8位
    TxData[1] = setv&0x00FF;     //速度低8位
    TxData[2] = (setv>>8)&0x00FF; //速度高8位
    TxData[3] = setv&0x00FF;   
    TxData[4] = (setv>>8)&0x00FF;
    TxData[5] = setv&0x00FF;   
    TxData[6] = (setv>>8)&0x00FF;
    TxData[7] = setv&0x00FF; 
	CanTransfer(TxData,8);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
