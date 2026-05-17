/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (终极指南针架构)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "bluetooth.h"
#include "MPU6050.h"    
#include "Gray_Sensor.h"   
#include "state_machine.h"
#include "vision.h"
#include "OLED.h"
#include <math.h>
#include <stdlib.h>
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

/* 全局变量：左右轮累计位置，由编码器模块更新 */
long long current_location_L = 0;
long long current_location_R = 0;

/* 全局变量：当前航向角，由 MPU6050 Z 轴角速度积分得到，范围保持在 -180° ~ 180° */
float current_yaw = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

static void App_ModuleInit(void);
static void App_UpdateYaw10ms(uint32_t dt_ms, int16_t *gz_now);
static void App_UpdateIdleGyroOffset(int16_t gz_now);
static void App_DebugOutput10ms(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Key_Init(void)
{
    /* 使能 GPIOE 时钟，因为 K1/K2/K3 都接在 GPIOE 上 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 定义 GPIO 初始化结构体，用于配置按键引脚 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PE0 是 K1，PE1 是 K2，PE2 是 K3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;

    /* 按键只需要读取电平，所以配置为输入模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    /* 按键默认上拉，按下时读取到低电平 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    /* 将配置写入 GPIOE */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static void OLED_ShowContestStatus(void)
{
    /* 定义左右识别字符，默认没有识别到左右 */
    char lor = '-';

    /* 如果 K230 回传目标在左边，OLED 显示 L */
    if (Vision_GetLoR() == VISION_LOR_LEFT) lor = 'L';

    /* 如果 K230 回传目标在右边，OLED 显示 R */
    else if (Vision_GetLoR() == VISION_LOR_RIGHT) lor = 'R';

    /* 清空显存，准备重新绘制状态页面 */
    OLED_Clear();

    /* 第一行显示当前模式、目标数字和左右识别结果 */
    OLED_Printf(0, 0, OLED_6X8, "M:%d T:%d %c", State_GetMode(), State_GetTargetDigit(), lor);

    /* 第二行显示 K230 当前数字、有效标志和置信度 */
    OLED_Printf(0, 12, OLED_6X8, "DIG:%d OK:%d C:%d", Vision_GetDigit(), Vision_GetFindedFlag(), Vision_GetConfidence());

    /* 第三行显示路线码、路口计数和视觉有效包数量 */
    OLED_Printf(0, 24, OLED_6X8, "R:%c I:%02d P:%lu", State_GetRouteCode(), State_GetIntersectionCount(), Vision_GetPacketCount());

    /* 第四行显示药品状态、启动确认和应用状态 */
    OLED_Printf(0, 36, OLED_6X8, "MED:%d GO:%d S:%d", State_GetMedicineState(), State_GetStartConfirmed(), State_GetAppState());

    /* 第五行显示视觉串口收到的字节数，并提示 K2 是启动确认键 */
    OLED_Printf(0, 48, OLED_6X8, "RX:%lu K2GO", Vision_GetRxByteCount());

    /* 将显存内容真正刷新到 OLED 屏幕 */
    OLED_Update();
}

static void App_ModuleInit(void)
{
    /* 初始化电机 PWM 和方向控制引脚 */
    Motor_Init();

    /* 初始化左右轮编码器，用于速度闭环 */
    Encoder_Init();

    /* 初始化蓝牙串口，并启动 USART2/USART3 中断接收 */
    Bluetooth_Init();

    /* 初始化 K230 视觉接收状态机 */
    Vision_Init();

    /* 初始化 K1/K2/K3 三个独立按键 */
    Key_Init();

    /* 初始化灰度传感器 ADC 校准 */
    Gray_Sensor_Init();

    /* 初始化 MPU6050 姿态传感器 */
    MPU6050_Init();

    /* 初始化软件 I2C OLED 显示屏 */
    OLED_Init();

    /* 初始化送药任务状态机和三个 PID 模块 */
    State_Machine_Init();

    /* 静止校准陀螺仪零漂，启动时小车必须保持不动 */
    MPU6050_Calibration();
}

static void App_UpdateYaw10ms(uint32_t dt_ms, int16_t *gz_now)
{
    /* 定义 MPU6050 六轴原始数据变量 */
    int16_t ax, ay, az, gx, gy, gz;

    /* 读取 MPU6050 当前一帧数据 */
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    /* 将当前 Z 轴角速度保存给外部零漂追踪函数使用 */
    *gz_now = gz;

    /* 减去校准得到的 Z 轴零漂 */
    float real_gz = (float)gz - gz_offset;

    /* 小角速度认为是静止噪声，直接归零，防止航向角慢慢漂移 */
    if (fabs(real_gz) < 2.0f) real_gz = 0.0f;

    /* 将本次循环间隔从毫秒转换成秒 */
    float dt = (float)dt_ms / 1000.0f;

    /* 陀螺仪比例系数，实车 90° 不准时可微调 */
    float gyro_scale = 1.0f;

    /* 根据 Z 轴角速度积分得到当前航向角 */
    current_yaw += (real_gz / 16.4f) * dt * gyro_scale;

    /* 将航向角限制在 -180° ~ 180°，防止跨界后角度 PID 判断错误 */
    while (current_yaw > 180.0f)  current_yaw -= 360.0f;
    while (current_yaw < -180.0f) current_yaw += 360.0f;
}

static void App_UpdateIdleGyroOffset(int16_t gz_now)
{
    /* 只有状态机空闲时才允许慢慢修正零漂，运行中不能改零点 */
    if (State_IsIdle()) {
        /* 如果原始角速度接近当前零漂，说明小车大概率静止 */
        if (gz_now > (gz_offset - 15) && gz_now < (gz_offset + 15)) {
            /* 用很小比例更新零漂，避免一次噪声造成零点突变 */
            gz_offset = gz_offset * 0.995f + (float)gz_now * 0.005f;
        }
    }
}

static void App_DebugOutput10ms(void)
{
    /* 蓝牙调试分频计数器，50 次 10ms 约等于 500ms */
    static uint8_t debug_cnt = 0;

    /* OLED 显示分频计数器，100 次 10ms 约等于 1s */
    static uint8_t oled_cnt = 0;

    /* 每 500ms 发送一次蓝牙遥测和灰度 ADC 数据 */
    if (++debug_cnt >= 50) {
        /* 清零计数器，准备下一次计时 */
        debug_cnt = 0;

        /* 读取当前灰度误差，同时会刷新 Gray_Value 数组 */
        float current_err = Get_Grayscale_Error();

        /* 空闲显示 0，运行显示 1，方便手机端快速判断状态 */
        uint8_t state_num = State_IsIdle() ? 0 : 1;

        /* 通过蓝牙发送状态、航向角和灰度误差 */
        Bluetooth_SendTelemetry(state_num, current_yaw, current_err);

        /* 通过蓝牙发送 8 路灰度 ADC 原始值 */
        Bluetooth_SendGrayData(Gray_Value);
    }

    /* 每 1s 刷新一次 OLED，避免软件 I2C 占用太多主循环时间 */
    if (++oled_cnt >= 100) {
        /* 清零计数器，准备下一次计时 */
        oled_cnt = 0;

        /* 刷新 OLED 上的比赛状态信息 */
        OLED_ShowContestStatus();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* 初始化电机、编码器、蓝牙、视觉、按键、灰度、陀螺仪、OLED 和状态机 */
  App_ModuleInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* 记录上一次执行 10ms 任务的系统时间 */
  uint32_t last_tick = HAL_GetTick();

  /* 保存本次读取到的 Z 轴原始角速度，用于空闲零漂追踪 */
  int16_t gz_now = 0;

  while (1)
  {
      /* 持续处理蓝牙命令，避免接收缓存长时间不解析 */
      Bluetooth_Loop();

      /* 读取当前系统毫秒计数 */
      uint32_t current_tick = HAL_GetTick();

      /* 计算距离上一次 10ms 任务已经过去的时间 */
      uint32_t dt_ms = current_tick - last_tick;

      /* 每 10ms 执行一次小车核心控制任务 */
      if (dt_ms >= 10)
      {
          /* 更新时间基准，下一轮从当前时刻重新计时 */
          last_tick = current_tick;

          /* 读取陀螺仪并积分当前航向角 */
          App_UpdateYaw10ms(dt_ms, &gz_now);

          /* 扫描 K1/K2/K3 按键下降沿事件 */
          State_Machine_ScanKeys10ms();

          /* 周期向 K230 发送当前视觉任务 */
          Vision_Loop10ms();

          /* 执行送药状态机，并在内部执行速度 PID */
          State_Machine_Run();

          /* 小车空闲时缓慢修正陀螺仪零漂 */
          App_UpdateIdleGyroOffset(gz_now);

          /* 周期输出蓝牙调试数据并刷新 OLED */
          App_DebugOutput10ms();
      }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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