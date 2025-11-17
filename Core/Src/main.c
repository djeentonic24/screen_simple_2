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
#include "cmsis_os.h"
#include "FreeRTOS.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "LCDcontroller.h"
#include "task.h"
#include "lfs.h"
#include "nor.h"
#include "ymodem.h"

LV_IMG_DECLARE(picture);
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId TaskDrawHandle;
osThreadId TaskDCDCHandle;
osThreadId TaskLittleFSHandle;
/* USER CODE BEGIN PV */

// Logging variables
uint8_t logging_enabled = 0;
uint32_t log_interval_ms = 1000; // Log every 1 second
uint32_t last_log_time = 0;
uint32_t log_sequence = 0;

// Переменные системы управления DCDC
float Vout = 0.0f;              // Выходное напряжение
volatile float Device_temp = 25.0f; // Температура устройства
volatile float Vref = 5.0f;     // Опорное напряжение
float control = 0.0;            // Управляющий сигнал ПИ-регулятора
float duty_last = 0.0f;         // Последнее значение скважности ШИМ
uint16_t log_index_dc = 0;      // Индекс для логирования
PI_Controller pi;               // Структура ПИ-регулятора

// Файловая система LittleFS и NOR Flash
nor_t nor_flash;                // Структура NOR Flash
lfs_t lfs;                      // Структура LittleFS
lfs_file_t file;                // Файловый дескриптор
struct lfs_config cfg;          // Конфигурация LittleFS

// Буферы для LittleFS
uint8_t lfs_read_buffer[256];
uint8_t lfs_prog_buffer[256];
uint8_t lfs_lookahead_buffer[128];

// Прототипы функций для работы с NOR Flash
void nor_cs_assert(void);
void nor_cs_deassert(void);
void nor_spi_tx(uint8_t *txBuf, uint32_t size);
void nor_spi_rx(uint8_t *rxBuf, uint32_t size);
void nor_delay_us(uint32_t us);
void littlefs_config_init(void);
nor_err_e nor_init(void);
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void Start_TaskDraw(void const * argument);
void Start_TaskDCDC(void const * argument);
void Start_TaskLittleFS(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE BEGIN PFP */
void start_logging(void);
void stop_logging(void);
void log_data_point(void);
void create_log_file(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Инициализация DWT счётчика для точных задержек
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // Инициализация ШИМ для управления DCDC преобразователем
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskDraw */
  osThreadDef(TaskDraw, Start_TaskDraw, osPriorityLow, 0, 1024);
  TaskDrawHandle = osThreadCreate(osThread(TaskDraw), NULL);

  /* definition and creation of TaskDCDC */
  osThreadDef(TaskDCDC, Start_TaskDCDC, osPriorityHigh, 0, 256);
  TaskDCDCHandle = osThreadCreate(osThread(TaskDCDC), NULL);

  /* definition and creation of TaskLittleFS */
  osThreadDef(TaskLittleFS, Start_TaskLittleFS, osPriorityAboveNormal, 0, 1024);
  TaskLittleFSHandle = osThreadCreate(osThread(TaskLittleFS), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Check filesystem health
  */
void check_filesystem_health(void) {
    char msg[100];

    // Проверяем корневую директорию
    lfs_dir_t dir;
    if (lfs_dir_open(&lfs, &dir, "/") == LFS_ERR_OK) {
        snprintf(msg, sizeof(msg), "Root directory OK\r\n");
        lfs_dir_close(&lfs, &dir);
    } else {
        snprintf(msg, sizeof(msg), "Root directory FAILED\r\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Проверяем директорию /data
    if (lfs_dir_open(&lfs, &dir, "/data") == LFS_ERR_OK) {
        snprintf(msg, sizeof(msg), "/data directory OK\r\n");
        lfs_dir_close(&lfs, &dir);
    } else {
        snprintf(msg, sizeof(msg), "/data directory FAILED, creating...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        lfs_mkdir(&lfs, "/data");
    }
}

/**
  * @brief Start data logging to CSV file
  */
void start_logging(void) {
    logging_enabled = 1;
    log_sequence = 0;

    // Create new log file with timestamp
    create_log_file();

    // Write CSV header
    lfs_file_t log_file;
    if (lfs_file_open(&lfs, &log_file, "/data/datalog.csv", LFS_O_WRONLY | LFS_O_APPEND) == LFS_ERR_OK) {
        const char *header = "Timestamp(ms),Sequence,Vref(V),Vout(V),Temp(C),Duty(%),Control\r\n";
        lfs_file_write(&lfs, &log_file, header, strlen(header));
        lfs_file_close(&lfs, &log_file);
    }

    char msg[100];
    snprintf(msg, sizeof(msg), "Logging STARTED: /data/datalog.csv\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief Stop data logging
  */
void stop_logging(void) {
    logging_enabled = 0;
    HAL_UART_Transmit(&huart2, (uint8_t*)"Logging STOPPED\r\n", 17, HAL_MAX_DELAY);
}

/**
  * @brief Create new log file with timestamp
  */
void create_log_file(void) {
    char filename[50] = "/data/datalog.csv";

    // Сначала убедимся, что директория существует
    lfs_mkdir(&lfs, "/data");

    // Создаем файл и сразу пишем заголовок
    lfs_file_t log_file;
    int ret = lfs_file_open(&lfs, &log_file, filename, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (ret == LFS_ERR_OK) {
        const char *header = "Timestamp(ms),Sequence,Vref(V),Vout(V),Temp(C),Duty(%),Control\r\n";
        lfs_file_write(&lfs, &log_file, header, strlen(header));
        lfs_file_close(&lfs, &log_file);

        char msg[100];
        snprintf(msg, sizeof(msg), "Log file created: %s\r\n", filename);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        char msg[100];
        snprintf(msg, sizeof(msg), "ERROR creating log file: %d\r\n", ret);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

/**
  * @brief Log one data point to CSV
  */
void log_data_point(void) {
    if (!logging_enabled) return;

    char filename[50] = "/data/datalog.csv";

    lfs_file_t log_file;
    int ret = lfs_file_open(&lfs, &log_file, filename, LFS_O_WRONLY | LFS_O_APPEND | LFS_O_CREAT);
    if (ret != LFS_ERR_OK) {
        char msg[100];
        snprintf(msg, sizeof(msg), "ERROR opening log file: %d, recreating...\r\n", ret);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        // Пытаемся пересоздать файл
        create_log_file();
        return;
    }

    char log_line[256];
    float duty_percent = (duty_last / 9000.0f) * 100.0f;

    snprintf(log_line, sizeof(log_line), "%lu,%lu,%.3f,%.3f,%.2f,%.1f,%.3f\r\n",
             HAL_GetTick(),
             log_sequence,
             Vref,
             Vout,
             Device_temp,
             duty_percent,
             control);

    ret = lfs_file_write(&lfs, &log_file, log_line, strlen(log_line));
    if (ret < 0) {
        char msg[100];
        snprintf(msg, sizeof(msg), "ERROR writing log: %d\r\n", ret);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // Важно: sync перед закрытием!
    lfs_file_sync(&lfs, &log_file);
    lfs_file_close(&lfs, &log_file);

    // Отладочный вывод
    if (log_sequence % 5 == 0) {
        char msg[100];
        snprintf(msg, sizeof(msg), "Logged #%lu: Vout=%.2f\r\n", log_sequence, Vout);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    log_sequence++;
}

/**
  * @brief Send log file via XMODEM
  */
void send_log_file_xmodem(void) {
    char msg[100];
    lfs_file_t file;

    // Check if log file exists
    if (lfs_file_open(&lfs, &file, "/data/datalog.csv", LFS_O_RDONLY) != LFS_ERR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"No log file found\r\n", 19, HAL_MAX_DELAY);
        return;
    }

    uint32_t file_size = lfs_file_size(&lfs, &file);
    lfs_file_close(&lfs, &file);

    if (file_size == 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Log file is empty\r\n", 19, HAL_MAX_DELAY);
        return;
    }

    snprintf(msg, sizeof(msg), "Log file size: %lu bytes\r\n", file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Send via XMODEM
    send_file_xmodem("/data/datalog.csv");
}


static int load_file_into_ram(const char *path, uint8_t **buffer, uint32_t *file_size) {
    lfs_file_t file;
    char msg[100];

    if (lfs_file_open(&lfs, &file, path, LFS_O_RDONLY) != LFS_ERR_OK) {
        snprintf(msg, sizeof(msg), "YMODEM: cannot open %s\r\n", path);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return -1;
    }

    lfs_soff_t size = lfs_file_size(&lfs, &file);
    if (size <= 0) {
        snprintf(msg, sizeof(msg), "YMODEM: empty file %s\r\n", path);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        lfs_file_close(&lfs, &file);
        return -2;
    }

    uint8_t *ram = pvPortMalloc(size);
    if (ram == NULL) {
        snprintf(msg, sizeof(msg), "YMODEM: no RAM for %s (%ld bytes)\r\n", path, (long)size);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        lfs_file_close(&lfs, &file);
        return -3;
    }

    lfs_file_rewind(&lfs, &file);
    int read = lfs_file_read(&lfs, &file, ram, size);
    lfs_file_close(&lfs, &file);

    if (read < 0 || (lfs_soff_t)read != size) {
        snprintf(msg, sizeof(msg), "YMODEM: read error %s (%d)\r\n", path, read);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vPortFree(ram);
        return -4;
    }

    *buffer = ram;
    *file_size = (uint32_t)size;
    return 0;
}

static int send_file_with_ymodem(const char *path) {
    uint8_t *buffer = NULL;
    uint32_t file_size = 0;
    char msg[128];

    if (load_file_into_ram(path, &buffer, &file_size) != 0) {
        return 0;
    }

    const char *name_only = strrchr(path, '/');
    if (name_only && *(name_only + 1) != '\0') {
        name_only++;
    } else {
        name_only = path;
    }

    snprintf(msg, sizeof(msg), "YMODEM send: %s (%lu bytes)\r\n", name_only, file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    COM_StatusTypeDef status = Ymodem_Transmit(buffer, (const uint8_t*)name_only, file_size);
    vPortFree(buffer);

    if (status == COM_OK) {
        snprintf(msg, sizeof(msg), "YMODEM OK: %s\r\n", name_only);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 1;
    } else {
        snprintf(msg, sizeof(msg), "YMODEM FAIL (%d): %s\r\n", status, name_only);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 0;
    }
}

static int send_directory_with_ymodem(const char *dir_path) {
    lfs_dir_t dir;
    struct lfs_info info;
    int sent = 0;

    if (lfs_dir_open(&lfs, &dir, dir_path) != LFS_ERR_OK) {
        char msg[80];
        snprintf(msg, sizeof(msg), "YMODEM: cannot open %s\r\n", dir_path);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return 0;
    }

    while (lfs_dir_read(&lfs, &dir, &info) > 0) {
        if (info.type != LFS_TYPE_REG) {
            continue;
        }
        if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0) {
            continue;
        }

        char filepath[64];
        if (strcmp(dir_path, "/") == 0) {
            snprintf(filepath, sizeof(filepath), "/%s", info.name);
        } else {
            snprintf(filepath, sizeof(filepath), "%s/%s", dir_path, info.name);
        }

        sent += send_file_with_ymodem(filepath);
        osDelay(200);
    }

    lfs_dir_close(&lfs, &dir);
    return sent;
}

void send_file_ymodem_working(void) {
    char msg[80];
    int total = 0;

    snprintf(msg, sizeof(msg), "=== YMODEM BATCH TRANSFER (RAM buffered) ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    total += send_directory_with_ymodem("/");
    total += send_directory_with_ymodem("/data");

    if (total == 0) {
        snprintf(msg, sizeof(msg), "YMODEM: no files found\r\n");
    } else {
        snprintf(msg, sizeof(msg), "YMODEM complete: %d file(s)\r\n", total);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief Working XMODEM file send for Tera Term
  */
void send_file_xmodem(const char *filename) {
    char msg[100];
    lfs_file_t file;
    uint8_t packet[132];
    uint8_t buffer[128];
    int bytes_read;
    uint8_t block_num = 1;
    uint32_t total_bytes = 0;

    // Open file
    if (lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY) != LFS_ERR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: Cannot open file\r\n", 24, HAL_MAX_DELAY);
        return;
    }

    uint32_t file_size = lfs_file_size(&lfs, &file);
    snprintf(msg, sizeof(msg), "XMODEM Send: %s (%lu bytes)\r\n", filename, file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Calculate how many full packets needed
    uint16_t total_packets = (file_size + 127) / 128;
    snprintf(msg, sizeof(msg), "Total packets: %d\r\n", total_packets);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t*)"Start transfer in 10 seconds...\r\n", 31, HAL_MAX_DELAY);
    osDelay(10000);

    // Send all packets
    for (uint16_t pkt = 0; pkt < total_packets; pkt++) {
        bytes_read = lfs_file_read(&lfs, &file, buffer, 128);
        total_bytes += bytes_read;

        // Prepare packet
        packet[0] = 0x01; // SOH
        packet[1] = block_num;
        packet[2] = 255 - block_num;

        // Copy actual data
        memcpy(&packet[3], buffer, bytes_read);

        // Fill remainder with zeros (not 0x1A!)
        if (bytes_read < 128) {
            memset(&packet[3 + bytes_read], 0, 128 - bytes_read);
        }

        // Calculate checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 128; i++) {
            checksum += packet[3 + i];
        }
        packet[131] = checksum;

        // Send packet
        HAL_UART_Transmit(&huart2, packet, 132, HAL_MAX_DELAY);

        // Progress
        snprintf(msg, sizeof(msg), "Packet %d/%d sent (%lu bytes)\r\n",
                pkt + 1, total_packets, total_bytes);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        block_num++;
        if (block_num == 0) block_num = 1;

        osDelay(100); // Delay between packets
    }

    // Send EOT
    uint8_t eot = 0x04;
    HAL_UART_Transmit(&huart2, &eot, 1, HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "Transfer complete: %lu bytes\r\n", total_bytes);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    lfs_file_close(&lfs, &file);
}
/**
  * @brief Simple file send for Tera Term
  */
void send_file_teraterm(const char *filename) {
    char msg[100];
    lfs_file_t file;
    uint8_t buffer[256];
    int bytes_read;
    uint32_t total_sent = 0;

    // Open file
    if (lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY) != LFS_ERR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: Cannot open file\r\n", 24, HAL_MAX_DELAY);
        return;
    }

    uint32_t file_size = lfs_file_size(&lfs, &file);

    // Send file info first
    snprintf(msg, sizeof(msg), "FILE:%s SIZE:%lu\r\n", filename, file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Small delay to let Tera Term prepare
    osDelay(100);

    // Send file content directly (Tera Term can save this as binary)
    while ((bytes_read = lfs_file_read(&lfs, &file, buffer, sizeof(buffer))) > 0) {
        HAL_UART_Transmit(&huart2, buffer, bytes_read, HAL_MAX_DELAY);
        total_sent += bytes_read;

        // Small delay to prevent buffer overrun
        osDelay(5);
    }

    // Send completion marker
    snprintf(msg, sizeof(msg), "\r\n=== FILE END: %s, %lu bytes ===\r\n", filename, total_sent);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    lfs_file_close(&lfs, &file);
}

/**
  * @brief Send file as hex dump (human readable)
  */
void send_file_hexdump(const char *filename) {
    char msg[100];
    lfs_file_t file;
    uint8_t buffer[16];
    int bytes_read;
    uint32_t offset = 0;

    if (lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY) != LFS_ERR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: Cannot open file\r\n", 24, HAL_MAX_DELAY);
        return;
    }

    uint32_t file_size = lfs_file_size(&lfs, &file);
    snprintf(msg, sizeof(msg), "=== HEX DUMP: %s (%lu bytes) ===\r\n", filename, file_size);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    while ((bytes_read = lfs_file_read(&lfs, &file, buffer, 16)) > 0) {
        // Print offset
        snprintf(msg, sizeof(msg), "%08lX: ", offset);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        // Print hex values
        for (int i = 0; i < 16; i++) {
            if (i < bytes_read) {
                snprintf(msg, sizeof(msg), "%02X ", buffer[i]);
            } else {
                snprintf(msg, sizeof(msg), "   ");
            }
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        // Print ASCII values
        HAL_UART_Transmit(&huart2, (uint8_t*)" ", 1, HAL_MAX_DELAY);
        for (int i = 0; i < bytes_read; i++) {
            if (buffer[i] >= 32 && buffer[i] <= 126) {
                snprintf(msg, sizeof(msg), "%c", buffer[i]);
            } else {
                snprintf(msg, sizeof(msg), ".");
            }
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
        offset += bytes_read;
        osDelay(10);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"=== END OF DUMP ===\r\n", 21, HAL_MAX_DELAY);
    lfs_file_close(&lfs, &file);
}

/**
  * @brief Receive file from Tera Term (simple version)
  */
void receive_file_teraterm(const char *filename) {
    char msg[100];
    lfs_file_t file;
    uint8_t buffer[128];
    uint32_t total_received = 0;
    uint32_t start_time = HAL_GetTick();

    snprintf(msg, sizeof(msg), "READY to receive file: %s\r\n", filename);
    snprintf(msg, sizeof(msg), "Send file data now. Press Ctrl+C in Tera Term to cancel.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Open file for writing
    if (lfs_file_open(&lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: Cannot create file\r\n", 26, HAL_MAX_DELAY);
        return;
    }

    // Receive data for 30 seconds or until no data for 2 seconds
    uint32_t last_data_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time < 30000) && (HAL_GetTick() - last_data_time < 2000)) {
        int bytes_received = 0;

        // Try to receive data
        for (int i = 0; i < sizeof(buffer); i++) {
            uint8_t ch;
            if (HAL_UART_Receive(&huart2, &ch, 1, 50) == HAL_OK) {
                buffer[bytes_received++] = ch;
                last_data_time = HAL_GetTick();
            } else {
                break; // No more data
            }
        }

        if (bytes_received > 0) {
            lfs_file_write(&lfs, &file, buffer, bytes_received);
            total_received += bytes_received;

            // Progress every 1KB
            if (total_received % 1024 == 0) {
                snprintf(msg, sizeof(msg), "Received: %lu bytes\r\n", total_received);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        }

        osDelay(10);
    }

    snprintf(msg, sizeof(msg), "FILE RECEIVED: %s, %lu bytes\r\n", filename, total_received);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    lfs_file_close(&lfs, &file);
}




/**
  * @brief Инициализация ПИ-регулятора
  * @param pi: указатель на структуру ПИ-регулятора
  * @param Kp: коэффициент пропорциональной составляющей
  * @param Ki: коэффициент интегральной составляющей
  * @param out_min: минимальное значение выхода
  * @param out_max: максимальное значение выхода
  */
void PI_Init(PI_Controller* pi, float Kp, float Ki, float out_min, float out_max)
{
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

/**
  * @brief Обновление ПИ-регулятора
  * @param pi: указатель на структуру ПИ-регулятора
  * @param setpoint: заданное значение
  * @param measurement: текущее измеренное значение
  * @param dt: время с последнего обновления (сек)
  * @retval Управляющее воздействие
  */
float PI_Update(PI_Controller* pi, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Обновляем интегральную составляющую
    pi->integral += error * dt;

    // Ограничение интеграла (anti-windup)
    if(pi->integral > pi->out_max / pi->Ki)
        pi->integral = pi->out_max / pi->Ki;
    if(pi->integral < pi->out_min / pi->Ki)
        pi->integral = pi->out_min / pi->Ki;

    // Вычисление выходного сигнала
    float output = pi->Kp * error + pi->Ki * pi->integral;

    // Ограничение выхода
    if(output > pi->out_max) output = pi->out_max;
    if(output < pi->out_min) output = pi->out_min;

    return output;
}

/**
  * @brief Вывод списка файлов в директории
  * @param path: путь к директории
  */
void list_files(const char *path) {
    char msg[100];
    lfs_dir_t dir;
    struct lfs_info info;

    snprintf(msg, sizeof(msg), "Listing directory: %s\r\n", path);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    int err = lfs_dir_open(&lfs, &dir, path);
    if (err != LFS_ERR_OK) {
        snprintf(msg, sizeof(msg), "Error opening directory: %d\r\n", err);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Чтение содержимого директории
    while (lfs_dir_read(&lfs, &dir, &info) > 0) {
        if (info.type == LFS_TYPE_REG) {
            snprintf(msg, sizeof(msg), "FILE: %s (%lu bytes)\r\n", info.name, info.size);
        } else if (info.type == LFS_TYPE_DIR) {
            snprintf(msg, sizeof(msg), "DIR:  %s/\r\n", info.name);
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    lfs_dir_close(&lfs, &dir);
}

// Функции драйвера для LittleFS
static int nor_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t address = block * c->block_size + off;
    return (NOR_ReadBytes(&nor_flash, buffer, address, size) == NOR_OK) ? 0 : LFS_ERR_IO;
}

static int nor_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t address = block * c->block_size + off;
    return (NOR_WriteBytes(&nor_flash, (uint8_t*)buffer, address, size) == NOR_OK) ? 0 : LFS_ERR_IO;
}

static int nor_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t address = block * c->block_size;
    return (NOR_EraseBlock(&nor_flash, block) == NOR_OK) ? 0 : LFS_ERR_IO;
}

static int nor_sync(const struct lfs_config *c) {
    return 0; // NOR flash не требует синхронизации
}

/**
  * @brief Инициализация конфигурации LittleFS
  */
void littlefs_config_init(void) {
    cfg.context = &nor_flash;

    // Функции блочного устройства
    cfg.read  = nor_read;
    cfg.prog  = nor_prog;
    cfg.erase = nor_erase;
    cfg.sync  = nor_sync;

    // Параметры памяти
    cfg.read_size = 256;
    cfg.prog_size = 256;
    cfg.block_size = 65536;    // 64KB - размер блока NOR
    cfg.block_count = 128;     // 128 блоков * 64KB = 8MB (W25Q64)
    cfg.block_cycles = 1000;   // Количество циклов стирания

    // Буферы для кэширования
    cfg.read_buffer = lfs_read_buffer;
    cfg.prog_buffer = lfs_prog_buffer;
    cfg.lookahead_buffer = lfs_lookahead_buffer;
    cfg.lookahead_size = 128;
    cfg.cache_size = 256;
}

/**
  * @brief Инициализация NOR Flash
  * @retval Статус инициализации
  */
nor_err_e nor_init(void) {
    nor_t nor = {
        .config = {
            .CsAssert = nor_cs_assert,
            .CsDeassert = nor_cs_deassert,
            .SpiTxFxn = nor_spi_tx,
            .SpiRxFxn = nor_spi_rx,
            .DelayUs = nor_delay_us,
            .MutexLockFxn = NULL,
            .MutexUnlockFxn = NULL
        }
    };

    memcpy(&nor_flash, &nor, sizeof(nor_t));
    return NOR_Init(&nor_flash);
}

// Функции низкоуровневого доступа к SPI
void nor_cs_assert(void) {
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    nor_delay_us(1);
}

void nor_cs_deassert(void) {
    nor_delay_us(1);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    nor_delay_us(1);
}

void nor_spi_tx(uint8_t *txBuf, uint32_t size) {
    nor_delay_us(1);
    HAL_SPI_Transmit(&hspi2, txBuf, size, HAL_MAX_DELAY);
}

void nor_spi_rx(uint8_t *rxBuf, uint32_t size) {
    nor_delay_us(1);
    HAL_SPI_Receive(&hspi2, rxBuf, size, HAL_MAX_DELAY);
}

/**
  * @brief Точная задержка в микросекундах
  * @param us: количество микросекунд
  */
void nor_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

void send_file_list(void) {
    char msg[100];
    lfs_dir_t dir;
    struct lfs_info info;
    int file_count = 0;

    HAL_UART_Transmit(&huart2, (uint8_t*)"=== Available Files ===\r\n", 25, HAL_MAX_DELAY);

    // List root directory
    if (lfs_dir_open(&lfs, &dir, "/") == LFS_ERR_OK) {
        while (lfs_dir_read(&lfs, &dir, &info) > 0) {
            if (info.type == LFS_TYPE_REG) {
                snprintf(msg, sizeof(msg), "ROOT: %s (%lu bytes)\r\n", info.name, info.size);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                file_count++;
            }
        }
        lfs_dir_close(&lfs, &dir);
    }

    // List /data directory
    if (lfs_dir_open(&lfs, &dir, "/data") == LFS_ERR_OK) {
        while (lfs_dir_read(&lfs, &dir, &info) > 0) {
            if (info.type == LFS_TYPE_REG) {
                snprintf(msg, sizeof(msg), "DATA: %s (%lu bytes)\r\n", info.name, info.size);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                file_count++;
            }
        }
        lfs_dir_close(&lfs, &dir);
    }

    snprintf(msg, sizeof(msg), "Total files: %d\r\n", file_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief Receive filename from UART
  */
int receive_filename(char *filename, int max_len) {
    char msg[100];
    uint8_t ch;
    int idx = 0;
    uint32_t timeout = 5000; // 5 second timeout
    uint32_t start = HAL_GetTick();

    HAL_UART_Transmit(&huart2, (uint8_t*)"Enter filename: ", 16, HAL_MAX_DELAY);

    while (HAL_GetTick() - start < timeout) {
        if (HAL_UART_Receive(&huart2, &ch, 1, 10) == HAL_OK) {
            if (ch == '\r' || ch == '\n') {
                filename[idx] = '\0';
                if (idx > 0) {
                    snprintf(msg, sizeof(msg), "OK: %s\r\n", filename);
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                    return 0; // Success
                }
            } else if (ch >= 32 && ch <= 126 && idx < max_len - 1) { // Printable chars
                filename[idx++] = ch;
                // Echo character
                HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);
            }
        }
        osDelay(1);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout!\r\n", 10, HAL_MAX_DELAY);
    return -1; // Timeout
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Функция задачи по умолчанию
  * @param  argument: Не используется
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_TaskDraw */
/**
* @brief Function implementing the TaskDraw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_TaskDraw */
void Start_TaskDraw(void const * argument)
{
  /* USER CODE BEGIN Start_TaskDraw */
	 // Инициализация LVGL
	  lv_init();
	  lv_port_disp_init();

	  // Настройка основного экрана
	  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x7cfe29), LV_PART_MAIN);
	  lv_obj_set_style_text_color(lv_scr_act(), lv_color_make(0,0,0), LV_PART_MAIN);

	  // Создание элементов интерфейса
	  // Бар для отображения температуры
	  lv_obj_t * temp_bar = lv_bar_create(lv_scr_act());
	  lv_obj_set_size(temp_bar, 20, 100);
	  lv_obj_align(temp_bar, LV_ALIGN_RIGHT_MID, -10, 50);
	  lv_obj_set_style_bg_color(temp_bar, lv_color_make(203, 58, 0), LV_PART_MAIN);
	  lv_obj_set_style_bg_color(temp_bar, lv_color_make(0xFF, 0, 0), LV_PART_INDICATOR);
	  lv_bar_set_range(temp_bar, 2000, 4000);
	  lv_bar_set_value(temp_bar, 0, LV_ANIM_OFF);

	  // Индикатор активности
	  lv_obj_t * spinner = lv_spinner_create(lv_scr_act(), 3000, 60);
	  lv_obj_set_size(spinner, 64, 64);
	  lv_obj_align(spinner, LV_ALIGN_TOP_RIGHT, 0, 0);

	  // График напряжения
	  lv_obj_t * chart = lv_chart_create(lv_scr_act());
	  lv_obj_set_size(chart, 100, 100);
	  lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, -20);
	  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
	  lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 5, 5, 5, 1, true, 30);
	  lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 5, 5, 5, 1, true, 30);
	  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 300, 600);

	  // Текстовый лейбл для отображения напряжений
	  lv_obj_t * label = lv_label_create(lv_scr_act());
	  lv_label_set_text(label, "0");
	  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 10);

	  static lv_style_t style_label;
	  lv_style_init(&style_label);
	  lv_style_set_text_font(&style_label, &lv_font_montserrat_14);
	  lv_obj_add_style(label, &style_label, 0);

	  // Серия данных для графика
	  lv_chart_series_t * series1 = lv_chart_add_series(chart, lv_color_make(0xFF,0,0), LV_CHART_AXIS_PRIMARY_Y);

	  // Основной цикл задачи
  /* Infinite loop */
  for(;;)
  {
    // Обновление данных на экране
    int Vout_int = (int)(Vout * 100);
    int Device_temp_int = (int)(Device_temp * 100);

    // Обновление бара температуры
    lv_bar_set_value(temp_bar, Device_temp_int, LV_ANIM_ON);

    // Обновление текстовой информации
    char buf[32];
    snprintf(buf, sizeof(buf), "Vref=%.2fV\nVout=%.2fV", Vref, Vout);
    lv_label_set_text(label, buf);

    // Добавление точки на график
    lv_chart_set_next_value(chart, series1, Vout_int);

    // Обработка событий LVGL
    lv_timer_handler();
    osDelay(25); // Обновление каждые 50 мс
  }
  /* USER CODE END Start_TaskDraw */
}

/* USER CODE BEGIN Header_Start_TaskDCDC */
/**
  * @brief  Задача управления DCDC преобразователем
  * @param  argument: Не используется
  * @retval None
  */
/* USER CODE END Header_Start_TaskDCDC */
void Start_TaskDCDC(void const * argument)
{
  /* USER CODE BEGIN Start_TaskDCDC */
	uint32_t last_button = 1;
	// Инициализация ПИ-регулятора
	PI_Init(&pi, 0.5f, 500.0f, -DCDC_VIN, DCDC_VIN);
  /* Infinite loop */
  for(;;)
  {
   // Обработка кнопки для переключения напряжения
	uint32_t btn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	if (btn == 0 && last_button == 1)
	{
		if (Vref == 3.3f) Vref = 5.0f;
		else Vref = 3.3f;
	}
	last_button = btn;

//	// Обновление ПИ-регулятора
//	control = PI_Update(&pi, Vref, Vout, DCDC_DT);
//
//	// Ограничение управляющего сигнала
//	if (control > DCDC_VIN) control = DCDC_VIN;
//	if (control < 0.0f)     control = 0.0f;
//
//	// Расчет скважности ШИМ
//	uint16_t duty = (uint16_t)((control / DCDC_VIN) * __HAL_TIM_GET_AUTORELOAD(&htim3));
//	duty_last = duty;
//
//	// Моделирование DCDC преобразователя и тепловых процессов
//	float P_heat = Vout * Vout / DCDC_RLOAD;
//	float P_cool = 0.1f * (Device_temp - 20.0f);
//	Device_temp += 100.0f * DCDC_DT * (P_heat - P_cool);
//	Vout += DCDC_DT * ((DCDC_VIN * duty / __HAL_TIM_GET_AUTORELOAD(&htim3)) - Vout / DCDC_RLOAD) / DCDC_C;

    // Data logging
    if (logging_enabled && (HAL_GetTick() - last_log_time >= log_interval_ms)) {
        last_log_time = HAL_GetTick();
        log_data_point();
    }

	osDelay(1); // Высокая частота обновления
  }
  /* USER CODE END Start_TaskDCDC */
}

/* USER CODE BEGIN Header_Start_TaskLittleFS */
/**
  * @brief  Задача файловой системы LittleFS
  * @param  argument: Не используется
  * @retval None
  */
/* USER CODE END Header_Start_TaskLittleFS */
void Start_TaskLittleFS(void const * argument)
{
  /* USER CODE BEGIN Start_TaskLittleFS */
	   char msg[100];
	    int err;
	    static uint32_t counter = 0;
	    static uint32_t last_save = 0;

	    // Инициализация NOR Flash
	    osDelay(1000);
	    strcpy(msg, "Initializing NOR Flash...\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

	    nor_err_e nor_result = nor_init();
	    if (nor_result != NOR_OK) {
	        snprintf(msg, sizeof(msg), "NOR Init failed: %d\r\n", nor_result);
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);
	        vTaskDelete(NULL);
	        return;
	    }

	    snprintf(msg, sizeof(msg), "NOR Flash ID: 0x%06lX\r\n", nor_flash.info.u32JedecID);
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

	    // Монтирование LittleFS с повторными попытками
	    strcpy(msg, "Mounting LittleFS...\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

	    littlefs_config_init();

	    // Пытаемся монтировать несколько раз
	    for(int retry = 0; retry < 3; retry++) {
	        err = lfs_mount(&lfs, &cfg);
	        if (err == LFS_ERR_OK) {
	            break;
	        }
	        snprintf(msg, sizeof(msg), "Mount failed (%d), formatting...\r\n", err);
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

	        lfs_format(&lfs, &cfg);
	        osDelay(100);
	    }

	    // Проверяем успешность монтирования
	    if (err != LFS_ERR_OK) {
	        strcpy(msg, "LittleFS mount FAILED!\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);
	        vTaskDelete(NULL);
	        return;
	    }

	    // Создание структуры директорий
	    lfs_mkdir(&lfs, "/data");

	    // Тестируем запись сразу
	    lfs_file_t test_file;
	    if (lfs_file_open(&lfs, &test_file, "/data/test.txt", LFS_O_WRONLY | LFS_O_CREAT) == LFS_ERR_OK) {
	        const char *test_msg = "LittleFS test successful!\r\n";
	        lfs_file_write(&lfs, &test_file, test_msg, strlen(test_msg));
	        lfs_file_close(&lfs, &test_file);
	        strcpy(msg, "LittleFS test write: OK\r\n");
	    } else {
	        strcpy(msg, "LittleFS test write: FAILED\r\n");
	    }
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

	    strcpy(msg, "=== LittleFS Ready ===\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);
  /* Infinite loop */
  for(;;)
  {
        char status_msg[128];

        // Обработка команд UART
        if (huart2.RxState == HAL_UART_STATE_READY) {
            uint8_t cmd;
            if (HAL_UART_Receive(&huart2, &cmd, 1, 10) == HAL_OK) {
                switch(cmd) {
                    case 'l': // Листинг файлов
                        list_files("/");
                        osDelay(1);
                        list_files("/data");
                        osDelay(1);
                        break;

                    case 's': // Статус системы
                        snprintf(status_msg, sizeof(status_msg),
                                 "Status - Counter: %lu, NOR ID: 0x%06lX\r\n",
                                 counter, nor_flash.info.u32JedecID);
                        HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 50);
                        osDelay(1);
                        break;

                    case 't': // Создание тестового файла
                        {
                            lfs_file_t test_file;
                            if (lfs_file_open(&lfs, &test_file, "/data/testfile.txt", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) == LFS_ERR_OK) {
                                const char *test_data = "Test file for LittleFS\r\n";
                                lfs_file_write(&lfs, &test_file, test_data, strlen(test_data));
                                lfs_file_close(&lfs, &test_file);
                                strcpy(status_msg, "Test file created\r\n");
                            } else {
                                strcpy(status_msg, "Failed to create test file\r\n");
                            }
                            HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 50);
                            osDelay(1);
                        }
                        break;

                    case 'c': // Принудительное сохранение счетчика
                        {
                            err = lfs_file_open(&lfs, &file, "/data/counter.txt", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
                            if (err == LFS_ERR_OK) {
                                char counter_str[20];
                                snprintf(counter_str, sizeof(counter_str), "%lu", counter);
                                lfs_file_write(&lfs, &file, counter_str, strlen(counter_str));
                                lfs_file_close(&lfs, &file);
                                snprintf(status_msg, sizeof(status_msg), "Counter saved: %lu\r\n", counter);
                            } else {
                                strcpy(status_msg, "Failed to save counter\r\n");
                            }
                            HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 50);
                            last_save = counter;
                            osDelay(1);
                        }
                        break;

                    // НОВЫЕ КОМАНДЫ ДЛЯ РАБОТЫ С ФАЙЛАМИ
                    case 'f': // Список файлов для передачи
                        send_file_list();
                        break;

                    case 'x': // Отправить файл как бинарный (для Tera Term)
                        {
                            char filename[32];
                            if (receive_filename(filename, sizeof(filename)) == 0) {
                                send_file_teraterm(filename);
                            }
                        }
                        break;

                    case 'h': // Отправить файл как hex dump
                        {
                            char filename[32];
                            if (receive_filename(filename, sizeof(filename)) == 0) {
                                send_file_hexdump(filename);
                            }
                        }
                        break;

                    case 'r': // Принять файл из Tera Term
                        {
                            char filename[32];
                            if (receive_filename(filename, sizeof(filename)) == 0) {
                                receive_file_teraterm(filename);
                            }
                        }
                        break;

                    case 'L': // Start logging (uppercase L)
                        start_logging();
                        break;

                    case 'S': // Stop logging (uppercase S)
                        stop_logging();
                        break;

                    case 'X': // Send log file via XMODEM (uppercase X)
                        send_log_file_xmodem();
                        break;

                    case 'i': // Set log interval
                        {
                            char interval_str[10];
                            if (receive_filename(interval_str, sizeof(interval_str)) == 0) {
                                log_interval_ms = atoi(interval_str);
                                char msg[50];
                                snprintf(msg, sizeof(msg), "Log interval set to: %lu ms\r\n", log_interval_ms);
                                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                            }
                        }
                        break;

                    case 'y': // Send file via YMODEM
                        {
                            send_file_ymodem_working();
                        }
                        break;

                    case 'm': // Send file via XMODEM
                        {
                            char filename[32];
                            if (receive_filename(filename, sizeof(filename)) == 0) {
                                send_file_xmodem(filename);
                            }
                        }
                        break;

                    case 'H': // Check filesystem health
                        check_filesystem_health();
                        break;

                    case '?': // Справка
                        strcpy(status_msg,
                            "=== DCDC Logger Commands ===\r\n"
                        	"H - Check filesystem health\r\n"
                            "L - Start logging Vout data\r\n"
                            "S - Stop logging\r\n"
                            "X - Send log file (XMODEM)\r\n"
                            "i - Set log interval (ms)\r\n"
                            "l - List files\r\n"
                            "f - File list for transfer\r\n"
                            "x - Send any file (XMODEM)\r\n"
                            "y - Send ALL files (YMODEM)\r\n"
                            "r - Receive file\r\n"
                            "s - System status\r\n"
                            "t - Create test file\r\n"
                            "c - Save counter now\r\n"
                            "? - Help\r\n");
                        HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 50);
                        osDelay(1);
                        break;
                    default:
                        snprintf(status_msg, sizeof(status_msg), "Unknown: %c (use '?' for help)\r\n", cmd);
                        HAL_UART_Transmit(&huart2, (uint8_t*)status_msg, strlen(status_msg), 50);
                        osDelay(1);
                        break;
                }
            }
        }

        // Автосохранение счетчика каждые 60 секунд
        if (counter - last_save >= 600) {
            last_save = counter;
            err = lfs_file_open(&lfs, &file, "/data/counter.txt", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
            if (err == LFS_ERR_OK) {
                char counter_str[20];
                snprintf(counter_str, sizeof(counter_str), "%lu", counter);
                lfs_file_write(&lfs, &file, counter_str, strlen(counter_str));
                lfs_file_close(&lfs, &file);
            }
            osDelay(1);
        }

        counter++;
        osDelay(100); // Основная задержка задачи
  }
  /* USER CODE END Start_TaskLittleFS */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3)
  {
		// Обновление ПИ-регулятора
		control = PI_Update(&pi, Vref, Vout, DCDC_DT);

		// Ограничение управляющего сигнала
		if (control > DCDC_VIN) control = DCDC_VIN;
		if (control < 0.0f)     control = 0.0f;

		// Расчет скважности ШИМ
		uint16_t duty = (uint16_t)((control / DCDC_VIN) * __HAL_TIM_GET_AUTORELOAD(&htim3));
		duty_last = duty;

		// Моделирование DCDC преобразователя и тепловых процессов
		float P_heat = Vout * Vout / DCDC_RLOAD;
		float P_cool = 0.1f * (Device_temp - 20.0f);
		Device_temp += 100.0f * DCDC_DT * (P_heat - P_cool);
		Vout += DCDC_DT * ((DCDC_VIN * duty / __HAL_TIM_GET_AUTORELOAD(&htim3)) - Vout / DCDC_RLOAD) / DCDC_C;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_last);
  }
  /* USER CODE END Callback 1 */
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
