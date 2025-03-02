/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app_fuseprogramming.h"
#include "ll_aton_runtime.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default) // Defines NN_Instance_Default and NN_Interface_Default with network.c info

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************/
/*                              TIME MEASUREMENT                              */
/******************************************************************************/
void time_in(void);
uint32_t time_out(void);

uint32_t duration_us;
uint32_t duration_dwt;
static uint32_t t_init;
static uint32_t t_out;
float_t clock_Hz;
uint32_t cpuclk;

/******************************************************************************/
/*                               DWT INITIALIZATION                           */
/******************************************************************************/
void init_dwt()
{
    /* Enable Trace */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset Cycle Counter and Event Counters */
    ARM_PMU_CYCCNT_Reset();

    /* Enable Cycle Counter */
    ARM_PMU_CNTR_Enable(PMU_CNTENSET_CCNTR_ENABLE_Msk);

    /* Enable the PMU */
    ARM_PMU_Enable();
}

void time_in(void)
{
    ARM_PMU_CYCCNT_Reset();
    t_init = ARM_PMU_Get_CCNTR();
}

uint32_t time_out(void)
{
    t_out = ARM_PMU_Get_CCNTR();
    return (t_out - t_init);
}

/******************************************************************************/
/*                          SECURITY CONFIGURATION                            */
/******************************************************************************/
CACHEAXI_HandleTypeDef hcacheaxi;

static void Security_Config(void)
{
    __HAL_RCC_RIFSC_CLK_ENABLE();

    RIMC_MasterConfig_t RIMC_master = {0};
    RIMC_master.MasterCID = RIF_CID_1;
    RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;

    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

/******************************************************************************/
/*                             NPU RAM ENABLE                                 */
/******************************************************************************/
static void NPURam_enable(void)
{
    __HAL_RCC_NPU_CLK_ENABLE();
    __HAL_RCC_NPU_FORCE_RESET();
    __HAL_RCC_NPU_RELEASE_RESET();

    /* Enable NPU RAMs (4x448KB) */
    __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
    __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
    __HAL_RCC_RAMCFG_CLK_ENABLE();

    RAMCFG_HandleTypeDef hramcfg = {0};

    hramcfg.Instance = RAMCFG_SRAM3_AXI;
    HAL_RAMCFG_EnableAXISRAM(&hramcfg);

    hramcfg.Instance = RAMCFG_SRAM4_AXI;
    HAL_RAMCFG_EnableAXISRAM(&hramcfg);

    hramcfg.Instance = RAMCFG_SRAM5_AXI;
    HAL_RAMCFG_EnableAXISRAM(&hramcfg);

    hramcfg.Instance = RAMCFG_SRAM6_AXI;
    HAL_RAMCFG_EnableAXISRAM(&hramcfg);
}

/******************************************************************************/
/*                             NPU CACHE ENABLE                               */
/******************************************************************************/
static void NPUCache_enable()
{
    hcacheaxi.Instance = CACHEAXI;
    __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
    __HAL_RCC_CACHEAXI_CLK_ENABLE();
    __HAL_RCC_CACHEAXI_FORCE_RESET();
    __HAL_RCC_CACHEAXI_RELEASE_RESET();

    int err = HAL_CACHEAXI_Init(&hcacheaxi);
    if (err != HAL_OK)
    {
        while (1);
    }
}

/******************************************************************************/
/*                       EXTERNAL MEMORY INITIALIZATION                       */
/******************************************************************************/
uint32_t ts[2] = { 0 };
float *nn_buffer_in;
float *nn_buffer_out_p;
float nn_buffer_out[10];

#define ENABLE_MEMORY_READ

static void init_external_memories(void)
{
    BSP_XSPI_NOR_Init_t Flash;

    Flash.InterfaceMode = MX25UM51245G_OPI_MODE;
    Flash.TransferRate = MX25UM51245G_DTR_TRANSFER;

    if (BSP_XSPI_NOR_Init(0, &Flash) != BSP_ERROR_NONE)
    {
        __BKPT(0);
    }

    BSP_XSPI_NOR_EnableMemoryMappedMode(0);
    MODIFY_REG(XSPI2->CR, XSPI_CR_NOPREF, HAL_XSPI_AUTOMATIC_PREFETCH_DISABLE);

#ifdef ENABLE_MEMORY_READ
    uint32_t test_val[50];
    for (int i = 0; i < 50; i++)
    {
        test_val[i] = *((volatile uint32_t*)0x70000000 + i);
    }
#endif
}

/******************************************************************************/
/*                                TEST DATA                                   */
/******************************************************************************/
float sum = 0;
float user_input[28*28]={
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0, 252, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, 252, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0, 254, 255,   0,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 252, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 252, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 254, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  MX_GPIO_Init();
  Fuse_Programming();
  SystemClock_Config_HSI_overdrive();
  init_external_memories();

  NPURam_enable();

  NPUCache_enable();

  /* Set all required IPs as secure privileged */
  Security_Config();


  /*****************************************************************************/
  /*                                NN INIT                                    */
  /*****************************************************************************/
  LL_ATON_RT_RetValues_t ll_aton_rt_ret = LL_ATON_RT_DONE;
  const EpochBlock_ItemTypeDef *eb_list = LL_ATON_EpochBlockItems_Default();

  /* Retreive the start address of the input and output buffer
  (reserved in the activation buffer) */
  const LL_Buffer_InfoTypeDef * ibuffersInfos = NN_Interface_Default.input_buffers_info();
  const LL_Buffer_InfoTypeDef * obuffersInfos = NN_Interface_Default.output_buffers_info();
  nn_buffer_in = (float *)LL_Buffer_addr_start(&ibuffersInfos[0]);
  nn_buffer_out_p = (float *)LL_Buffer_addr_start(&obuffersInfos[0]);

  LL_ATON_RT_RuntimeInit();

  /*****************************************************************************/
  /*  Normalize user input data (0-255) to range [0,1] for neural network      */
  /*****************************************************************************/
  for(int i = 0; i < 28*28; i++)
  {
    nn_buffer_in[i] = user_input[i]/255.0;// Normalize pixel values
  }

  /*****************************************************************************/
  /*  Get clock frequency to compute inference duration later on               */
  /*  and init time measurement capabilities with PMU                          */
  /*****************************************************************************/
  cpuclk =  HAL_RCC_GetCpuClockFreq();
  clock_Hz = (float_t) cpuclk;
  init_dwt();

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	time_in();
	/* ------------- */
	/* - Inference - */
	/* ------------- */

#if 1
	/* Pre-process and fill the input buffer */
	//_pre_process(buffer_in);
	/* Perform the inference */
	LL_ATON_RT_Init_Network(&NN_Instance_Default);  // Initialize passed network instance object
	do {
	  /* Execute first/next step */
	  ll_aton_rt_ret = LL_ATON_RT_RunEpochBlock(&NN_Instance_Default);
	  /* Wait for next event */
	  if (ll_aton_rt_ret == LL_ATON_RT_WFE)
	  LL_ATON_OSAL_WFE();
	} while (ll_aton_rt_ret != LL_ATON_RT_DONE);
	/* Post-process the output buffer */
	/* Invalidate the associated CPU cache region if requested */
	//_post_process(buffer_out);
	LL_ATON_RT_DeInit_Network(&NN_Instance_Default);
	/* -------------------- */
	/* - End of Inference - */
	/* -------------------- */
#else
	LL_ATON_RT_Main(&NN_Instance_Default);
#endif

	duration_dwt = time_out();
	duration_us = (uint32_t)(((float_t)duration_dwt * 1000000.0)/clock_Hz);
	//printf("Inference: %d us (%d) cycles)\r\n",duration_us, duration_dwt);

	sum = 0;
	for(int i = 0 ; i < 10; i ++)
	{
	  nn_buffer_out[i] = *((float *)nn_buffer_out_p+i);
	  sum += nn_buffer_out[i];
	}
	HAL_Delay(100);

    /* USER CODE END WHILE */

  //MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_RCC_GPION_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
