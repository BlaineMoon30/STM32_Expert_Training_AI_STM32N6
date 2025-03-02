
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
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

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Neural ART Embedded Client API
  *          Re-target of the printf function is out-of-scope.
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\[user_name]\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"

/* USER CODE BEGIN includes */
/* USER CODE END includes */

/* Entry points --------------------------------------------------------------*/

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default)
uint8_t *buffer_in;
uint8_t *buffer_out;

void set_clk_sleep_mode(void)
{
  /* Leave clocks enabled in Low Power modes */
  // Low-power clock enable misc
  __HAL_RCC_DBG_CLK_SLEEP_ENABLE();
  __HAL_RCC_XSPIPHYCOMP_CLK_SLEEP_ENABLE();

  // Low-power clock enable for memories
  __HAL_RCC_AXISRAM1_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM2_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM3_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_FLEXRAM_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_SLEEP_ENABLE();
  // LP clock AHB1: None
  // LP clock AHB2: None
  // LP clock AHB3
  __HAL_RCC_RIFSC_CLK_SLEEP_ENABLE();
  __HAL_RCC_RISAF_CLK_SLEEP_ENABLE();
  __HAL_RCC_IAC_CLK_SLEEP_ENABLE();
  // LP clock AHB4: None
  // LP clocks AHB5
  __HAL_RCC_XSPI1_CLK_SLEEP_ENABLE();
  __HAL_RCC_XSPI2_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_SLEEP_ENABLE();
  __HAL_RCC_NPU_CLK_SLEEP_ENABLE();
  // LP clocks APB1: None
  // LP clocks APB2
  __HAL_RCC_USART1_CLK_SLEEP_ENABLE();
  // LP clocks APB4: None
  // LP clocks APB5: None
}

void MX_X_CUBE_AI_Init(void)
{
    set_clk_sleep_mode();
    __HAL_RCC_NPU_CLK_ENABLE();
    __HAL_RCC_NPU_FORCE_RESET();
    __HAL_RCC_NPU_RELEASE_RESET();
    npu_cache_init();
    /* USER CODE BEGIN 5 */
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  LL_ATON_RT_RetValues_t ll_aton_rt_ret = LL_ATON_RT_DONE;
  const LL_Buffer_InfoTypeDef * ibuffersInfos = NN_Interface_Default.input_buffers_info();
  const LL_Buffer_InfoTypeDef * obuffersInfos = NN_Interface_Default.output_buffers_info();
  buffer_in = (uint8_t *)LL_Buffer_addr_start(&ibuffersInfos[0]);
  buffer_out = (uint8_t *)LL_Buffer_addr_start(&obuffersInfos[0]);
  LL_ATON_RT_RuntimeInit();
  // run 10 inferences
  for (int inferenceNb = 0; inferenceNb<10; ++inferenceNb) {
    /* ------------- */
    /* - Inference - */
    /* ------------- */
    /* Pre-process and fill the input buffer */
    //_pre_process(buffer_in);
    /* Perform the inference */
    LL_ATON_RT_Init_Network(&NN_Instance_Default);  // Initialize passed network instance object
    do {
      /* Execute first/next step */
      ll_aton_rt_ret = LL_ATON_RT_RunEpochBlock(&NN_Instance_Default);
      /* Wait for next event */
      if (ll_aton_rt_ret == LL_ATON_RT_WFE) {
        LL_ATON_OSAL_WFE();
      }
    } while (ll_aton_rt_ret != LL_ATON_RT_DONE);
    /* Post-process the output buffer */
    /* Invalidate the associated CPU cache region if requested */
    //_post_process(buffer_out);
    LL_ATON_RT_DeInit_Network(&NN_Instance_Default);
    /* -------------------- */
    /* - End of Inference - */
    /* -------------------- */
  }
  LL_ATON_RT_RuntimeDeInit();
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
