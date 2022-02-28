/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.hpp"

    /**
  * Enable DMA controller clock
  */
    void MX_DMA_Init(void)
    {

        /* Init with LL driver */
        /* DMA controller clock enable */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
        /* DMA interrupt init */
        /* DMA1_Stream6_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    }


#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
