/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#ifndef __fatfs_H
#define __fatfs_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#if defined(STM32F107xC) && defined(MKS_TFT)
 #include "spisd_diskio.h" /* defines SPISD_Driver as external */
 #ifndef BOOTLOADER
  #include "spiflash_w25q16dv.h"
  #include "usbh_diskio.h"

extern char SPIFL_Path[4];	/* SPI Flash logical drive path */
extern char USBH_Path[4];	/* USB stick logical drive path */
 #endif /* BOOTLOADER */
extern char SPISD_Path[4];	/* SPI SD card logical drive path */

extern SPI_HandleTypeDef hspi1;

typedef enum {
	SPI_SDCARD = 0,
	SPI_FLASH
} dselect_t;

void deviceSelect(dselect_t device);
void deviceDeselect();

#elif defined(STM32F103xE) && defined(CZMINI)
 #include "sd_diskio.h" /* defines SD_Driver as external */

extern char SD_Path[4]; /* SD logical drive path */
#endif

void MX_FATFS_Init(void);
FRESULT transferFile(const TCHAR *source, const TCHAR *dest, uint8_t overwrite);

#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
