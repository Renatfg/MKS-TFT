/**
  ******************************************************************************
  * File Name          : screenstates.c
  * Description        : This file contains screen state machine definitions
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 Roman Stepanov
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

#include <string.h>

#include "ui.h"
#include "lcd.h"
#include "fatfs.h"
#include "eeprom.h"

static FATFS flashFileSystem;	// 0:/
static FATFS sdFileSystem;		// 1:/
static FATFS usbFileSystem;		// 2:/

extern TIM_HandleTypeDef htim2;

#define MKS_PIC_SD	"1:/mks_pic"
#define MKS_PIC_FL	"0:/mks_pic"

#define READY_PRINT	"Магнум"

uint8_t statString[MAXSTATSIZE+1];

uint16_t moveStep = MOVE_10;
// uint8_t offMode = MANUAL_OFF;
uint8_t connectSpeed = CONNECT_115200;
uint8_t preheatDev = PR_EXTRUDER_1;
uint8_t extrudeDev = EXTRUDER_1;
// uint8_t preheatSelDegree = STEP_1_DEGREE;
uint8_t extrudeDistance = DISTANCE_1;
uint8_t extrudeSelSpeed = SPEED_SLOW;
uint8_t selectedFs = FS_SD;

#define MAX_EXTRUDER_TEMP   275

uint16_t e1PreheatTemp = 220;
uint16_t e2PreheatTemp = 220;
uint16_t filChangeTemp = 220;

/*
 * user callback declaration
 */

void uiInitialize (xUIEvent_t *pxEvent);
void uiMainMenu   (xUIEvent_t *pxEvent);
void uiSetupMenu  (xUIEvent_t *pxEvent);

void uiMoveMenu (xUIEvent_t *pxEvent);
void uiMoveMenuXAct(xUIEvent_t *pxEvent);
void uiMoveMenuYAct(xUIEvent_t *pxEvent);
void uiMoveMenuZAct(xUIEvent_t *pxEvent);
void uiMoveMenuDistance(xUIEvent_t *pxEvent);

void uiTemperatureMenu(xUIEvent_t *pxEvent);
void uiE1TempSliderMenu(xUIEvent_t *pxEvent);
void uiE1TempSliderSetMenu(xUIEvent_t *pxEvent);

void uiFilChangeMenu(xUIEvent_t *pxEvent);
void uiFilChangeTempSliderSetMenu(xUIEvent_t *pxEvent);

void uiFilPreheatMenu(xUIEvent_t *pxEvent);
void uiFilReplaceMenu(xUIEvent_t *pxEvent);
void uiFilFeedMenu(xUIEvent_t *pxEvent);

typedef void (*volatile eventProcessor_t) (xUIEvent_t *);
eventProcessor_t processEvent = uiInitialize;

/*
 * service routines declaration
 */

static void uiMediaStateChange(uint16_t event);

static void uiDrawSlider(uint16_t y, uint32_t pos, uint32_t scale, uint16_t color1, uint16_t color2);
static void uiDrawProgressBar(uint32_t scale, uint16_t color);
static void uiUpdateProgressBar(uint32_t progress);

static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y,
		uint16_t width, uint16_t height, uint8_t resetWindow);

__STATIC_INLINE void uiNextState(void (*volatile next) (xUIEvent_t *pxEvent)) {
	processEvent = next;
	xUIEvent_t event = { INIT_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

__STATIC_INLINE void uiToggleParentState(void (*volatile next) (xUIEvent_t *pxEvent)) {
	processEvent = next;
	xUIEvent_t event = { TOGGLE_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

__STATIC_INLINE void uiShortBeep() {
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	osDelay(12);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}

typedef struct {
    uint16_t                x1, y1, x2, y2;
    uint16_t                color;
    const char              *label;
    void                    (*labelFn) (uint16_t x, uint16_t y);

	const eventProcessor_t	pOnTouchUp;
	const eventProcessor_t  pOnTouchDown;
} xButton_t;

__STATIC_INLINE void uiDrawMenuItem(const xButton_t *pMenu) {

    Lcd_Fill_Rect(pMenu->x1, pMenu->y1, pMenu->x2, pMenu->y2, pMenu->color);

    if (pMenu->label) {
        uint16_t    x = (pMenu->x2 - pMenu->x1)/2 + pMenu->x1 - (strlen(pMenu->label) << 2),
                    y = (pMenu->y2 - pMenu->y1)/2 + pMenu->y1 - 8;

        Lcd_Put_Text(x, y, 16, pMenu->label, 0xffffu);

    } else if (pMenu->labelFn) {

        (*pMenu->labelFn) ((pMenu->x2 - pMenu->x1)/2 + pMenu->x1, (pMenu->y2 - pMenu->y1)/2 + pMenu->y1);
    }
}

__STATIC_INLINE void uiDrawMenu(const xButton_t *pMenu, size_t menuSize) {

	for (int s=0; s<menuSize; pMenu++, s++)
	    uiDrawMenuItem(pMenu);
}

static uint16_t touchX, touchY;

__STATIC_INLINE void uiMenuHandleEventDefault(const xButton_t *pMenu, size_t menuSize, xUIEvent_t *pxEvent) {

	if (pxEvent) {
		switch (pxEvent->ucEventID) {
		case SDCARD_INSERT:
		case SDCARD_REMOVE:
		case USBDRIVE_INSERT:
		case USBDRIVE_REMOVE:
			uiMediaStateChange(pxEvent->ucEventID);
			break;

        case SHOW_STATUS:
            Lcd_Fill_Rect(0, LCD_MAX_Y - 9, LCD_MAX_X, LCD_MAX_Y, 0);
            Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, statString, 0xffffu);
            break;

		case INIT_EVENT:
			if (pMenu) {
				Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));
				uiDrawMenu(pMenu, menuSize);
#if 0
				char buffer[12];
				sprintf(buffer, "%03u:%03u", touchX, touchY);
				Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
#endif
			}
			break;

		case TOUCH_DOWN_EVENT:
		case TOUCH_UP_EVENT:
			if (pMenu) {
#if 0
				char buffer[12];
                sprintf(buffer, "%05u:%05u", (pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
						pxEvent->ucData.touchXY & 0x7fffu);
                Lcd_Put_Text(80, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
#endif
				Lcd_Translate_Touch_Pos((pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
						pxEvent->ucData.touchXY & 0x7fffu, &touchX, &touchY);

                for (int s=0; s<menuSize; s++) {

                    if (touchX >= pMenu[s].x1 && touchX <= pMenu[s].x2
                        && touchY >= pMenu[s].y1 && touchY <= pMenu[s].y2)
                    {
                 		if (TOUCH_DOWN_EVENT == pxEvent->ucEventID && pMenu[s].pOnTouchDown) {
                            uiNextState(pMenu[s].pOnTouchDown);
                        }
                        if (TOUCH_UP_EVENT == pxEvent->ucEventID && pMenu[s].pOnTouchUp) {
                            uiNextState(pMenu[s].pOnTouchUp);
                        }
                        break;
                    }
                }
			}
			break;

		default:
			break;
		}
	}
}

/*
 * user callback definition
 */

void uiInitialize (xUIEvent_t *pxEvent) {

	if (INIT_EVENT == pxEvent->ucEventID) {

		DIR dir;

		Lcd_Init(LCD_LANDSCAPE_CL);
		Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));

		// mount internal flash, format if needed
		f_mount(&flashFileSystem, SPIFL_Path, 1);  // mount flash
		f_mount(&sdFileSystem, SPISD_Path, 1);      // mount sd card, mount usb later

		if (sdFileSystem.fs_type && FR_OK == f_opendir(&dir, MKS_PIC_SD)) {

			FRESULT res = FR_OK; /* Open the directory */
			BYTE *work = pvPortMalloc(_MAX_SS);
			if (work) {

				f_mkfs ("0:", FM_ANY, 0, work, _MAX_SS);	/* Create a FAT volume */
				vPortFree(work);

				f_mount(&flashFileSystem, SPIFL_Path, 1);
			}

			res = f_mkdir(MKS_PIC_FL);
			if (res == FR_OK || res == FR_EXIST) {

				FILINFO fno;
				size_t count = 0;
				while (FR_OK == f_readdir(&dir, &fno) && fno.fname[0]) {
					if (((fno.fattrib & AM_DIR) == 0) && strstr(fno.fname, ".bin"))
						count++;
				}

				f_rewinddir(&dir);
				if (count) {

					Lcd_Put_Text(56, 80, 16, "Copying files to FLASH...", Lcd_Get_RGB565(0, 63, 0));
					uiDrawProgressBar(count, Lcd_Get_RGB565(0, 63, 0));

					// TODO: format flash again?

					count = 0;
					res = FR_OK;

					while (FR_OK == f_readdir(&dir, &fno) && fno.fname[0]) {
						if (((fno.fattrib & AM_DIR) == 0) && strstr(fno.fname, ".bin")) {

							char src[50];
							char dst[50];

							Lcd_Fill_Rect(0, 232, 319, 240, 0);
							snprintf(src, sizeof(src), "%02u", res);
							Lcd_Put_Text(304, 232, 8, src,
									res == FR_OK ? Lcd_Get_RGB565(0, 63, 0) : Lcd_Get_RGB565(31, 0, 0));

							snprintf(src, sizeof(src), MKS_PIC_SD "/%s", fno.fname);
							snprintf(dst, sizeof(dst), MKS_PIC_FL "/%s", fno.fname);

							Lcd_Put_Text(0, 232, 8, src, Lcd_Get_RGB565(0, 63, 0));
							res = transferFile(src, dst, 1);

							uiUpdateProgressBar(++count);
						}
					}

					Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));
				}
			}

			f_closedir(&dir);
			f_rename(MKS_PIC_SD, MKS_PIC_SD ".old");
		}

		uiNextState(uiMainMenu);
	} else
		uiMenuHandleEventDefault(NULL, 0, pxEvent);
}

void uiMainMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 110, LCD_BLACK, "МАГНУМ" },
        { 0, 110, 320, 150, LCD_BLACK, "Иконки и статусы температуры" },
        { 20, 170, 150, 230, LCD_DANUBE, "Печать"  },
        { 170, 170, 300, 230, LCD_DANUBE, "Настройки", .pOnTouchUp = uiSetupMenu },
    };

	uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static char currentIPLabel[30] = "Текущий IP: 192.168.0.253";
static xButton_t setupMenu[] = {
    { 5,  20,  80, 80, LCD_ORANGE, "T Off" },
    { 85,  20, 160, 80, LCD_ORANGE, "ДВ Off" },
    { 165, 20, 240, 80, LCD_ORANGE, "Ст. вниз" },
    { 245, 20, 315, 80, LCD_ORANGE, "Парк XY" },
    { 20, 100, 300, 130, LCD_BLACK, currentIPLabel },
    { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiMainMenu },
    { 85,  170, 160, 230, LCD_ORANGE, "Пруток", .pOnTouchUp = uiFilChangeMenu },
    { 165, 170, 240, 230, LCD_ORANGE, "T", .pOnTouchUp = uiTemperatureMenu },
    { 245, 170, 315, 230, LCD_ORANGE, "Движ", .pOnTouchUp = uiMoveMenu }
};

void uiSetupMenu (xUIEvent_t *pxEvent) {

	uiMenuHandleEventDefault(setupMenu, sizeof(setupMenu)/sizeof(xButton_t), pxEvent);
}

static char coordinatesLabel[] = "Координаты X:1200 Y:1200 Z:1200";

void printDistanceLabel(uint16_t x, uint16_t y) {

    char distanceLabel[8];

    if (moveStep == MOVE_01)
        snprintf(distanceLabel, sizeof(distanceLabel), "0.1 мм");
    else
        snprintf(distanceLabel, sizeof(distanceLabel), "%3d мм", moveStep);

    Lcd_Put_Text(x - (strlen(distanceLabel) << 2), y - 8, 16, distanceLabel, 0xffffu);
}

static xButton_t moveMenu[] = {
    { 5,  20,  80, 80, LCD_GREEN, "X", .pOnTouchUp = uiMoveMenuXAct },
    { 85,  20, 160, 80, LCD_ORANGE, "Y", .pOnTouchUp = uiMoveMenuYAct },
    { 165, 20, 240, 80, LCD_ORANGE, "Z", .pOnTouchUp = uiMoveMenuZAct },
    { 245, 20, 315, 80, LCD_ORANGE, NULL, printDistanceLabel, uiMoveMenuDistance },
    { 20, 100, 300, 130, LCD_BLACK, coordinatesLabel },
    { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
    { 85,  170, 210, 230, LCD_ORANGE, "Ближе" },
    { 215, 170, 315, 230, LCD_ORANGE, "Дальше" }
};

void uiMoveMenu (xUIEvent_t *pxEvent) {

    uiMenuHandleEventDefault(moveMenu, sizeof(moveMenu)/sizeof(xButton_t), pxEvent);
}

void uiMoveMenuXAct(xUIEvent_t *pxEvent) {

    moveMenu[0].color = LCD_GREEN; // X
    moveMenu[1].color = LCD_ORANGE;
    moveMenu[2].color = LCD_ORANGE;

    uiDrawMenu(moveMenu, 3);
    uiToggleParentState(uiMoveMenu);
}

void uiMoveMenuYAct(xUIEvent_t *pxEvent) {

    moveMenu[1].color = LCD_GREEN; // Y
    moveMenu[0].color = LCD_ORANGE;
    moveMenu[2].color = LCD_ORANGE;

    uiDrawMenu(moveMenu, 3);
    uiToggleParentState(uiMoveMenu);
}

void uiMoveMenuZAct(xUIEvent_t *pxEvent) {

    moveMenu[2].color = LCD_GREEN; // Z
    moveMenu[0].color = LCD_ORANGE;
    moveMenu[1].color = LCD_ORANGE;

    uiDrawMenu(moveMenu, 3);
    uiToggleParentState(uiMoveMenu);
}

void uiMoveMenuDistance(xUIEvent_t *pxEvent) {

    switch(moveStep) {
        case MOVE_01:   moveStep = MOVE_1;   break;
        case MOVE_1:    moveStep = MOVE_5;   break;
        case MOVE_5:    moveStep = MOVE_10;  break;
        case MOVE_10:   moveStep = MOVE_100; break;
        default:        moveStep = MOVE_01; break;
    }

    uiDrawMenuItem(&moveMenu[3]);
    uiToggleParentState(uiMoveMenu);
}

static char heater1Temp[] = "27 Экстр1";
static char heater2Temp[] = "28 Экстр2";
static char bedTemp[] = "25 Стол";
static char fanSpeed[] = "146 %";
static xButton_t temperatureMenu[] = {
    { 5,  20,  80, 80, LCD_ORANGE, heater1Temp, .pOnTouchUp = uiE1TempSliderMenu },
    { 85,  20, 160, 80, LCD_ORANGE, heater2Temp },
    { 165, 20, 240, 80, LCD_ORANGE, bedTemp },
    { 245, 20, 315, 80, LCD_ORANGE, fanSpeed },
    { 20, 100, 300, 130, LCD_BLACK, "Пока просто дырка" },
    { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
    { 85,  170, 210, 230, LCD_ORANGE, "Преднагр PLA" },
    { 215, 170, 315, 230, LCD_ORANGE, "Преднагр ABS" }
};

void uiTemperatureMenu (xUIEvent_t *pxEvent) {

    uiMenuHandleEventDefault(temperatureMenu, sizeof(temperatureMenu)/sizeof(xButton_t), pxEvent);
}

void printE1TempSlider(uint16_t x, uint16_t y) {
    uiDrawSlider(115, e1PreheatTemp, MAX_EXTRUDER_TEMP, LCD_DANUBE, LCD_RED);
}

void printE1TempLabel(uint16_t x, uint16_t y) {

    char tempLabel[40];
    snprintf(tempLabel, sizeof(tempLabel),  "%3d C", e1PreheatTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static xButton_t tempSliderMenu[] = {
    { 20, 30, 300, 69, LCD_BLACK, "Температура экструдера 1" },
    { 20, 70, 186, 99, LCD_BLACK, "Установить температуру" },
    { 190, 70, 240, 99, LCD_BLACK, NULL, printE1TempLabel },
    { 5, 107, 315, 143, LCD_BLACK, NULL, printE1TempSlider, .pOnTouchDown = uiE1TempSliderSetMenu },
    { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiTemperatureMenu },
    { 215, 170, 315, 230, LCD_ORANGE, "Установить", .pOnTouchUp = uiTemperatureMenu }
};

void uiE1TempSliderMenu (xUIEvent_t *pxEvent) {

    uiMenuHandleEventDefault(tempSliderMenu, sizeof(tempSliderMenu)/sizeof(xButton_t), pxEvent);
}

void uiE1TempSliderSetMenu (xUIEvent_t *pxEvent) {

    uint16_t temp = MAX_EXTRUDER_TEMP * touchX / 320;

    if (temp != e1PreheatTemp) {
        e1PreheatTemp = temp;

        Lcd_Fill_Rect(190, 70, 240, 99, 0);
        uiDrawMenuItem(&tempSliderMenu[2]);

        Lcd_Fill_Rect(8, 95, 312, 146, 0);
        uiDrawMenuItem(&tempSliderMenu[3]);
    }

    uiToggleParentState(uiE1TempSliderMenu);
}

void printFilChangeTempSlider(uint16_t x, uint16_t y) {
    uiDrawSlider(115, filChangeTemp, MAX_EXTRUDER_TEMP, LCD_DANUBE, LCD_RED);
}

void printFilChangeTempLabel(uint16_t x, uint16_t y) {

    char tempLabel[40];
    snprintf(tempLabel, sizeof(tempLabel),  "%3d C", filChangeTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static xButton_t filChangeMenu[] = {
    { 20, 30, 300, 69, LCD_BLACK, "Смена/извлечение прутка" },
    { 20, 70, 186, 99, LCD_BLACK, "Температура нагрева" },
    { 190, 70, 240, 99, LCD_BLACK, NULL, printFilChangeTempLabel },
    { 5, 107, 315, 143, LCD_BLACK, NULL, printFilChangeTempSlider, .pOnTouchDown = uiFilChangeTempSliderSetMenu },
    { 5,  170, 80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
    { 85,  170, 210, 230, LCD_ORANGE, "Экструдер 1", .pOnTouchUp = uiFilPreheatMenu },
    { 215, 170, 315, 230, LCD_ORANGE, "Экструдер 2", .pOnTouchUp = uiFilPreheatMenu }
};

void uiFilChangeMenu(xUIEvent_t *pxEvent) {

    uiMenuHandleEventDefault(filChangeMenu, sizeof(filChangeMenu)/sizeof(xButton_t), pxEvent);
}

void uiFilChangeTempSliderSetMenu (xUIEvent_t *pxEvent) {

    uint16_t temp = MAX_EXTRUDER_TEMP * touchX / 320;

    if (temp != e1PreheatTemp) {
        e1PreheatTemp = temp;

        Lcd_Fill_Rect(190, 70, 240, 99, 0);
        uiDrawMenuItem(&tempSliderMenu[2]);

        Lcd_Fill_Rect(8, 95, 312, 146, 0);
        uiDrawMenuItem(&tempSliderMenu[3]);
    }

    uiToggleParentState(uiFilChangeMenu);
}

void printFilPreheatTempLabel(uint16_t x, uint16_t y) {

    char tempLabel[40];
    snprintf(tempLabel, sizeof(tempLabel),  "%3d C", /* FIXME */ 666 );
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

void uiFilPreheatMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Нагрев экструдера" },
        { 20, 70, 186, 99, LCD_BLACK, "Температура нагрева" },
        { 190, 70, 240, 99, LCD_BLACK, NULL, printFilPreheatTempLabel },
        { 0, 100, 320, 116, LCD_BLACK, "После нагрева нажмите \"Продолжить\" для" },
        { 0, 116, 320, 132, LCD_BLACK, "начала подачи прутка" },
        { 20, 170, 150, 230, LCD_DANUBE, "Продолжить", .pOnTouchUp = uiFilReplaceMenu },
        { 170, 170, 300, 230, LCD_DANUBE, "Отменить", .pOnTouchUp = uiSetupMenu },
    };

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

void uiFilReplaceMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Экструдер нагрет и пруток извлечен" },
        { 0, 100, 320, 116, LCD_BLACK, "Заправьте новый пруток и нажмите" },
        { 0, 116, 320, 132, LCD_BLACK, "\"Продолжить\" для начала подачи прутка." },
        { 20, 170, 150, 230, LCD_DANUBE, "Продолжить", .pOnTouchUp = uiFilFeedMenu },
        { 170, 170, 300, 230, LCD_DANUBE, "Отменить", .pOnTouchUp = uiSetupMenu },
    };

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

void uiFilFeedMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Идет подача прутка" },
        { 0, 100, 320, 116, LCD_BLACK, "Когда пластик начнет выходить из сопла" },
        { 0, 116, 320, 132, LCD_BLACK, "нажмите \"Завершить\" для остановки" },
        { 0, 132, 320, 148, LCD_BLACK, "подачи и охлаждения экструдера." },
        { 100, 170, 230, 230, LCD_DANUBE, "Завершить", .pOnTouchUp = uiSetupMenu },
    };

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

/*
 * service routines definition
 */

static void uiMediaStateChange(uint16_t event) {

	switch (event) {
	case SDCARD_INSERT:
		f_mount(&sdFileSystem, SPISD_Path, 1);
		break;

	case SDCARD_REMOVE:
		f_mount(NULL, SPISD_Path, 1);
		{
			BYTE poweroff = 0;
			(*SPISD_Driver.disk_ioctl)(0, CTRL_POWER, &poweroff);
		}
		break;

	case USBDRIVE_INSERT:
		f_mount(&usbFileSystem, USBH_Path, 1);
		break;

	case USBDRIVE_REMOVE:
		f_mount(NULL, USBH_Path, 1);
		break;
	}
}

static void uiDrawSlider(uint16_t y, uint32_t pos, uint32_t scale, uint16_t color1, uint16_t color2) {

	uint16_t x0 = 10 + 280 * pos / scale, x1 = x0 + 20;

    Lcd_Line(10 - 2, y - 17, 310 + 2, y - 17, color1);
    Lcd_Line(310 + 2, y - 17, 310 + 2, y + 17, color1);
    Lcd_Line(10 - 2, y - 17, 10 - 2, y + 17, color1);
    Lcd_Line(10 - 2, y + 17, 310 + 2, y + 17, color1);

    Lcd_Fill_Rect(10, y - 15, x0, y + 15, color1);
   	Lcd_Fill_Rect(x0, y - 20, x1, y + 20, color2);
}

static uint32_t pBarScale = 0;
static uint16_t pBarColor = 0xffffu;
static uint32_t pBarProgress = 0;

static void uiDrawProgressBar(uint32_t scale, uint16_t color) {

	Lcd_Fill_Rect(10 - 2, 100 - 2, 310 + 2, 140 + 2, color);
	Lcd_Fill_Rect(10 - 1, 100 - 1, 310 + 1, 140 + 1, 0);

	pBarScale = scale;
	pBarColor = color;
	pBarProgress = 0;
}

static void uiUpdateProgressBar(uint32_t progress) {

	uint16_t x0 = 300 * pBarProgress / pBarScale;
	pBarProgress = progress;
	uint16_t x1 = 300 * pBarProgress / pBarScale;

	if (x0 < x1) {
		Lcd_Fill_Rect(10 + x0, 100, 10 + x1, 140, pBarColor);
	}
}

static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t resetWindow) {

	FIL *pIconFile = NULL;
	BYTE *pBuffer = NULL;

	if (!path)
		return;

	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0052 : 0x0050, x);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0050 : 0x0052, y);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0053 : 0x0051, x + width - 1);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0051 : 0x0053, y + height - 1);

	if ((pIconFile = pvPortMalloc(sizeof(FIL))) != NULL
			&& (pBuffer = pvPortMalloc(_MIN_SS)) != NULL) {

		if (f_open(pIconFile, path, FA_READ) == FR_OK) {

			size_t bytes = (size_t) -1;

			Lcd_Go_XY(x, y);
			Lcd_Com (0x0022);

			HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port,  LCD_RS_Pin,  GPIO_PIN_SET);

			do {
				f_read(pIconFile, pBuffer, _MIN_SS, &bytes);
				if (bytes) {

					for (size_t i=0; i<bytes; i+=2) {

						GPIOE->ODR = *(uint16_t *)&pBuffer[i];
					    GPIOB->BSRR = (uint32_t)LCD_nWR_Pin << 16;
						GPIOB->BSRR = LCD_nWR_Pin;
					}
				}
			} while (bytes);

			f_close(pIconFile);
		}
	}

	if (pIconFile) vPortFree(pIconFile);
	if (pBuffer) vPortFree(pBuffer);

	if (resetWindow) {
		Lcd_Com_Data(0x0050, 0x0000);		  // Window Horizontal RAM Address Start (R50h)
		Lcd_Com_Data(0x0051, 239);			  // Window Horizontal RAM Address End (R51h)
		Lcd_Com_Data(0x0052, 0x0000);		  // Window Vertical RAM Address Start (R52h)
		Lcd_Com_Data(0x0053, 319);			  // Window Vertical RAM Address End (R53h)
	}
}

/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
