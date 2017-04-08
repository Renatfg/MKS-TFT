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

#define READY_PRINT	"Магнум"

uint8_t comm1RxBuf[MAXSTATSIZE+1];

uint16_t moveStep = MOVE_10;
// uint8_t offMode = MANUAL_OFF;
uint8_t connectSpeed = CONNECT_115200;
uint8_t preheatDev = PR_EXTRUDER_1;
uint8_t extrudeDev = EXTRUDER_1;
// uint8_t preheatSelDegree = STEP_1_DEGREE;
uint8_t extrudeDistance = DISTANCE_1;
uint8_t extrudeSelSpeed = SPEED_SLOW;
uint8_t selectedFs = FS_SD;

#define MAX_EXTRUDER_TEMP   270
#define MAX_HEATBED_TEMP    115

volatile float e1TargetTemp  = 0;
volatile float e2TargetTemp  = 0;
volatile float bedTargetTemp = 0;

volatile float e1CurTemp  = 0.0;
volatile float e2CurTemp  = 0.0;
volatile float bedCurTemp = 0.0;

static float newTargetTemp = 0.0;
static uint8_t uiSelExtruder = 1;
volatile uint8_t isPrinting = 0;

volatile float printerX  = 0;
volatile float printerY  = 0;
volatile float printerZ  = 0;
volatile float printerE1 = 0;
volatile float printerE2 = 0;

static uint8_t massStorage = 1;
static uint8_t fileOffset = 0;
static uint8_t fileListEnd = 0;

/*
 * user callback declaration
 */

static void uiInitialize (xUIEvent_t *pxEvent);
static void uiMainMenu   (xUIEvent_t *pxEvent);
static void uiSetupMenu  (xUIEvent_t *pxEvent);

static void uiMoveMenu (xUIEvent_t *pxEvent);
static void  uiMoveMenuXAct(xUIEvent_t *pxEvent);
static void  uiMoveMenuYAct(xUIEvent_t *pxEvent);
static void  uiMoveMenuZAct(xUIEvent_t *pxEvent);
static void  uiMoveMenuDistance(xUIEvent_t *pxEvent);
static void  uiMoveBack(xUIEvent_t *pxEvent);
static void  uiMoveForward(xUIEvent_t *pxEvent);

static void uiTemperatureMenu(xUIEvent_t *pxEvent);
static void  uiE1TempSliderMenu(xUIEvent_t *pxEvent);
static void   uiE1TempSliderSetMenu(xUIEvent_t *pxEvent);
static void    uiE1TempSliderApply(xUIEvent_t *pxEvent);
static void    uiE1TempSliderCancel(xUIEvent_t *pxEvent);

static void  uiE2TempSliderMenu(xUIEvent_t *pxEvent);
static void   uiE2TempSliderSetMenu(xUIEvent_t *pxEvent);
static void    uiE2TempSliderApply(xUIEvent_t *pxEvent);
static void    uiE2TempSliderCancel(xUIEvent_t *pxEvent);

static void  uiBedTempSliderMenu(xUIEvent_t *pxEvent);
static void   uiBedTempSliderSetMenu(xUIEvent_t *pxEvent);
static void    uiBedTempSliderApply(xUIEvent_t *pxEvent);
static void    uiBedTempSliderCancel(xUIEvent_t *pxEvent);

static void uiFilChangeMenu(xUIEvent_t *pxEvent);
static void  uiFilChangeTempSliderSetMenu(xUIEvent_t *pxEvent);

static void uiFilPreheatMenu(xUIEvent_t *pxEvent);
static void  uiFilReplaceMenu(xUIEvent_t *pxEvent);
static void  uiFilFeedMenu(xUIEvent_t *pxEvent);

static void uiFileSelectMenu(xUIEvent_t *pxEvent);
static void uiFileSelectUSB(xUIEvent_t *pxEvent);
static void uiFileSelectSD(xUIEvent_t *pxEvent);
static void uiFileSelectOffsetPlus(xUIEvent_t *pxEvent);
static void uiFileSelectOffsetMinus(xUIEvent_t *pxEvent);

static void  uiFilePrintMenu(xUIEvent_t *pxEvent);
static void  uiPrintFilChangeMenu(xUIEvent_t *pxEvent);
static void  uiPrintFilFeedMenu(xUIEvent_t *pxEvent);
static void  uiE1PrintSliderMenu(xUIEvent_t *pxEvent);
static void  uiE1PrintSliderSetMenu(xUIEvent_t *pxEvent);
static void  uiE2PrintSliderMenu(xUIEvent_t *pxEvent);
static void  uiE2PrintSliderSetMenu(xUIEvent_t *pxEvent);
static void  uiBedPrintSliderMenu(xUIEvent_t *pxEvent);
static void  uiBedPrintSliderSetMenu(xUIEvent_t *pxEvent);

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

__STATIC_INLINE void uiToggleRedrawParentState(void (*volatile next) (xUIEvent_t *pxEvent)) {
	processEvent = next;
	xUIEvent_t event = { REDRAW_EVENT };
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

            xUIEvent_t uiEvent = { REDRAW_EVENT };
            xQueueSendToFront(xUIEventQueue, &uiEvent, 1000);
			break;

//        case SHOW_STATUS:
//            Lcd_Fill_Rect(0, LCD_MAX_Y - 9, LCD_MAX_X, LCD_MAX_Y, 0);
//            Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, comm1RxBuf, 0xffffu);
//            break;

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

                            uiShortBeep();
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

static void uiInitialize (xUIEvent_t *pxEvent) {

    Lcd_Init(LCD_LANDSCAPE_CL);
    Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));

    osDelay(500);

    f_mount(&flashFileSystem, SPIFL_Path, 1);  // mount flash
//  f_mount(&sdFileSystem, SPISD_Path, 1);      // mount sd card, mount usb later

	uiNextState(uiMainMenu);
}

static void uiMainMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 110, LCD_BLACK, "МАГНУМ" },
        { 0, 110, 320, 150, LCD_BLACK, "Иконки и статусы температуры" },
        { 20, 170, 150, 230, LCD_DANUBE, "Печать", .pOnTouchUp = uiFileSelectMenu},
        { 170, 170, 300, 230, LCD_DANUBE, "Настройки", .pOnTouchUp = uiSetupMenu },
    };

    switch (pxEvent->ucEventID) {
    case INIT_EVENT:
        xTimerStop(xIdleTimer, 10);
        xTimerStop(xM114Timer, 10);
        isPrinting = 0;
        break;

    case REDRAW_EVENT:  // usb mounted bug workaround
        uiDrawMenu(menu, sizeof(menu)/sizeof(xButton_t));
        break;

    default:
        break;
    }

	uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

void vIdleTimerCallback( TimerHandle_t xTimer ) {

    uiShortBeep();
	uiNextState(uiMainMenu);
}

static char currentIPLabel[30] = "Текущий IP: 192.168.0.253";

static void uiSetupMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
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

    switch (pxEvent->ucEventID) {
    case INIT_EVENT:
        xTimerReset(xIdleTimer, 10);
        xTimerStop(xM114Timer, 10);
        break;

    default:
        break;
    }

	uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void printDistanceLabel(uint16_t x, uint16_t y) {

    char distanceLabel[8];

    if (moveStep == MOVE_01)
        snprintf(distanceLabel, sizeof(distanceLabel), "0.1 мм");
    else
        snprintf(distanceLabel, sizeof(distanceLabel), "%3d мм", moveStep);

    Lcd_Put_Text(x - (strlen(distanceLabel) << 2), y - 8, 16, distanceLabel, 0xffffu);
}

static void printCoordinatesLabel(uint16_t x, uint16_t y) {

    char coordinatesLabel[40];

    snprintf(coordinatesLabel, sizeof(coordinatesLabel), "Координаты X:%3.1f Y:%3.1f Z:%3.1f",
             printerX, printerY, printerZ);

    Lcd_Put_Text(x - (strlen(coordinatesLabel) << 2), y - 8, 16, coordinatesLabel, 0xffffu);
}

static xButton_t moveMenu[] = {
    { 5,  20,  80, 80, LCD_GREEN, "X", .pOnTouchUp = uiMoveMenuXAct },
    { 85,  20, 160, 80, LCD_ORANGE, "Y", .pOnTouchUp = uiMoveMenuYAct },
    { 165, 20, 240, 80, LCD_ORANGE, "Z", .pOnTouchUp = uiMoveMenuZAct },
    { 245, 20, 315, 80, LCD_ORANGE, NULL, printDistanceLabel, uiMoveMenuDistance },
    { 20, 100, 300, 130, LCD_BLACK, NULL, printCoordinatesLabel },
    { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
    { 85,  170, 210, 230, LCD_ORANGE, "Ближе", .pOnTouchUp = uiMoveBack },
    { 215, 170, 315, 230, LCD_ORANGE, "Дальше", .pOnTouchUp = uiMoveForward }
};

static void uiMoveMenu (xUIEvent_t *pxEvent) {

    switch (pxEvent->ucEventID) {
    case REDRAW_EVENT:

        uiDrawMenuItem(&moveMenu[0]);
        uiDrawMenuItem(&moveMenu[1]);
        uiDrawMenuItem(&moveMenu[2]);
        uiDrawMenuItem(&moveMenu[3]);
        uiDrawMenuItem(&moveMenu[4]);
        break;

    case INIT_EVENT:
    {
        xCommEvent_t event = { "M114\n" };
        xQueueSendToBack(xPCommEventQueue, &event, 1000);

        xTimerReset(xIdleTimer, 10);
        if( xTimerIsTimerActive( xIdleTimer ) == pdFALSE ) {
            xTimerReset(xIdleTimer, 10);

        }
        if(xTimerStart(xM114Timer, 10) != pdPASS) // TODO: only start if connected to printer
        {
            /* The timer could not be set into the Active
            state. */
        }
    }

    default:
        uiMenuHandleEventDefault(moveMenu, sizeof(moveMenu)/sizeof(xButton_t), pxEvent);
    }
}

static void uiMoveMenuXAct(xUIEvent_t *pxEvent) {

    moveMenu[0].color = LCD_GREEN; // X
    moveMenu[1].color = LCD_ORANGE;
    moveMenu[2].color = LCD_ORANGE;

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiMoveMenu);
}

static void uiMoveMenuYAct(xUIEvent_t *pxEvent) {

    moveMenu[1].color = LCD_GREEN; // Y
    moveMenu[0].color = LCD_ORANGE;
    moveMenu[2].color = LCD_ORANGE;

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiMoveMenu);
}

static void uiMoveMenuZAct(xUIEvent_t *pxEvent) {

    moveMenu[2].color = LCD_GREEN; // Z
    moveMenu[0].color = LCD_ORANGE;
    moveMenu[1].color = LCD_ORANGE;

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiMoveMenu);
}

static void uiMoveMenuDistance(xUIEvent_t *pxEvent) {

    switch(moveStep) {
        case MOVE_01:   moveStep = MOVE_1;   break;
        case MOVE_1:    moveStep = MOVE_5;   break;
        case MOVE_5:    moveStep = MOVE_10;  break;
//      case MOVE_10:   moveStep = MOVE_100; break;
        default:        moveStep = MOVE_01; break;
    }


    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiMoveMenu);
}

void uiMoveBack(xUIEvent_t *pxEvent) {

    uint8_t axis = 'X';
    if (moveMenu[1].color == LCD_GREEN) axis = 'Y';
    else if (moveMenu[2].color == LCD_GREEN) axis = 'Z';

    xCommEvent_t event = { "G91\n" };
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    if (moveStep == MOVE_01)
        snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "G1 %c-0.1\n", axis);
    else
        snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "G1 %c-%d\n", axis, moveStep);

    xTimerReset(xIdleTimer, 10);
    xQueueSendToBack(xPCommEventQueue, &event, 1000);
    uiToggleRedrawParentState(uiMoveMenu);
}

void uiMoveForward(xUIEvent_t *pxEvent) {

    uint8_t axis = 'X';
    if (moveMenu[1].color == LCD_GREEN) axis = 'Y';
    else if (moveMenu[2].color == LCD_GREEN) axis = 'Z';

    xCommEvent_t event = { "G91\n" };
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    if (moveStep == MOVE_01)
        snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "G1 %c0.1\n", axis);
    else
        snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "G1 %c%d\n", axis, moveStep);

    xTimerReset(xIdleTimer, 10);
    xQueueSendToBack(xPCommEventQueue, &event, 1000);
    uiToggleRedrawParentState(uiMoveMenu);
}

static void printNewTempSlider(uint16_t x, uint16_t y) {

    if (0xffu == uiSelExtruder)
        uiDrawSlider(y, (int) newTargetTemp, MAX_HEATBED_TEMP, LCD_DANUBE, LCD_RED);
    else
        uiDrawSlider(y, (int) newTargetTemp, MAX_EXTRUDER_TEMP, LCD_DANUBE, LCD_RED);
}

static void printE1TempLabel(uint16_t x, uint16_t y) {

    char tempLabel[8];
    snprintf(tempLabel, sizeof(tempLabel),  "%3.1f C", e1CurTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static void printE2TempLabel(uint16_t x, uint16_t y) {

    char tempLabel[8];
    snprintf(tempLabel, sizeof(tempLabel),  "%3.1f C", e2CurTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static void printBedTempLabel(uint16_t x, uint16_t y) {

    char tempLabel[8];
    snprintf(tempLabel, sizeof(tempLabel),  "%3.1f C", bedCurTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static void printNewTempLabel(uint16_t x, uint16_t y) {

    char tempLabel[8];
    snprintf(tempLabel, sizeof(tempLabel),  "%3.1f C", newTargetTemp);
    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static void uiTemperatureMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 5,  20,  80, 80, LCD_ORANGE, NULL, printE1TempLabel, .pOnTouchUp = uiE1TempSliderMenu },
        { 85,  20, 160, 80, LCD_ORANGE, NULL, printE2TempLabel, .pOnTouchUp = uiE2TempSliderMenu },
        { 165, 20, 240, 80, LCD_ORANGE, NULL, printBedTempLabel, .pOnTouchUp = uiBedTempSliderMenu  },
        { 245, 20, 315, 80, LCD_ORANGE, "146 %" },
        { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
        { 85,  170, 200, 230, LCD_DANUBE, "Преднагрев PLA" },
        { 205, 170, 315, 230, LCD_DANUBE, "Преднагрев ABS" }
    };

    if (pxEvent->ucEventID == INIT_EVENT) {
        xTimerReset(xIdleTimer, 10);
    }

    if (pxEvent->ucEventID == REDRAW_EVENT)
    {
        uiDrawMenu(menu, 4);
    }
    else
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void printHeatDevLabel(uint16_t x, uint16_t y) {

    char tempLabel[50];

    if (0xffu == uiSelExtruder)
        snprintf(tempLabel, sizeof(tempLabel),  "Температура горячего стола");
    else
        snprintf(tempLabel, sizeof(tempLabel),  "Температура экструдера %d", uiSelExtruder);

    Lcd_Put_Text(x - (strlen(tempLabel) << 2), y - 8, 16, tempLabel, 0xffffu);
}

static void uiTempSliderMenu (
        xUIEvent_t *pxEvent,
        eventProcessor_t back,
        eventProcessor_t forward,
        eventProcessor_t sliderSet,
        uint8_t canCancel,
        eventProcessor_t cancel) {

    xButton_t menu[] = {
        { 20, 30, 300, 69, LCD_BLACK, NULL, printHeatDevLabel },
        { 20, 70, 186, 99, LCD_BLACK, "Установить температуру" },
        { 200, 70, 260, 99, LCD_BLACK, NULL, printNewTempLabel },
        { 5, 100, 315, 150, LCD_BLACK, NULL, printNewTempSlider,
                .pOnTouchDown = sliderSet },
        { 5,  170,  80, 230, LCD_RED, "Назад", .pOnTouchUp = back },
        { 85, 170,  210, 230, (canCancel ? LCD_DANUBE : LCD_BLACK),
                (canCancel ? "Выключить" : NULL), .pOnTouchUp = cancel },
        { 215, 170, 315, 230, LCD_ORANGE, "Установить", .pOnTouchUp = forward }
    };

    uint16_t temp = 0;

    switch (pxEvent->ucEventID) {
    case REDRAW_EVENT:
        if (0xffu == uiSelExtruder)
            temp = MAX_HEATBED_TEMP * (touchX - 5) / 310;
        else
            temp = MAX_EXTRUDER_TEMP * (touchX - 5) / 310;

        if (temp != newTargetTemp) {
            newTargetTemp = temp;
            uiDrawMenuItem(&menu[2]);
            uiDrawMenuItem(&menu[3]);
        }
        break;

    case INIT_EVENT:
        xTimerReset(xIdleTimer, 10);
        if (0xffu == uiSelExtruder)
            newTargetTemp = bedTargetTemp;
        else
            newTargetTemp = (uiSelExtruder == 1) ? e1TargetTemp : e2TargetTemp;

    default:
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
        break;
    }
}

static void uiE1TempSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 1;
    uiTempSliderMenu(pxEvent, uiTemperatureMenu, uiE1TempSliderApply, uiE1TempSliderSetMenu, 1, uiE1TempSliderCancel);
}

static void uiE1TempSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiE1TempSliderMenu);
}

static void uiE1TempSliderApply(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M104 T0 S%3.0f\n", newTargetTemp);
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiE1TempSliderCancel(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M104 T0 S0\n");
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiE2TempSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 2;
    uiTempSliderMenu(pxEvent, uiTemperatureMenu, uiE2TempSliderApply, uiE2TempSliderSetMenu, 1, uiE2TempSliderCancel);
}

static void uiE2TempSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiE2TempSliderMenu);
}

static void uiE2TempSliderApply(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M104 T1 S%3.0f\n", newTargetTemp);
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiE2TempSliderCancel(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M104 T1 S0\n");
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiBedTempSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 0xffu;
    uiTempSliderMenu(pxEvent, uiTemperatureMenu, uiBedTempSliderApply, uiBedTempSliderSetMenu, 1, uiBedTempSliderCancel);
}

static void uiBedTempSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiBedTempSliderMenu);
}

static void uiBedTempSliderApply(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M140 S%3.0f\n", newTargetTemp);
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiBedTempSliderCancel(xUIEvent_t *pxEvent) {

    xCommEvent_t event;
    snprintf((char *)event.ucCmd, sizeof(event.ucCmd), "M140 S0\n");
    xQueueSendToBack(xPCommEventQueue, &event, 1000);

    xTimerReset(xIdleTimer, 10);
    uiNextState(uiTemperatureMenu);
}

static void uiFilChangeMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 20, 30, 300, 69, LCD_BLACK, "Смена/извлечение прутка" },
        { 20, 70, 186, 99, LCD_BLACK, "Температура нагрева" },
        { 200, 70, 260, 99, LCD_BLACK, NULL, printNewTempLabel },
        { 5, 100, 315, 150, LCD_BLACK, NULL, printNewTempSlider, .pOnTouchDown = uiFilChangeTempSliderSetMenu },
        { 5,  170, 80, 230, LCD_RED, "Назад", .pOnTouchUp = uiSetupMenu },
        { 85,  170, 210, 230, LCD_DANUBE, "Экструдер 1", .pOnTouchUp = uiFilPreheatMenu },
        { 215, 170, 315, 230, LCD_DANUBE, "Экструдер 2", .pOnTouchUp = uiFilPreheatMenu }
    };

    uint16_t temp = 0;

    switch (pxEvent->ucEventID) {
    case REDRAW_EVENT:
        temp = MAX_EXTRUDER_TEMP * touchX / 320;

        if (temp != newTargetTemp) {
            newTargetTemp = temp;
            uiDrawMenuItem(&menu[2]);
            uiDrawMenuItem(&menu[3]);
        }
        break;

    case INIT_EVENT:
        /* TODO: check material */
        xTimerReset(xIdleTimer, 10);
        newTargetTemp = 220;

    default:
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
        break;
    }
}

static void uiFilChangeTempSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiFilChangeMenu);
}

static void printFilPreheatTempLabel(uint16_t x, uint16_t y) {

    if (1 == uiSelExtruder)
        printE1TempLabel(x, y);
    else
        printE2TempLabel(x, y);
}

static void uiFilPreheatMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Нагрев экструдера" },
        { 20, 70, 186, 99, LCD_BLACK, "Температура нагрева" },
        { 190, 70, 240, 99, LCD_BLACK, NULL, printFilPreheatTempLabel },
        { 0, 100, 320, 116, LCD_BLACK, "После нагрева нажмите \"Продолжить\" для" },
        { 0, 116, 320, 132, LCD_BLACK, "начала подачи прутка" },
        { 20, 170, 150, 230, LCD_DANUBE, "Продолжить", .pOnTouchUp = uiFilReplaceMenu },
        { 170, 170, 300, 230, LCD_DANUBE, "Отменить", .pOnTouchUp = uiSetupMenu },
    };

    if (pxEvent->ucEventID == INIT_EVENT) {
        xTimerStop(xIdleTimer, 10);
    }

    if (pxEvent->ucEventID == REDRAW_EVENT)
    {
        uiDrawMenuItem(&menu[2]);
    }
    else
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiFilReplaceMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Экструдер нагрет и пруток извлечен" },
        { 0, 100, 320, 116, LCD_BLACK, "Заправьте новый пруток и нажмите" },
        { 0, 116, 320, 132, LCD_BLACK, "\"Продолжить\" для начала подачи прутка." },
        { 20, 170, 150, 230, LCD_DANUBE, "Продолжить", .pOnTouchUp = uiFilFeedMenu },
        { 170, 170, 300, 230, LCD_DANUBE, "Отменить", .pOnTouchUp = uiSetupMenu },
    };

    if (pxEvent->ucEventID == INIT_EVENT) {
        xTimerStop(xIdleTimer, 10);
    }

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiFilFeedMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Идет подача прутка" },
        { 0, 100, 320, 116, LCD_BLACK, "Когда пластик начнет выходить из сопла" },
        { 0, 116, 320, 132, LCD_BLACK, "нажмите \"Завершить\" для остановки" },
        { 0, 132, 320, 148, LCD_BLACK, "подачи и охлаждения экструдера." },
        { 100, 170, 230, 230, LCD_DANUBE, "Завершить", .pOnTouchUp = uiSetupMenu },
    };

    if (pxEvent->ucEventID == INIT_EVENT) {
        xTimerStop(xIdleTimer, 10);
    }

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiFileSelectMenu(xUIEvent_t *pxEvent) {

	char fname_table[4][26] = { "", "", "", "" };

    xButton_t menu[] = {
        { 5,  10, 80, 70, LCD_RED, "Назад", .pOnTouchUp = uiMainMenu },
        { 90,  10, 190, 70, LCD_DANUBE, "USB", .pOnTouchUp = uiFileSelectUSB },
        { 200, 10, 315, 70, LCD_DANUBE, "SD карта", .pOnTouchUp = uiFileSelectSD },
        { 5, 80, 235, 119, LCD_GRAY20,  fname_table[0], .pOnTouchUp = uiFilePrintMenu },
        { 5, 120, 235, 159, LCD_BLACK,  fname_table[1], .pOnTouchUp = uiFilePrintMenu },
        { 5, 160, 235, 199, LCD_GRAY20, fname_table[2], .pOnTouchUp = uiFilePrintMenu },
        { 5, 200, 235, 239, LCD_BLACK,  fname_table[3], .pOnTouchUp = uiFilePrintMenu },
        { 245, 80, 315, 149, LCD_RED, "Вверх", .pOnTouchUp = uiFileSelectOffsetMinus },
        { 245, 170, 315, 239, LCD_RED, "Вниз", .pOnTouchUp = uiFileSelectOffsetPlus }
    };

    DIR dir;
    FRESULT res;
    int idx = 0;

    char cwd[10];
    snprintf(cwd, sizeof(cwd), "%c:/models", massStorage ? '2' : '1');

    if (pxEvent->ucEventID == INIT_EVENT) {

        xTimerReset(xIdleTimer, 10);
        fileOffset = 0;
    }

    if (pxEvent->ucEventID == INIT_EVENT || pxEvent->ucEventID == REDRAW_EVENT) {

        res = f_opendir(&dir, cwd); /* Open the directory */
        if (res == FR_OK) {

            FILINFO *pFno;

            if ((pFno = pvPortMalloc(sizeof(FILINFO))) != NULL) {

                while (FR_OK == f_readdir(&dir, pFno) && pFno->fname[0]) {

                    if (pFno->fattrib & AM_DIR)
                        continue;

                    if (idx - fileOffset >= 0 && idx < (4 + fileOffset)) {
                        strncpy(fname_table[idx - fileOffset], pFno->fname, 25);
                    }

                    idx++;
                }

                fileListEnd = idx;
                vPortFree(pFno);
            }
            f_closedir(&dir);
        }
    }

    if (pxEvent->ucEventID == REDRAW_EVENT) {

        uiDrawMenuItem(&menu[3]);
        uiDrawMenuItem(&menu[4]);
        uiDrawMenuItem(&menu[5]);
        uiDrawMenuItem(&menu[6]);
    }
    else
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiFileSelectOffsetPlus(xUIEvent_t *pxEvent) {

    if (fileOffset + 4 < fileListEnd) fileOffset++;

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiFileSelectMenu);
}

static void uiFileSelectOffsetMinus(xUIEvent_t *pxEvent) {

    if (fileOffset) fileOffset--;

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiFileSelectMenu);
}

static void uiFileSelectUSB(xUIEvent_t *pxEvent) {

    massStorage = 1;
    fileOffset = 0;

    if (!usbFileSystem.fs_type)
        f_mount(&usbFileSystem, USBH_Path, 1);

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiFileSelectMenu);
}

static void uiFileSelectSD(xUIEvent_t *pxEvent) {

    massStorage = 0;
    fileOffset = 0;

    if (!sdFileSystem.fs_type)
        f_mount(&sdFileSystem, SPISD_Path, 1);

    xTimerReset(xIdleTimer, 10);
    uiToggleRedrawParentState(uiFileSelectMenu);
}

static void uiFilePrintMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 5,  20,  80, 80, LCD_ORANGE, NULL, printE1TempLabel, .pOnTouchUp = uiE1PrintSliderMenu },
        { 85,  20, 160, 80, LCD_ORANGE, NULL, printE2TempLabel, .pOnTouchUp = uiE2PrintSliderMenu },
        { 165, 20, 240, 80, LCD_ORANGE, NULL, printBedTempLabel, .pOnTouchUp = uiBedPrintSliderMenu },
        { 245, 20, 315, 80, LCD_ORANGE, "146 %" },
        { 20, 100, 300, 130, LCD_BLACK, "Пока просто дырка" },
        { 5,  170,  80, 230, LCD_DANUBE, "Пауза" },
        { 85,  170, 210, 230, LCD_DANUBE, "Сменить пруток", .pOnTouchUp =  uiPrintFilChangeMenu},
        { 215, 170, 315, 230, LCD_RED, "Отменить", .pOnTouchUp = uiMainMenu }
    };

    switch (pxEvent->ucEventID) {
    case REDRAW_EVENT:
        uiDrawMenu(menu, 3);
        break;

    case INIT_EVENT:
        while (xTimerStop(xIdleTimer, 10) == pdFAIL) ;
        isPrinting = 1;

    default:
        uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
        break;
    }
}

static void uiPrintFilChangeMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Пруток извлекается..." },
        { 0, 100, 320, 116, LCD_BLACK, "Подождите пока старый пруток" },
        { 0, 116, 320, 132, LCD_BLACK, "будет извлечен из экструдера." },
        { 0, 132, 320, 148, LCD_BLACK, "Удалите его и заправьте новый." },
        { 100, 170, 230, 230, LCD_DANUBE, "Далее", .pOnTouchUp = uiPrintFilFeedMenu },
    };

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiPrintFilFeedMenu(xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 70, LCD_BLACK, "Идет подача прутка" },
        { 0, 100, 320, 116, LCD_BLACK, "Когда пластик начнет выходить из сопла" },
        { 0, 116, 320, 132, LCD_BLACK, "нажмите \"Продолжить\" для возвращения" },
        { 0, 132, 320, 148, LCD_BLACK, "к печати." },
        { 100, 170, 230, 230, LCD_DANUBE, "Продолжить", .pOnTouchUp = uiFilePrintMenu  },
    };

    uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

static void uiE1PrintSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 1;
    uiTempSliderMenu(pxEvent, uiFilePrintMenu, uiFilePrintMenu, uiE1PrintSliderSetMenu, 0, NULL);
}

static void uiE1PrintSliderSetMenu(xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiE1PrintSliderMenu);
}

static void uiE2PrintSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 2;
    uiTempSliderMenu(pxEvent, uiFilePrintMenu, uiFilePrintMenu, uiE2PrintSliderSetMenu, 0, NULL);
}

static void uiE2PrintSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiE2PrintSliderMenu);
}

static void uiBedPrintSliderMenu (xUIEvent_t *pxEvent) {

    uiSelExtruder = 0xffu;
    uiTempSliderMenu(pxEvent, uiFilePrintMenu, uiFilePrintMenu, uiBedPrintSliderSetMenu, 0, NULL);
}

static void uiBedPrintSliderSetMenu (xUIEvent_t *pxEvent) {

    uiToggleRedrawParentState(uiBedPrintSliderMenu);
}

/*
 * service routines definition
 */

static void uiMediaStateChange(uint16_t event) {

	BYTE power;

	switch (event) {
	case SDCARD_INSERT:
		f_mount(&sdFileSystem, SPISD_Path, 1);
		break;

	case SDCARD_REMOVE:
		f_mount(NULL, SPISD_Path, 1);

		power = 0;
        (*SPISD_Driver.disk_ioctl)(0, CTRL_POWER, &power);
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

	uint16_t x0 = 15 + 270 * pos / scale, x1 = x0 + 20;

    Lcd_Line(15 - 2, y - 17, 305 + 2, y - 17, color1);
    Lcd_Line(305 + 2, y - 17, 305 + 2, y + 17, color1);
    Lcd_Line(15 - 2, y - 17, 15 - 2, y + 17, color1);
    Lcd_Line(15 - 2, y + 17, 305 + 2, y + 17, color1);

    Lcd_Fill_Rect(15, y - 15, x0, y + 15, color1);
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
