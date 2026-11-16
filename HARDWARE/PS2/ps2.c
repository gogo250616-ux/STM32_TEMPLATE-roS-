#include "ps2.h"
#include "chassis_control.h"
#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "bsp_ps2.h"
#include "rosnfcusart.h"
#include <stdio.h>

#define PS2_DEADZONE       20
#define PS2_DEFAULT_SPEED  50
#define PS2_MIN_SPEED      10
#define PS2_MAX_SPEED      100
#define PS2_SPEED_STEP     10
/* 045E:028E decoder: report byte 3 bit 7 = Y, bit 4 = A. */
#define PS2_SPEED_UP_KEY   PS2KEY_1GREEN
#define PS2_SPEED_DOWN_KEY PS2KEY_3BLUE
/* Change to -1 if physical left/right motor wiring reverses steering. */
#define PS2_TURN_DIRECTION 1

extern USBH_HandleTypeDef hUsbHostFS;
extern PS2INFO_t ps2_info;
extern USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost);

static uint8_t usb_ps2_ready;
static uint8_t manual_mode;
static uint8_t previous_start;
static uint8_t previous_select;
static uint8_t previous_y, previous_a;
static int16_t speed_limit = PS2_DEFAULT_SPEED;
static uint32_t report_count, last_report_ms;
static uint32_t usb_recovery_ms, usb_recovery_count;

void PS2Control_Set(void) { usb_ps2_ready = 1U; }

void PS2Control_Reset(void)
{
    usb_ps2_ready = 0U;
    previous_start = 0U;
    previous_select = 0U;
    previous_y = previous_a = 0U;
    speed_limit = PS2_DEFAULT_SPEED;
    if (manual_mode)
    {
        manual_mode = 0U;
        ChassisControl_EmergencyStop();
        ChassisControl_SetManualMode(0U);
    }
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    int16_t forward, turn, right_turn, left, right;
    uint8_t start, select, y, a;
    if (USBH_HID_PS2_Decode(phost) != USBH_OK) return;
    ++report_count;
    last_report_ms = HAL_GetTick();

    start = ps2_info.getKeyState(PS2KEY_START);
    select = ps2_info.getKeyState(PS2KEY_SELECT);
    if (start && !previous_start)
    {
        manual_mode = 1U;
        ChassisControl_SetManualMode(1U);
    }
    if (select && !previous_select)
    {
        manual_mode = 0U;
        ChassisControl_EmergencyStop();
        ChassisControl_SetManualMode(0U);
    }
    previous_start = start;
    previous_select = select;
    y = ps2_info.getKeyState(PS2_SPEED_UP_KEY);
    a = ps2_info.getKeyState(PS2_SPEED_DOWN_KEY);
    /* Rising edge only; holding or pressing both buttons does not repeat. */
    if (manual_mode && !(y && a))
    {
        if (y && !previous_y && speed_limit < PS2_MAX_SPEED)
            speed_limit += PS2_SPEED_STEP;
        if (a && !previous_a && speed_limit > PS2_MIN_SPEED)
            speed_limit -= PS2_SPEED_STEP;
    }
    previous_y = y;
    previous_a = a;
    if (!manual_mode) return;

    forward = (int16_t)128 - (int16_t)ps2_info.LY;
    /* Left stick: forward/back and steering. Right stick: steering fallback. */
    turn = (int16_t)ps2_info.LX - (int16_t)128;
    right_turn = (int16_t)ps2_info.RX - (int16_t)128;
    if (forward > -PS2_DEADZONE && forward < PS2_DEADZONE) forward = 0;
    if (turn > -PS2_DEADZONE && turn < PS2_DEADZONE) turn = 0;
    if (right_turn > -PS2_DEADZONE && right_turn < PS2_DEADZONE) right_turn = 0;
    if (turn == 0) turn = right_turn;
    forward = (int16_t)(forward * speed_limit / 127);
    turn = (int16_t)(turn * speed_limit / 127 * PS2_TURN_DIRECTION);

    /* Measured convention: forward = left positive, right negative. */
    left = forward + turn;
    right = -forward + turn;
    /* Preserve steering ratio while respecting the selected wheel limit. */
    {
        int16_t peak = left < 0 ? -left : left;
        int16_t right_abs = right < 0 ? -right : right;
        if (right_abs > peak) peak = right_abs;
        if (peak > speed_limit) {
            left = (int16_t)(left * speed_limit / peak);
            right = (int16_t)(right * speed_limit / peak);
        }
    }
    ChassisControl_SetPs2Command((int8_t)left, (int8_t)right);
}

void PS2_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    usb_ps2_ready = 0U;
    manual_mode = 0U;
    previous_start = previous_select = previous_y = previous_a = 0U;
    speed_limit = PS2_DEFAULT_SPEED;

    /* Pin-level check before AF10 takes ownership. With a full-speed USB
       device already inserted, DP should be 1 and DM should be 0. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_Delay(20U);
    printf("USB PIN TEST DM_PA11=%u DP_PA12=%u IDR=0x%04X\r\n",
           (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11),
           (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12),
           (unsigned)(GPIOA->IDR & 0xFFFFU));

    MX_USB_HOST_Init();
    usb_recovery_ms = HAL_GetTick();
}

void PS2_Task(void)
{
    static uint32_t last_log;
    HID_HandleTypeDef *hid;
    volatile uint32_t *hprt_reg =
        (volatile uint32_t *)((uint32_t)USB_OTG_FS +
                              USB_OTG_HOST_PORT_BASE);
    (void)USBH_Process(&hUsbHostFS);
    if (!usb_ps2_ready && manual_mode) PS2Control_Reset();

    /* Recover an OTG host that remained in HOST_IDLE and missed device
       connection. All calls run in the main loop, never in an IRQ/callback. */
    if (!usb_ps2_ready && hUsbHostFS.gState == HOST_IDLE &&
        (uint32_t)(HAL_GetTick() - usb_recovery_ms) >= 5000U)
    {
        usb_recovery_ms = HAL_GetTick();
        (void)USBH_Stop(&hUsbHostFS);
        (void)USBH_DeInit(&hUsbHostFS);
        __HAL_RCC_USB_OTG_FS_FORCE_RESET();
        HAL_Delay(2U);
        __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
        MX_USB_HOST_Init();
        usb_recovery_count++;
        printf("PS2 USB host recovery=%lu\r\n",
               (unsigned long)usb_recovery_count);
    }
    if ((uint32_t)(HAL_GetTick() - last_log) >= 1000U)
    {
        last_log = HAL_GetTick();
        hid = hUsbHostFS.pActiveClass ?
            (HID_HandleTypeDef *)hUsbHostFS.pActiveClass->pData : NULL;
        printf("PS2 ready=%u host=%u hid=%u timer=%lu count=%lu age=%lu recover=%lu LX=%u LY=%u RX=%u RY=%u START=%u SELECT=%u manual=%u L=%d R=%d Y=%u A=%u limit=%d%%\r\n",
            usb_ps2_ready, (unsigned)hUsbHostFS.gState,
            hid ? (unsigned)hid->state : 255U,
            (unsigned long)hUsbHostFS.Timer, (unsigned long)report_count,
            (unsigned long)(HAL_GetTick()-last_report_ms),
            (unsigned long)usb_recovery_count,
            ps2_info.LX, ps2_info.LY, ps2_info.RX, ps2_info.RY,
            previous_start, previous_select, manual_mode, leftSpeedSet, rightSpeedSet,
            previous_y, previous_a, (int)speed_limit);
        printf("USB REG HPRT=0x%08lX PCSTS=%u PENA=%u PPWR=%u PLSTS=%lu GINTSTS=0x%08lX GINTMSK=0x%08lX GAHBCFG=0x%08lX GUSBCFG=0x%08lX GCCFG=0x%08lX AHB2ENR=0x%08lX\r\n",
            (unsigned long)(*hprt_reg),
            (unsigned)((*hprt_reg & USB_OTG_HPRT_PCSTS) != 0U),
            (unsigned)((*hprt_reg & USB_OTG_HPRT_PENA) != 0U),
            (unsigned)((*hprt_reg & USB_OTG_HPRT_PPWR) != 0U),
            (unsigned long)((*hprt_reg & USB_OTG_HPRT_PLSTS) >>
                            USB_OTG_HPRT_PLSTS_Pos),
            (unsigned long)USB_OTG_FS->GINTSTS,
            (unsigned long)USB_OTG_FS->GINTMSK,
            (unsigned long)USB_OTG_FS->GAHBCFG,
            (unsigned long)USB_OTG_FS->GUSBCFG,
            (unsigned long)USB_OTG_FS->GCCFG,
            (unsigned long)RCC->AHB2ENR);
    }
}

uint8_t PS2_IsManualMode(void) { return manual_mode; }
