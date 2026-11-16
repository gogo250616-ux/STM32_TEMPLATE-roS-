#include "chassis_control.h"
#include "can.h"
#include "rosnfcusart.h"

#define DRIVE_CAN_ID          0x603U
#define ROS_COMMAND_TIMEOUT   1000U
#define PS2_COMMAND_TIMEOUT   150U
#define FORWARD_TEST_DELAY_MS 3000U
#define FORWARD_TEST_SPEED    10

static int8_t ros_left, ros_right, ps2_left, ps2_right;
static uint32_t ros_update_ms, ps2_update_ms;
static uint8_t manual_mode, force_stop;
#if CHASSIS_FORWARD_TEST_MODE
static uint32_t forward_test_start_ms;
#endif

void ChassisControl_Init(void)
{
    ros_update_ms = HAL_GetTick();
    ps2_update_ms = HAL_GetTick();
    manual_mode = 0U;
    force_stop = 1U;
#if CHASSIS_FORWARD_TEST_MODE
    forward_test_start_ms = HAL_GetTick();
#endif
}

void ChassisControl_SetRosCommand(int8_t left, int8_t right)
{
    ros_left = left; ros_right = right; ros_update_ms = HAL_GetTick();
    if (!manual_mode) force_stop = 0U;
}

void ChassisControl_SetPs2Command(int8_t left, int8_t right)
{
    ps2_left = left; ps2_right = right; ps2_update_ms = HAL_GetTick();
    if (manual_mode) force_stop = 0U;
}

void ChassisControl_SetManualMode(uint8_t enabled)
{
    manual_mode = enabled ? 1U : 0U;
    force_stop = 1U;
}

void ChassisControl_EmergencyStop(void)
{
    uint8_t stop[8] = {0x2B,0,0,0,0,0,0,0};
    force_stop = 1U;
    leftSpeedSet = 0;
    rightSpeedSet = 0;
    CAN1_Send(DRIVE_CAN_ID, 8U, stop);
}

void ChassisControl_Tick(void)
{
    int8_t left = 0, right = 0;
    uint32_t now = HAL_GetTick();

#if CHASSIS_FORWARD_TEST_MODE
    uint32_t elapsed = (uint32_t)(now - forward_test_start_ms);

    /* Keep the normal-mode state compiled and type-checked in test builds. */
    (void)ros_left; (void)ros_right; (void)ps2_left; (void)ps2_right;
    (void)ros_update_ms; (void)ps2_update_ms;
    (void)manual_mode; (void)force_stop;

    /*
     * Exact measured forward convention at speed 10:
     * 2B 00 00 00 F6 0A 00 00
     * data[4] = right = -10 (0xF6), data[5] = left = +10 (0x0A).
     */
    if (elapsed >= FORWARD_TEST_DELAY_MS)
    {
        left = FORWARD_TEST_SPEED;
        right = -FORWARD_TEST_SPEED;
    }

    leftSpeedSet = left;
    rightSpeedSet = right;
    control_cmmd[0] = 0x2BU;
    control_cmmd[1] = control_cmmd[2] = control_cmmd[3] = 0U;
    control_cmmd[4] = (uint8_t)right;
    control_cmmd[5] = (uint8_t)left;
    control_cmmd[6] = control_cmmd[7] = 0U;
    CAN1_Send(DRIVE_CAN_ID, 8U, control_cmmd);
#else

    if (!force_stop)
    {
        if (manual_mode && (uint32_t)(now - ps2_update_ms) <= PS2_COMMAND_TIMEOUT)
        {
            left = ps2_left; right = ps2_right;
        }
        else if (!manual_mode && (uint32_t)(now - ros_update_ms) <= ROS_COMMAND_TIMEOUT)
        {
            left = ros_left; right = ros_right;
        }
    }
    else force_stop = 0U;

    leftSpeedSet = left;
    rightSpeedSet = right;
    control_cmmd[0] = 0x2BU;
    control_cmmd[1] = control_cmmd[2] = control_cmmd[3] = 0U;
    control_cmmd[4] = (uint8_t)right;
    control_cmmd[5] = (uint8_t)left;
    control_cmmd[6] = control_cmmd[7] = 0U;
    CAN1_Send(DRIVE_CAN_ID, 8U, control_cmmd);
#endif
}
