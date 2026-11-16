# USB PS2 手柄与 ROS 双控制说明

## 当前运行模式

`HARDWARE/PS2/chassis_control.h` 中的 `CHASSIS_FORWARD_TEST_MODE` 当前为
`0`，已关闭 CANgaroo 持续发送测试，恢复 ROS/USB 手柄双控制。

如需再次进行 CANgaroo 测试，可临时改为 `1U`：上电延时 3 秒后，以
20 Hz 持续发送 `0x603: 2B 00 00 00 F6 0A 00 00`。此时若连接电机，底盘会持续前进。

## 接线

- USB D- -> PA11 (USB_OTG_FS_DM, AF10)
- USB D+ -> PA12 (USB_OTG_FS_DP, AF10)
- USB 5V 和 GND 必须正常供电并与主控共地
- PA12 原来的 LED 功能已停用，不能再把 PA12 当普通 LED GPIO 使用

当前解码器直接采用 WHEELTEC 示例，支持有线 V1 `0810:0001`、有线 V2
`0079:0006`、无线 PC 模式 `0079:0126` 和无线 Android 模式
`045E:028E`。

## 控制权

- 上电默认由 ROS 控制。
- 按手柄 `START`，切换到手柄控制；此时 ROS 指令仍会接收，但不会下发到底盘。
- 按手柄 `SELECT`，先停车一帧，再返回 ROS 控制。
- 手柄拔出、USB HID 数据中断超过 150 ms、ROS 指令中断超过 1000 ms，均输出停车速度。

## 手柄映射

- 左摇杆前后：底盘前进/后退。
- 左摇杆左右：转向；同时前后拨动可边走边转。
- 右摇杆左右：也可转向；左摇杆横向超出死区时优先使用左摇杆，不叠加两路转向。
- 原地转向时，右拨输出左右轮同为正值，左拨同为负值。实车若反向，将 `ps2.c` 中 `PS2_TURN_DIRECTION` 改为 `-1`。
- 摇杆死区为 20，最大 CAN 速度值为 50，可在 `HARDWARE/PS2/ps2.c` 修改。

底盘的实测正方向已经固化为“左轮正、右轮负”。例如左右速度幅值均为 10 时，发送数据为：

`2B 00 00 00 F6 0A 00 00`

CAN 标准帧 ID 使用 `0x603`，与原 ROS 接收代码保持一致。

## 重新生成 CubeMX 代码时

工程的 `.ioc` 已改成 USB Host 模式并把 PLLQ 设置为 7（USB 时钟 48 MHz）。重新生成后仍需确认：

- `PA11/PA12` 保持 USB OTG FS，而不是 GPIO。
- `PLLQ=7`。
- `HAL_HCD_MODULE_ENABLED` 已启用。
- USB Host、PS2 和 `chassis_control` 文件仍在 Keil 工程中。
