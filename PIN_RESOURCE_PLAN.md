# H 题引脚资源表

| 子系统 | 引脚/外设 | 当前状态 |
|---|---|---|
| 右轮 PWM | PA12 / TIMG0_C0 | 已配置 |
| 左轮 PWM | PA13 / TIMG0_C1 | 已配置 |
| 右轮方向 | PB17, PB19 | 已配置 |
| 左轮方向 | PA16, PB24 | 已配置 |
| TB6612 STBY | PB5 | 已配置，复位默认关闭 |
| 左编码器 | PA25, PA14 | 双边沿中断 |
| 右编码器 | PA26, PA27 | 双边沿中断 |
| 八路红外 D1..D8 | PB25, PB18, PB21, PB22, PA30, PB0, PB1, PB10 | 已配置 |
| OLED | PA28 SDA, PA31 SCL / I2C0 | 400 kHz |
| KEY1 | PB2 | 启动/台架暂停，低有效上拉 |
| KEY2 | PB3 | 任务模式切换，低有效上拉 |
| KEY3 | PB23 | 确认/执行，低有效上拉 |
| KEY4 | PB15 | 急停/故障复位，低有效上拉 |
| D36A DIR1 / EN1_N | PA17 / PB12 | 已配置，默认禁用 |
| D36A DIR2 / EN2_N | PB16 / PB13 | 已配置，默认禁用 |
| D36A STEP1 / STEP2 | PA22 / TIMA0_C1；PA15 / TIMA0_C2 | 后续阶段配置 |
| MaixCAM | PB6 TX, PB7 RX / UART1 | 后续阶段配置 |
| MPU6050 | PA0, PA1 软件 I2C；PB4 可选 INT | 后续阶段配置 |
| 调试串口 | PA10 TX, PA11 RX | 保留 |

PA21、PA23、PA02、PA18 不作为普通扩展资源使用。当前按键方案不使用这些引脚。
`system.syscfg` 是实际生成配置的唯一来源。
