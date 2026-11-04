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
| D36A轴1 DIR / EN | PA17 / PB12 | 已配置，EN高有效 |
| D36A轴2 DIR / EN | PB16 / PB13 | 预留，默认禁用 |
| D36A轴1 STEP | PA22，TIMG6提供20 kHz调度 | 已配置 |
| MaixCAM Pro | PB6 TX, PB7 RX / UART1 | 115200-8-N-1 |
| JY61P | PA0 TX, PA1 RX / UART0 | 9600-8-N-1 |
| 可选HOME_NC | PB4 | 仅保留接口，最终样机未安装 |
| 调试串口 | PA10 TX, PA11 RX | 保留 |

PA21、PA23、PA02、PA18 不作为普通扩展资源使用。当前按键方案不使用这些引脚。
`system.syscfg` 是实际生成配置的唯一来源。

