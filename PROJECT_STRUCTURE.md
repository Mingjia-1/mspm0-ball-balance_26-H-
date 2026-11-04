# 工程结构与职责

```text
ball_balance_car_mspm0g3507/
├─ .project/.cproject/.ccsproject  CCS工程元数据
├─ system.syscfg                   时钟、引脚、外设和中断的唯一配置源
├─ targetConfigs/                  MSPM0G3507调试目标配置
├─ src/                            固件入口
├─ app/                            比赛模式、系统状态与安全策略
├─ bsp/                            只负责硬件驱动和物理接口
├─ control/                        与具体引脚无关的控制算法
├─ protocol/                       串口数据帧定义和编解码
├─ config/                         可调参数和硬件常量
├─ docs/                           架构、接线和调试文档
└─ tests/                          后续台架与主机侧测试
```

每个源文件顶部均说明了自身职责。生成的`Debug/ti_msp_dl_config.*`、链接脚本、对象文件和输出文件不属于可编辑源文件。
