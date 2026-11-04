# 最终样机接线摘要

- TB6612：PWMA=PA12，PWMB=PA13，AIN1=PB17，AIN2=PB19，
  BIN1=PA16，BIN2=PB24，STBY=PB5。
- 左编码器：PA25、PA14；右编码器：PA26、PA27。
- 八路巡线 D1..D8：PB25、PB18、PB21、PB22、PA30、PB0、PB1、PB10。
  D1 必须安装在车辆左侧最外端。
- OLED：PA28=SDA，PA31=SCL，I2C 上拉到 3.3 V。
- 四键模块：KEY1=PB2、KEY2=PB3、KEY3=PB23、KEY4=PB15；按键另一端共地，
  不接 VCC，全部低电平有效。
- MaixCAM Pro：PB6=UART1_TX、PB7=UART1_RX，115200-8-N-1，3.3 V TTL。
- JY61P：PA0=UART0_TX、PA1=UART0_RX，默认9600-8-N-1，3.3 V TTL。
- D36A轴1：STEP=PA22、DIR=PA17、EN=PB12；DIR=0抬高摆杆右端，DIR=1降低右端。
- D36A轴2预留：DIR=PB16、EN=PB13，当前控制只使用轴1。
- PB4保留为HOME_NC输入，但最终样机未安装机械限位开关；上电后采用人工调平零位与软件行程限制。

上电前确认所有外设共地，巡线数字输出和编码器输出不超过 3.3 V。
