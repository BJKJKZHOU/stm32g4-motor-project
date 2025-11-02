# stm32g4-motor-project
这是一个普通的工程


## 修Bug
1. 串口通信时不时出点小毛病：原因是使用的软件双缓冲和 DMA 配置为 Circular 循环模式会有冲突。
使用硬件双缓冲可配置循环模式，软件双缓冲需要配置为 NORMAL。
之前配置发送时的bug也是因为DMA Circular 循环模式导致定时器中断无效，不能实现定时发送，配置为 NORMAL解决。