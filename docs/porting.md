# 移植

主机仿真和板上固件调用的是同一个 `cell_tick(cell, dt_ms)`。板上要补的是节拍、引脚和两个通信口。

建议拆成两个任务：

- 控制任务，1 kHz：`cell_tick`。里面有运动规划、夹爪计时、安全和状态机。
- 通信任务：Modbus RTU 收满一帧就调用 `cell_modbus`，MQTT 收到 `sss/cell/cmd` 就调用 `cell_cmd_json`。每收到一帧合法通信调用一次 `cell_kick`，给 500 ms 看门狗喂狗。

不要在 `cell_tick` 里做阻塞的网络收发。

## ESP32-S3-DevKitC-1

`ports/esp32/port_main.c` 在定义了 `ESP_PLATFORM` 时用 `esp_timer` 计算 `dt`，并用 FreeRTOS 起 1 kHz 任务。MQTT 建议用 ESP-IDF 自带的 `esp-mqtt`，收到的 payload 直接交给 `cell_cmd_json`。发布时把 `cell_set_publisher` 接到 `esp_mqtt_client_publish`，主题用 `sss/task` 和 `sss/cell/status`。

参考引脚：

| 信号 | GPIO |
| --- | --- |
| 急停，低有效，内部上拉 | 1 |
| 门关闭，高有效 | 2 |
| 夹爪阀 | 4 |
| 夹爪有料 | 5 |
| X / Y / Z 原点 | 6 / 7 / 9 |
| X 步进、方向 | 10 / 11 |
| Y 步进、方向 | 12 / 13 |
| Z 步进、方向 | 14 / 15 |
| RS485 TX / RX / DE | 17 / 18 / 8 |

步进脉冲不必在 `cell_tick` 里逐个微米去翻引脚。每个轴在这一毫秒里的位移是 `motion_pos` 的差，换成脉冲数后交给 RMT 或 MCPWM。原点开关在回零时由 `motion_home` 驱动轴朝 0 走，开关触发后把该轴位置清零。仿真里这个开关就是「位置小于等于 0」。

组件的 `CMakeLists.txt` 在 `ports/esp32/`。把工程的 `EXTRA_COMPONENT_DIRS` 指到那里，或把这些源文件收进自己的 `main`。

## STM32F407

用 TIM6 更新中断做 1 kHz，中断里只调用 `cell_tick(cell, 1)`，或者给任务发一个信号量再在任务里调用。Modbus 放在 USART2 的空闲中断加 DMA 上，一帧结束再解析，不要在中断里跑状态机。

步进可以用 TIM1/TIM3/TIM4 的比较输出。方向脚在速度符号变化时改。急停除了进 `cell_set_estop`，硬件上还应该直接切断驱动器使能，不把这一路只交给软件。

FreeRTOS 的对应关系：

| 这里 | FreeRTOS |
| --- | --- |
| `cell_tick` 每 1 ms | 控制任务里 `vTaskDelayUntil` |
| `cell_kick` | 通信任务在成功收发之后 |
| `cell_modbus` | 串口任务 |
| `cell_cmd_json` | MQTT 或调试串口任务 |

## 仿真和真机的边界

库存、有料开关和轴的位置在 `cell_tick` 里由一个料箱模型更新。真机上删掉这个模型，改成读编码器（或发步进脉冲）和有料接近开关。任务顺序、Modbus 寄存器和 `sss/task` 的 JSON 不用改，前端和 PLC 看到的还是同一份协议。
