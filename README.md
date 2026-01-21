# BTHome Door/Window Status & Illumination Sensor
## Project Introduction (项目简介)
The BTHome protocol is a lightweight Bluetooth communication protocol designed specifically for data transmission between smart home sensor devices and receivers (such as gateways or main control devices).

BTHome 协议是一种轻量级的蓝牙通信协议，专为智能家居传感器设备与接收端（如网关或主控设备）之间的数据传输设计。

Based on the BTHome protocol, this project combines TI Hall sensors and photometric sensors to achieve accurate collection and reporting of data such as door/window status, illumination intensity, and battery voltage. The core hardware adopts the CH592F MCU, and the overall circuit design is extremely streamlined, allowing for soldering and assembly without any resistive components.

本项目基于 BTHome 协议，结合 TI 霍尔传感器与光度传感器，实现了门窗状态、光照强度、电池电压等数据的精准采集与上报。核心硬件使用了 CH592F MCU，整体电路设计极为精简，无需使用任何电阻元件即可完成焊接组装。

<img width="591" height="411" alt="image" src="https://github.com/user-attachments/assets/f473346d-b932-494d-9a7d-70b505d8e5fb" />


## Features & Collected Content (功能与采集内容)
<img width="1656" height="936" alt="image" src="https://github.com/user-attachments/assets/ac532dc1-d7cb-4818-8edf-7d671a256950" />


### 1. Illumination Intensity (光照强度)
- Sampling Frequency (采样频率): Collected once every 40 seconds (每 40 秒采集一次)
- Sensor (传感器): TI Photometric Sensor (TI 光度传感器)
- Function Description (功能描述): Real-time monitoring of ambient illumination intensity, suitable for application scenarios such as indoor lighting adjustment (实时监测环境光照强度，适用于室内光照调节等应用场景)

### 2. Door/Window Closing Count (门窗关合次数)
- Trigger Mode (触发方式): MCU interrupt triggered each time the door/window is closed (每次关门触发 MCU 中断)
- Recorded Content (记录内容): Cumulative number of door/window closings (累计关门的次数)
- Function Description (功能描述): Real-time recording of door/window opening and closing actions through Hall sensors (通过霍尔传感器实时记录门窗开关动作次数)

### 3. Current Door/Window Status (门窗当前状态)
Real-time recording of the current open/closed status of doors/windows, facilitating status monitoring and linkage control (实时记录当前门窗开/关状态，方便状态监测与联动控制)

### 4. Battery Voltage Monitoring (电池电压监测)
Monitoring the current voltage of the button battery, making it easy to grasp the power usage and replace the battery in a timely manner (监测纽扣电池当前电压，便于掌握电量使用情况并及时更换电池)

## Project Hardware Design (项目硬件设计)
- Main Control Chip (主控芯片): WCH CH592F
- Hall Sensor (霍尔传感器): TI Series (TI 系列)
- Photometric Sensor (光度传感器): TI Series (TI 系列)

### Circuit Characteristics (电路特点)
- Ultra-minimal layout (极简布局)
- No need to solder resistive components (无需焊接电阻元件)
- Low cost and high integration (低成本、高集成度)

## Known Issues & To-Do Items (已知问题与待完善事项)
1. Unfinished Low-Power Testing (低功耗测试尚未完成)
Complete low-power performance testing has not been conducted yet. Friends with low-power testing equipment are welcome to assist in testing and feedback the results (目前尚未进行完整的低功耗性能测试。欢迎有低功耗测试设备的小伙伴协助测试并反馈结果)

2. Poor Antenna Matching (天线部分匹配不好)

## Usage Instructions (使用说明)
### 1. Firmware Burning (固件烧录)
<img width="1364" height="830" alt="image" src="https://github.com/user-attachments/assets/a9c6b9de-dd5e-4b80-8904-2f32470ee63a" />


- Use the WCH_TOOL for firmware burning (使用 WCH_TOOL 工具进行固件烧录)
- The burning process is simple and supports mass production operations (烧录流程简单，支持量产化操作)

### 2. Usage with Home Assistant
After burning is completed, add the device in Home Assistant (HA). The device usually pops up for identification automatically, displayed with the name "LWSensor". After adding, you can view sensor data in real time and support automated linkage.

烧录完成后，在 Home Assistant (HA) 中添加设备。设备通常会自动弹出识别，显示名称为 LWSensor。完成添加后可实时查看传感器数据，并支持自动化联动。

<img width="1497" height="627" alt="image" src="https://github.com/user-attachments/assets/6e7fdb43-5745-4e14-b379-0940b3c53406" />

### Commercial Restriction (商用限制)
This project is prohibited for commercial use (本项目禁止商用)
