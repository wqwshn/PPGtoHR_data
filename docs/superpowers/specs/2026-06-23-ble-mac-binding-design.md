# BLE MAC 地址绑定方案设计

日期: 2026-06-23

## 1. 目标

通过 MCU 配置 HJ-131 自定义 MAC 地址，上位机连接时向 HJ-380 发送指定连接指令，
使 HJ-380 只连接并接收来自该 MAC 地址的 HJ-131 蓝牙芯片的数据，实现设备绑定隔离。

## 2. 架构

```
STM32 (MCU)                      PC (Python/PyQt5)
+------------------------+    +------------------------------------+
| main.c                  |    | SerialReader (QThread)              |
|  BLEC_Init()           |    |  +-----------------------------+    |
|   + ST_OWN_MAC         |    |  | PrefixStripper              |    |
|                        |    |  |  识别 From mac: 前缀/AT响应 |    |
|  HR/RAW packet         |    |  |  剥离 -> 纯二进制帧        |    |
|  (binary 0xAA..)       |    |  +-------------+---------------+    |
+------------------------+    |                |                    |
         | UART               |  +-------------v---------------+    |
         v                    |  | FrameParser (existing)      |    |
    +-----------+             |  |  0xAA BB->raw, CC->hr,      |    |
    |  HJ-131   |             |  |  DD->status                 |    |
    |  (SLAVE)  |             |  +-------------+---------------+    |
    +-----------+             |                |                    |
         | BLE                |  +-------------v---------------+    |
         v                    |  | AT 响应 -> HJ380状态信号    |    |
    +-----------+  USB(CH340) |  +-----------------------------+    |
    |  HJ-380   |-------------|                                    |
    |  (MASTER) |             | dashboard.py                       |
    +-----------+             |  显示 MAC 配置/连接状态           |
                              +------------------------------------+
```

数据流: STM32 USART2 -> HJ-131 --BLE--> HJ-380 --USB/CH340--> PC SerialReader -> PrefixStripper -> FrameParser -> UI

## 3. MCU 端改动

### 3.1 main.h 新增宏

```c
#define BLE_CUSTOM_MAC  "784128c58150"  // 12字节自定义MAC地址
```

### 3.2 main.c BLEC_Init() 增加 MAC 设置命令

在 BLE 配置序列中增加 `<ST_OWN_MAC=...>` 命令。设置后 HJ-131 自动重启并使用新 MAC 广播。

### 3.3 启用 BLE 配置

`ENABLE_BLE_CONFIG` 从 0 改为 1。

MCU 端不需要启用 UART RX，也不需要读取 HJ-131 的响应（HJ-131 设置 MAC 后自动重启）。

## 4. PC 端改动

### 4.1 前缀剥离器 (PrefixStripper)

在 `serial_reader.py` 的数据接收循环中增加状态机：

状态: IDLE -> MAYBE_PREFIX (收到 'F'/'<') -> DATA/AT_RESPONSE -> IDLE

- **IDLE**: 收到 `0xAA` -> 进入现有帧解析；收到 `'F'` -> 进入前缀匹配；收到 `'<'` -> 进入 AT 响应收集
- **前缀匹配**: 匹配 `From    xxxxxxxxxxxx:  ` 模式（"From" + 空格 + 12 hex MAC + ":  "），提取 MAC 后将后续字节送入帧解析器
- **AT 响应**: 收集到 `>` 后解析 AT 响应（如 `<st_con_mac=xxxx>`），发射配置状态信号
- **帧尾 0xCC 后**: 回到 IDLE 状态

性能分析: 125Hz * 35 bytes = 4375 bytes/s (原始), 加上 ~29 字节前缀 = 8000 bytes/s,
远低于 115200 波特率 (11520 bytes/s 可用)。前缀剥离每字节 O(1) 判断，Python 微秒级，不会引入丢包。

### 4.2 HJ-380 连接握手

在 `SerialReader` 连接成功后:

1. 发送 `<ST_CON_MAC=` + BLE_CUSTOM_MAC + `>`
2. 启动 3 秒超时定时器
3. 等待 `<st_con_mac=xxxxxxxxxxxx>` 响应
4. 发射 `mac_configured(mac, success)` 信号

### 4.3 协议常量

`protocol.py` 新增:
```python
BLE_CUSTOM_MAC = "784128c58150"
HJ380_CMD_CON_MAC = "<ST_CON_MAC={}>"
HJ380_RSP_CON_MAC = "<st_con_mac={}>"
```

### 4.4 dashboard.py UI

在连接状态栏增加 MAC 配置状态显示:
- 配置成功: "已绑定设备 MAC: xxxxxxxxxxxx" (绿色)
- 配置失败: "MAC 配置失败，请检查设备" (红色)
- 配置中: "正在配置设备绑定..." (黄色)

## 5. 涉及文件

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| Core/Inc/main.h | 新增宏 | BLE_CUSTOM_MAC 定义, ENABLE_BLE_CONFIG=1 |
| Core/Src/main.c | 新增 1 条命令 | BLEC_Init() 中加 ST_OWN_MAC |
| tools/monitor/protocol.py | 新增常量 | BLE_CUSTOM_MAC 常量 |
| tools/monitor/serial_reader.py | 主要改动 | 前缀剥离、AT 响应解析、握手流程 |
| tools/monitor/dashboard.py | 小幅改动 | MAC 配置状态 UI |

## 6. 风险与验证

- **数据格式风险**: HJ-380 `<ST_CON_MAC>` 连接后数据是否确实带 `From` 前缀，需要实测验证
- **前缀剥离验证**: 检查帧解析器收到的数据是否为纯二进制帧（0xAA 开头）
- **丢包验证**: 连接后运行 10 分钟以上，通过序列号和 CSV 录制确认无丢包
- **MAC 持久化**: HJ-131 `<ST_OWN_MAC>` 设置后掉电是否保存，数据手册未明确说明；
  若不保存，每次上电 MCU 都会重新设置
