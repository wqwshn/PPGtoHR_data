# 协议与链路诊断

- [Raw 数据质量检测](raw_data_quality_detection.md)
- [当前录制文件格式](../acquisition/原始数据录制文件结构说明.md)

当前 Raw 数据包为 35 字节，STATUS 包为 69 字节。上位机保留 HJ-380 握手与 From 前缀剥离，并可解析远程新增的 RF/功率事件。协议常量以 `tools/monitor/protocol.py` 和 `rf_events.py` 为准。
