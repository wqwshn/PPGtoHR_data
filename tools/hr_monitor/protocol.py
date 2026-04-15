"""
PPG Heart Rate Monitor - 协议定义与帧解析

帧类型 A: 31 字节心率结果包 (1Hz, 0xAA 0xCC):
  偏移  字段                类型        说明
  0-1   帧头                uint8 x2    0xAA, 0xCC
  2-3   融合心率 BPM        uint16 BE   x10 精度 (72.5 = 725)
  4     运动标志             uint8       0=静息, 1=运动
  5     窗口填充状态         uint8       0=未满, 1=已满
  6-7   LMS-HF 路径 BPM     uint16 BE   x10 精度
  8-9   LMS-ACC 路径 BPM    uint16 BE   x10 精度
  10-11 FFT 路径 BPM        uint16 BE   x10 精度
  12-13 PPG 信号均值         uint16 BE   信号强度
  14    运动校准状态         uint8       0=未校准, 1=已校准
  15-16 时间戳              uint16 BE   秒计数器
  17    校准窗口进度         uint8       0-8
  18    采样率              uint8       固定 125
  19-20 HF1 AC 幅值          uint16 BE   x100 mV (桥顶1)
  21-22 HF1-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0)
  23-24 ACC-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0, 最优轴)
  25-26 HF2 AC 幅值          uint16 BE   x100 mV (桥顶2)
  27-28 HF2-PPG 相关系数     int16 BE   x10000 (-1.0~+1.0)
  29    XOR 校验             uint8       bytes[2..28] 异或
  30    帧尾                uint8       0xCC

帧类型 B: 33 字节多光谱原始数据包 (100Hz, 0xAA 0xBB):
  偏移  字段              类型        说明
  0-1   帧头              uint8 x2    0xAA, 0xBB
  2-9   ADC 桥路          uint16 x4   桥顶2, 桥顶1, 桥中2, 桥中1
  10-15 ACC X/Y/Z         int16 x3    加速度计 (完整16位)
  16-21 GYRO X/Y/Z        int16 x3    陀螺仪角速度
  22-24 PPG Green         3 bytes     17-bit 原始ADC值
  25-27 PPG Red           3 bytes     17-bit 原始ADC值
  28-30 PPG IR            3 bytes     17-bit 原始ADC值
  31    XOR 校验           uint8       bytes[2..30] 异或
  32    帧尾              uint8       0xCC
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

# 帧常量
HEADER_BYTE_0 = 0xAA
HEADER_BYTE_1 = 0xCC
FOOTER_BYTE = 0xCC

PACKET_LEN = 31
PAYLOAD_START = 2    # 帧头后的 payload 起始偏移
PAYLOAD_END = 29     # XOR 校验前一字节 (含)
XOR_START = 2        # XOR 计算起始偏移
XOR_END = 28         # XOR 计算结束偏移 (含)


@dataclass
class HRPacket:
    """解析后的心率数据包"""
    fused_bpm: float        # 融合心率 (BPM)
    is_motion: bool         # 运动标志
    win_filled: bool        # 窗口是否已填满
    hr_lms_hf: float        # LMS-HF 路径 BPM
    hr_lms_acc: float       # LMS-ACC 路径 BPM
    hr_fft: float           # FFT 路径 BPM
    ppg_mean: int           # PPG 信号均值 (原始均值)
    motion_calibrated: bool # 运动阈值是否已校准
    timestamp: int          # 秒计数器
    calib_progress: int     # 校准进度 0-8
    sampling_rate: int      # 采样率 (Hz)
    hf1_ac_mv: float        # HF1(桥顶1) AC 幅值 (mV)
    hf1_ppg_corr: float     # HF1-PPG 相关系数 (-1~+1)
    acc_ppg_corr: float     # ACC-PPG 相关系数 (-1~+1, 最优轴)
    hf2_ac_mv: float        # HF2(桥顶2) AC 幅值 (mV)
    hf2_ppg_corr: float     # HF2-PPG 相关系数 (-1~+1)


def _decode_int16(raw: int) -> int:
    """大端 uint16 转 int16 有符号"""
    if raw >= 0x8000:
        return raw - 0x10000
    return raw


def parse_hr_packet(data: bytes) -> Optional[HRPacket]:
    """
    解析 31 字节心率结果包.

    Args:
        data: 完整的 31 字节原始帧

    Returns:
        HRPacket 解析成功, None 校验失败
    """
    if len(data) != PACKET_LEN:
        return None

    # 校验帧头
    if data[0] != HEADER_BYTE_0 or data[1] != HEADER_BYTE_1:
        return None

    # 校验帧尾
    if data[30] != FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..28] 异或 == data[29]
    xor_val = 0
    for i in range(XOR_START, XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[29]:
        return None

    # 大端解析各字段
    fused_bpm_raw = (data[2] << 8) | data[3]
    hr_lms_hf_raw = (data[6] << 8) | data[7]
    hr_lms_acc_raw = (data[8] << 8) | data[9]
    hr_fft_raw = (data[10] << 8) | data[11]
    ppg_mean_raw = (data[12] << 8) | data[13]
    timestamp_raw = (data[15] << 8) | data[16]

    # 信号质量字段
    hf1_ac_raw = (data[19] << 8) | data[20]
    hf1_corr_raw = _decode_int16((data[21] << 8) | data[22])
    acc_corr_raw = _decode_int16((data[23] << 8) | data[24])
    hf2_ac_raw = (data[25] << 8) | data[26]
    hf2_corr_raw = _decode_int16((data[27] << 8) | data[28])

    return HRPacket(
        fused_bpm=fused_bpm_raw / 10.0,
        is_motion=bool(data[4]),
        win_filled=bool(data[5]),
        hr_lms_hf=hr_lms_hf_raw / 10.0,
        hr_lms_acc=hr_lms_acc_raw / 10.0,
        hr_fft=hr_fft_raw / 10.0,
        ppg_mean=ppg_mean_raw,
        motion_calibrated=bool(data[14]),
        timestamp=timestamp_raw,
        calib_progress=data[17],
        sampling_rate=data[18],
        hf1_ac_mv=hf1_ac_raw / 100.0,
        hf1_ppg_corr=hf1_corr_raw / 10000.0,
        acc_ppg_corr=acc_corr_raw / 10000.0,
        hf2_ac_mv=hf2_ac_raw / 100.0,
        hf2_ppg_corr=hf2_corr_raw / 10000.0,
    )


# ── 原始数据帧常量 ──────────────────────────────────────
RAW_HEADER_BYTE_1 = 0xBB
RAW_PACKET_LEN = 33
RAW_XOR_START = 2
RAW_XOR_END = 30   # 含


@dataclass
class RawDataPacket:
    """解析后的多光谱原始传感器数据包 (33 字节)"""
    # ADC 桥路
    adc_hf2: int          # ADS124S06 桥顶2
    adc_hf1: int          # ADS124S06 桥顶1
    adc_mid2: int         # ADS124S06 桥中2
    adc_mid1: int         # ADS124S06 桥中1
    # 加速度计 (完整 16-bit)
    acc_x: int
    acc_y: int
    acc_z: int
    # 陀螺仪 (角速度)
    gyro_x: int
    gyro_y: int
    gyro_z: int
    # PPG 多光谱
    ppg_green: int        # 绿光 17-bit 原始值
    ppg_red: int          # 红光 17-bit 原始值
    ppg_ir: int           # 红外光 17-bit 原始值


def parse_raw_packet(data: bytes) -> Optional[RawDataPacket]:
    """
    解析 33 字节多光谱原始数据包.

    Args:
        data: 完整的 33 字节原始帧

    Returns:
        RawDataPacket 解析成功, None 校验失败
    """
    if len(data) != RAW_PACKET_LEN:
        return None

    if data[0] != HEADER_BYTE_0 or data[1] != RAW_HEADER_BYTE_1:
        return None

    if data[32] != FOOTER_BYTE:
        return None

    # XOR 校验: bytes[2..30]
    xor_val = 0
    for i in range(RAW_XOR_START, RAW_XOR_END + 1):
        xor_val ^= data[i]
    if xor_val != data[31]:
        return None

    # ADC (uint16 BE, 无符号)
    adc_hf2  = (data[2] << 8)  | data[3]
    adc_hf1  = (data[4] << 8)  | data[5]
    adc_mid2 = (data[6] << 8)  | data[7]
    adc_mid1 = (data[8] << 8)  | data[9]

    # ACC (int16 BE, 有符号)
    acc_x = _decode_int16((data[10] << 8) | data[11])
    acc_y = _decode_int16((data[12] << 8) | data[13])
    acc_z = _decode_int16((data[14] << 8) | data[15])

    # GYRO (int16 BE, 有符号)
    gyro_x = _decode_int16((data[16] << 8) | data[17])
    gyro_y = _decode_int16((data[18] << 8) | data[19])
    gyro_z = _decode_int16((data[20] << 8) | data[21])

    # PPG 三通道 (3 bytes each, 17-bit 对齐后)
    ppg_green = ((data[22] << 16) | (data[23] << 8) | data[24]) & 0x01FFFF
    ppg_red   = ((data[25] << 16) | (data[26] << 8) | data[27]) & 0x01FFFF
    ppg_ir    = ((data[28] << 16) | (data[29] << 8) | data[30]) & 0x01FFFF

    return RawDataPacket(
        adc_hf2=adc_hf2, adc_hf1=adc_hf1, adc_mid2=adc_mid2, adc_mid1=adc_mid1,
        acc_x=acc_x, acc_y=acc_y, acc_z=acc_z,
        gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
        ppg_green=ppg_green, ppg_red=ppg_red, ppg_ir=ppg_ir,
    )
