import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_DIR = ROOT / "tools" / "monitor"
sys.path.insert(0, str(MONITOR_DIR))

import protocol
from raw_quality import RawQualityStats


MAIN_H = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
MAIN_C = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")


def make_raw_packet(sequence: int) -> bytes:
    data = bytearray(protocol.RAW_PACKET_LEN)
    data[0] = protocol.RAW_HEADER_BYTE_0
    data[1] = protocol.RAW_HEADER_BYTE_1
    data[22:25] = bytes([0x00, 0x12, 0x34])
    data[25:28] = bytes([0x00, 0x56, 0x78])
    data[28:31] = bytes([0x00, 0x7A, 0xBC])
    data[31] = (sequence >> 8) & 0xFF
    data[32] = sequence & 0xFF
    xor_val = 0
    for idx in range(protocol.RAW_XOR_START, protocol.RAW_XOR_END + 1):
        xor_val ^= data[idx]
    data[protocol.RAW_XOR_POS] = xor_val
    data[34] = protocol.RAW_FOOTER_BYTE
    return bytes(data)


def test_raw_packet_parses_35_byte_sequence_field():
    assert protocol.RAW_PACKET_LEN == 35
    assert protocol.RAW_XOR_END == 32
    assert protocol.RAW_XOR_POS == 33

    pkt = protocol.parse_raw_packet(make_raw_packet(0x1234))

    assert pkt is not None
    assert pkt.sequence == 0x1234
    assert pkt.ppg_green == 0x1234
    assert pkt.ppg_red == 0x5678
    assert pkt.ppg_ir == 0x7ABC


def test_raw_quality_stats_count_sequence_gaps_and_loss_rate():
    stats = RawQualityStats()

    assert stats.observe(10) == 0
    assert stats.observe(11) == 0
    assert stats.observe(15) == 3

    assert stats.received_count == 3
    assert stats.expected_count == 6
    assert stats.missing_count == 3
    assert math.isclose(stats.loss_rate, 0.5)


def test_raw_quality_stats_handle_uint16_wraparound():
    stats = RawQualityStats()

    assert stats.observe(0xFFFE) == 0
    assert stats.observe(0x0000) == 1

    assert stats.received_count == 2
    assert stats.expected_count == 3
    assert stats.missing_count == 1


def test_firmware_raw_packet_declares_sequence_extended_layout():
    assert "#define PACKET_LEN 35" in MAIN_H
    assert "#define XOR_CHECK_LEN 31" in MAIN_H
    assert "#define RAW_SEQUENCE_START_INDEX  31" in MAIN_H
    assert "allData[RAW_SEQUENCE_START_INDEX]" in MAIN_C
    assert "allData[RAW_SEQUENCE_START_INDEX + 1]" in MAIN_C
    assert "raw_packet_seq++" in MAIN_C
