# Raw Transport Quality Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add 100Hz Raw packet sequence tracking and PC-side transport quality statistics.

**Architecture:** Extend the Raw packet to 35 bytes with a device-side `uint16` sequence field. Parse sequence on the PC, maintain sequence-gap statistics in a small isolated helper, and expose the metrics in the Raw UI and CSV.

**Tech Stack:** STM32 HAL C firmware, PyQt5 monitor, pytest regression tests, Markdown docs.

---

### Task 1: Protocol Regression Tests

**Files:**
- Create/modify: `tests/test_raw_transport_quality.py`
- Modify: `tools/monitor/protocol.py`
- Modify: `tools/monitor/raw_quality.py`

- [x] Write failing tests for 35-byte Raw packet parsing and sequence-gap accounting.
- [x] Run the tests and verify they fail before implementation.
- [x] Implement parser constants, `RawDataPacket.sequence`, and `RawQualityStats`.
- [x] Run the tests and verify they pass.

### Task 2: Firmware Packet Extension

**Files:**
- Modify: `Core/Inc/main.h`
- Modify: `Core/Src/main.c`
- Test: `tests/test_mimu_config.py`

- [x] Update packet length and XOR length to 35/31.
- [x] Add sequence index definitions.
- [x] Pack sequence at bytes 31-32 and compute XOR at byte 33.
- [x] Increment sequence once per generated Raw sample cycle, independent of UART DMA return status.
- [x] Extend existing source-inspection tests for the packet constants and sequence semantics.

### Task 3: Monitor Integration

**Files:**
- Modify: `tools/monitor/serial_reader.py`
- Modify: `tools/monitor/raw_data_panel.py`
- Modify: `tests/test_monitor_recording.py`

- [x] Update serial chunk size automatically through `RAW_PACKET_LEN`.
- [x] Feed parsed `sequence` into quality statistics.
- [x] Display `RX Hz`, `DEV Hz`, `Loss`, and packet count in the Raw info bar.
- [x] Add `Seq` and `MissingBefore` columns to Raw CSV.
- [x] Reset statistics on clear and recording start.

### Task 4: Documentation and Verification

**Files:**
- Modify: `docs/上位机UI说明文档.md`
- Modify: `docs/在线心率算法实施文档.md`
- Modify: `docs/MAX30101项目配置说明.md`

- [x] Document the 35-byte Raw packet layout.
- [x] Document quality metric definitions.
- [x] Add change-log entries dated 2026-04-24.
- [x] Run pytest, Python compilation, user-run CMake build, and diff checks.
