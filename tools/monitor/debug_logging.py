"""Persistent local diagnostic sessions and non-disruptive log viewing."""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtGui import QDesktopServices, QTextCursor
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QApplication

from firmware_build import ROOT, BUILD

LOG_DIR = ROOT / ".local" / "logs"
LOCAL_TIME = timezone(timedelta(hours=8))


def append_log_text(widget, text: str):
    """Append through a separate cursor without destroying a user's selection."""
    scroll = widget.verticalScrollBar()
    position = scroll.value()
    follow = not widget.textCursor().hasSelection() and scroll.value() >= scroll.maximum() - 2
    cursor = QTextCursor(widget.document())
    cursor.movePosition(QTextCursor.End)
    cursor.insertText(text)
    if follow:
        scroll.setValue(scroll.maximum())
    else:
        scroll.setValue(position)


class DebugSessionLog:
    def __init__(self, port: str, metadata: dict, *, directory: Path | None = None,
                 kind: str = "board-check", raw_text: bool = True):
        if kind not in ("board-check", "capture"):
            raise ValueError(f"Unknown diagnostic session kind: {kind}")
        directory = (directory if directory is not None else LOG_DIR).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(LOCAL_TIME).strftime("%Y%m%d-%H%M%S-%f")
        self.path = directory / f"{kind}-{stamp}.log"
        self._raw_text = raw_text
        self.raw_path = self.path.with_suffix(".bin")
        self.rx_bytes = 0
        self.last_rx = None
        self._text = self.path.open("x", encoding="utf-8", newline="")
        self._raw = None
        try:
            self._raw = self.raw_path.open("xb")
            self.event("SESSION", f"port={port} baud=115200 8N1; raw={self.raw_path.name}")
            self.event("METADATA", json.dumps(metadata, ensure_ascii=False))
            manifest = BUILD / "firmware-manifest.json"
            if manifest.exists():
                try:
                    # This describes a local build, not proof of the installed firmware.
                    self.event("LOCAL_BUILD_NOT_DEVICE_VERIFIED", manifest.read_text(encoding="utf-8"))
                except (OSError, UnicodeError) as exc:
                    self.event("MANIFEST_UNAVAILABLE", str(exc))
            latest = directory / f"latest-{kind}.json"
            temporary = directory / f"latest-{kind}-{stamp}.tmp"
            temporary.write_text(json.dumps({"log_path": str(self.path), "raw_path": str(self.raw_path),
                                             "started_at": datetime.now(LOCAL_TIME).isoformat(), "port": port},
                                            ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(latest)
        except Exception:
            self._text.close()
            if self._raw is not None:
                self._raw.close()
            raise

    def event(self, name: str, detail: str):
        now = datetime.now(LOCAL_TIME).isoformat(timespec="milliseconds")
        line = f"\n[{now}] [{name}] {detail}\n"
        self._text.write(line)
        self._text.flush()
        return line

    def receive(self, data: bytes):
        self._raw.write(data)
        self._raw.flush()
        self.rx_bytes += len(data)
        self.last_rx = datetime.now(LOCAL_TIME).isoformat(timespec="milliseconds")
        # Keep ordinary firmware lines readable, including a final partial line.
        # Escape binary/control bytes losslessly; exact bytes remain in .bin.
        if self._raw_text:
            self._text.write("".join(chr(byte) if byte in (9, 10, 13) or 32 <= byte < 127
                                     else f"\\x{byte:02x}" for byte in data))
            self._text.flush()

    def close(self):
        if self._text.closed:
            return
        try:
            self.event("CLOSE", f"rx_bytes={self.rx_bytes} last_rx={self.last_rx}")
        finally:
            try:
                self._text.close()
            finally:
                self._raw.close()


class LogTools(QWidget):
    """Copy the full saved log, rather than just a bounded text display."""

    def __init__(self, display, parent=None):
        super().__init__(parent)
        self.display = display
        self.path = None
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        self.copy_button = QPushButton("复制完整日志")
        self.open_button = QPushButton("打开日志")
        self.folder_button = QPushButton("打开日志目录")
        for button in (self.copy_button, self.open_button, self.folder_button):
            row.addWidget(button)
        row.addStretch()
        root.addLayout(row)
        self.path_label = QLabel("尚未生成日志")
        self.path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.path_label.setWordWrap(True)
        root.addWidget(self.path_label)
        self.copy_button.clicked.connect(self.copy_all)
        self.open_button.clicked.connect(self.open_log)
        self.folder_button.clicked.connect(self.open_folder)
        self.open_button.setEnabled(False)

    def set_path(self, path: Path):
        self.path = Path(path)
        self.path_label.setText(f"自动保存：{self.path.name}")
        self.path_label.setToolTip(str(self.path))
        self.open_button.setEnabled(True)

    def copy_all(self):
        try:
            text = self.path.read_text(encoding="utf-8") if self.path else self.display.toPlainText()
            QApplication.clipboard().setText(text)
            self.path_label.setText(f"已复制完整日志（{len(text)} 字符）")
        except (OSError, UnicodeError) as exc:
            self.path_label.setText(f"读取日志失败：{exc}")

    def open_log(self):
        if self.path and not QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.path))):
            self.path_label.setText(f"无法打开日志：{self.path}")

    def open_folder(self):
        folder = self.path.parent if self.path else LOG_DIR
        try:
            folder.mkdir(parents=True, exist_ok=True)
            if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder))):
                self.path_label.setText(f"无法打开目录：{folder}")
        except OSError as exc:
            self.path_label.setText(f"无法打开日志目录：{exc}")
