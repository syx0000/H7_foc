"""CAN Traffic Panel - 独立显示 RX/TX 报文 + 保存功能.

双栏布局: 左侧 RX, 右侧 TX, 各自独立的 Pause/Clear/Save CSV 控制.
顶部 Promiscuous 复选框控制 CAN worker 过滤模式.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLabel, QCheckBox, QFileDialog, QHeaderView, QSplitter
)
from PySide6.QtCore import Slot, Qt
from PySide6.QtGui import QFont
from datetime import datetime


class _TrafficTable(QWidget):
    """单个流量表格 (RX 或 TX) + Pause/Clear/Save 控制."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._paused = False
        self._pending_rows = []  # (dt_str, can_id, dlc, data_hex)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 标题栏
        header = QHBoxLayout()
        header.addWidget(QLabel(f"<b>{title}</b>"))
        header.addStretch()

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.setMaximumWidth(80)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        header.addWidget(self._pause_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setMaximumWidth(80)
        self._clear_btn.clicked.connect(self._clear)
        header.addWidget(self._clear_btn)

        self._save_btn = QPushButton("Save CSV...")
        self._save_btn.setMaximumWidth(100)
        self._save_btn.clicked.connect(self._on_save_clicked)
        header.addWidget(self._save_btn)

        layout.addLayout(header)

        # 表格: Time | CAN ID | DLC | Data
        self._table = QTableWidget()
        self._table.setColumnCount(4)
        self._table.setHorizontalHeaderLabels(["Time", "CAN ID", "DLC", "Data (hex)"])
        self._table.setFont(QFont("Consolas", 9))
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)

        # 列宽: Time 120px, ID 80px, DLC 50px, Data 自动
        self._table.setColumnWidth(0, 120)
        self._table.setColumnWidth(1, 80)
        self._table.setColumnWidth(2, 50)
        self._table.horizontalHeader().setStretchLastSection(True)

        layout.addWidget(self._table)

    def append_frame(self, can_id: int, data: bytes, ts_us: int):
        """追加一条 CAN 帧记录.

        Args:
            can_id: CAN ID (int)
            data: 数据字节
            ts_us: 硬件时间戳 (µs), TX 时为 0 则用主机时间
        """
        if ts_us > 0:
            # RX: 用硬件时间戳 (相对值, 仅展示)
            dt_str = f"{ts_us / 1e6:.6f}"
        else:
            # TX: 用主机时间
            dt_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        dlc = len(data)
        data_hex = data.hex(' ').upper()

        if self._paused:
            # 暂停时缓存
            self._pending_rows.append((dt_str, can_id, dlc, data_hex))
            return

        self._add_row(dt_str, can_id, dlc, data_hex)

    def _add_row(self, dt_str: str, can_id: int, dlc: int, data_hex: str):
        """插入一行到表格顶部 (最新在上)."""
        row = 0
        self._table.insertRow(row)
        self._table.setItem(row, 0, QTableWidgetItem(dt_str))
        self._table.setItem(row, 1, QTableWidgetItem(f"0x{can_id:03X}"))
        self._table.setItem(row, 2, QTableWidgetItem(str(dlc)))
        self._table.setItem(row, 3, QTableWidgetItem(data_hex))

        # 限制最大行数 (避免内存溢出)
        if self._table.rowCount() > 5000:
            self._table.removeRow(5000)

    @Slot(bool)
    def _on_pause_toggled(self, checked: bool):
        """Pause/Resume."""
        self._paused = checked
        if checked:
            self._pause_btn.setText("Resume")
        else:
            self._pause_btn.setText("Pause")
            # 恢复时，把缓存的行一次性插入
            for row_data in self._pending_rows:
                self._add_row(*row_data)
            self._pending_rows.clear()

    def _clear(self):
        """清空表格."""
        self._table.setRowCount(0)
        self._pending_rows.clear()

    def _on_save_clicked(self):
        """导出 CSV."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"can_traffic_{timestamp}.csv"

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save CAN Traffic", default_name, "CSV Files (*.csv);;All Files (*.*)"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                # 标题行
                f.write("Time,CAN ID,DLC,Data\n")
                # 从底部往上读 (最老的在前)
                for row in reversed(range(self._table.rowCount())):
                    time_str = self._table.item(row, 0).text()
                    id_str = self._table.item(row, 1).text()
                    dlc_str = self._table.item(row, 2).text()
                    data_str = self._table.item(row, 3).text().replace(' ', '')
                    f.write(f"{time_str},{id_str},{dlc_str},{data_str}\n")
            # 可选: 在某处显示成功提示 (这里不 emit 信号, 避免耦合)
        except Exception as e:
            pass  # 静默失败, 或改成 emit error signal


class CanTrafficPanel(QWidget):
    """CAN 流量监控面板: 双栏 RX/TX + Promiscuous 开关."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)

        # 顶栏: Promiscuous 开关
        top_bar = QHBoxLayout()
        self._promisc_cb = QCheckBox("Promiscuous Mode (receive all CAN IDs)")
        self._promisc_cb.setToolTip(
            "Unchecked: only 0x7E0~0x7EF (debug protocol)\n"
            "Checked: receive all 0x000~0x7FF (includes WLY frames)"
        )
        self._promisc_cb.toggled.connect(self._on_promisc_toggled)
        top_bar.addWidget(self._promisc_cb)
        top_bar.addStretch()
        layout.addLayout(top_bar)

        # 双栏 splitter: RX | TX
        splitter = QSplitter(Qt.Horizontal)
        self._rx_table = _TrafficTable("RX")
        self._tx_table = _TrafficTable("TX")
        splitter.addWidget(self._rx_table)
        splitter.addWidget(self._tx_table)
        splitter.setSizes([500, 500])
        layout.addWidget(splitter)

        self._can_worker = None  # 由 main_window 通过 set_can_worker 注入

    def set_can_worker(self, worker):
        """注入 CanWorker 实例, 对接信号 + 混杂模式控制."""
        self._can_worker = worker
        if worker:
            worker.sig_can_rx_raw.connect(self._on_rx_frame)
            worker.sig_can_tx_raw.connect(self._on_tx_frame)

    @Slot(bool)
    def _on_promisc_toggled(self, checked: bool):
        """用户切换混杂模式."""
        if self._can_worker and hasattr(self._can_worker, 'set_promiscuous'):
            self._can_worker.set_promiscuous(checked)

    @Slot(int, bytes, int)
    def _on_rx_frame(self, can_id: int, data: bytes, ts_us: int):
        """接收 RX 帧."""
        self._rx_table.append_frame(can_id, data, ts_us)

    @Slot(int, bytes, int)
    def _on_tx_frame(self, can_id: int, data: bytes, ts_us: int):
        """接收 TX 帧."""
        self._tx_table.append_frame(can_id, data, ts_us)
