"""串口适配器，UI 只接触此类暴露的信号。"""
from __future__ import annotations

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 允许未安装 pyserial 时启动模拟模式
    serial = None
    list_ports = None

try:
    from PyQt5 import QtCore
except ImportError:  # pragma: no cover
    QtCore = None


if QtCore:
    class SerialPort(QtCore.QObject):
        data_received = QtCore.pyqtSignal(bytes)
        connection_changed = QtCore.pyqtSignal(bool, str)

        def __init__(self, parent=None):
            super().__init__(parent)
            self._serial = None
            self._timer = QtCore.QTimer(self)
            self._timer.timeout.connect(self.poll)

        @staticmethod
        def ports() -> list[str]:
            return [item.device for item in list_ports.comports()] if list_ports else []

        @property
        def connected(self) -> bool:
            return bool(self._serial and self._serial.is_open)

        def open(self, port: str, baudrate: int = 921600) -> bool:
            if serial is None:
                self.connection_changed.emit(False, "未安装 pyserial")
                return False
            try:
                self.close(emit=False)
                self._serial = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0,
                    write_timeout=0.2,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                self._timer.start(5)
                self.connection_changed.emit(True, f"已连接 {port}")
                return True
            except Exception as error:
                self.connection_changed.emit(False, f"串口打开失败：{error}")
                return False

        def close(self, emit: bool = True) -> None:
            self._timer.stop()
            if self._serial:
                self._serial.close()
            self._serial = None
            if emit:
                self.connection_changed.emit(False, "串口未连接")

        def write(self, data: bytes) -> bool:
            if not self.connected:
                return False
            try:
                return self._serial.write(data) == len(data)
            except (OSError, serial.SerialException) as error:
                self.close(emit=False)
                self.connection_changed.emit(False, f"串口写入失败：{error}")
                return False

        def poll(self) -> None:
            if not self.connected:
                return
            try:
                if self._serial.in_waiting:
                    self.data_received.emit(self._serial.read(self._serial.in_waiting))
            except (OSError, serial.SerialException) as error:
                self.close(emit=False)
                self.connection_changed.emit(False, f"串口读取失败：{error}")
else:
    class SerialPort:  # pragma: no cover
        def __init__(self, *args, **kwargs): self._serial = None
