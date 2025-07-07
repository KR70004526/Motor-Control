from PyQt5 import QtCore

class SerialReaderThread(QtCore.QThread):
    status_received = QtCore.pyqtSignal(dict)

    def __init__(self, serial_obj, parser):
        super().__init__()
        self.serial = serial_obj
        self.parser = parser
        self.running = True

    def run(self):
        while self.running:
            if self.serial and self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("Position:"):
                        parsed = self.parser.parse_line(line)
                        if parsed:
                            self.status_received.emit(parsed)
                except Exception as e:
                    print(f"[Serial Thread Error] {e}")
            self.msleep(50) 

    def stop(self):
        self.running = False