import sys
import math
from PyQt5.QtWidgets import QMessageBox
from PyQt5 import QtCore, QtWidgets
from Manual_Dialog import Ui_Dialog
from MotorStatusParser import MotorStatusParser
from ParameterManager import ParameterManager
from SerialPortManager import SerialPortManager
from SerialReaderThread import SerialReaderThread
from motorGUI2 import Ui_MotorGUI2

class Main:
    def __init__(self):
        self.app = QtWidgets.QApplication([])
        self.MainWindow = QtWidgets.QMainWindow()
        self.current_motor_id = None

        # UI Setup
        self.ui = Ui_MotorGUI2()
        self.ui.setupUi(self.MainWindow)

        # Serial / Parameter / Parser Manager
        self.serial_manager = SerialPortManager()
        self.parameter_manager = ParameterManager(self.serial_manager)
        self.status_parser = MotorStatusParser()

        # Load Port List on Start
        self.refresh_serial_ports()

        # Signal Connection
        self.ui.serialButton.clicked.connect(self.connect_serial)       
        self.ui.disconnectButton.clicked.connect(self.disconnect_serial)
        self.ui.cmdButton.clicked.connect(self.send_parameters)         
        self.ui.onButton.clicked.connect(self.send_on_command)          
        self.ui.offButton.clicked.connect(self.send_off_command)        
        self.ui.IDButton.clicked.connect(self.send_motor_id)            
        self.ui.originButton.clicked.connect(self.send_origin_command)  
        self.ui.helpButton.clicked.connect(self.show_help_dialog)
        self.ui.emergencyButton.clicked.connect(self.send_emergency_stop)       

        self.SCALE      = 100
        self.deg2rad = math.pi / 180
        self.rad2deg = 180 / math.pi
        P_MIN, P_MAX    = -12.5*self.rad2deg, 12.5*self.rad2deg
        V_MIN, V_MAX    = -50.0*self.rad2deg, 50.0*self.rad2deg
        KP_MIN, KP_MAX  = 0.0, 500.0
        KD_MIN, KD_MAX  = 0.0, 5.0
        T_MIN, T_MAX    = -25.0, 25.0

        self.ui.positionSlider.setRange(int(P_MIN*self.SCALE), int(P_MAX*self.SCALE))
        self.ui.velocitySlider.setRange(int(V_MIN*self.SCALE), int(V_MAX*self.SCALE))
        self.ui.kpSlider.setRange(int(KP_MIN*self.SCALE), int(KP_MAX*self.SCALE))
        self.ui.kdSlider.setRange(int(KD_MIN*self.SCALE), int(KD_MAX*self.SCALE))
        self.ui.torqueSlider.setRange(int(T_MIN*self.SCALE), int(T_MAX*self.SCALE))

        self.ui.positionSlider.setValue(0)
        self.ui.velocitySlider.setValue(0)
        self.ui.torqueSlider.setValue(0)

        self.ui.posBox.setRange(P_MIN, P_MAX)
        self.ui.velBox.setRange(V_MIN, V_MAX)
        self.ui.kpBox .setRange(KP_MIN, KP_MAX)
        self.ui.kdBox .setRange(KD_MIN, KD_MAX)
        self.ui.torBox.setRange(T_MIN, T_MAX)
        for box in (self.ui.posBox, self.ui.velBox, self.ui.kpBox, self.ui.kdBox, self.ui.torBox):
            box.setDecimals(2)
            box.setSingleStep(0.01)

        self.ui.positionSlider.valueChanged.connect(
            lambda v: self.ui.posBox.setValue(v / self.SCALE)
        )
        self.ui.velocitySlider.valueChanged.connect(
            lambda v: self.ui.velBox.setValue(v / self.SCALE)
        )
        self.ui.kpSlider.valueChanged.connect(
            lambda v: self.ui.kpBox.setValue(v / self.SCALE)
        )
        self.ui.kdSlider.valueChanged.connect(
            lambda v: self.ui.kdBox.setValue(v / self.SCALE)
        )
        self.ui.torqueSlider.valueChanged.connect(
            lambda v: self.ui.torBox.setValue(v / self.SCALE)
        )

        self.ui.posBox.valueChanged.connect(
            lambda v: self.ui.positionSlider.setValue(int(v * self.SCALE))
        )
        self.ui.velBox.valueChanged.connect(
            lambda v: self.ui.velocitySlider.setValue(int(v * self.SCALE))
        )
        self.ui.kpBox.valueChanged.connect(
            lambda v: self.ui.kpSlider.setValue(int(v * self.SCALE))
        )
        self.ui.kdBox.valueChanged.connect(
            lambda v: self.ui.kdSlider.setValue(int(v * self.SCALE))
        )
        self.ui.torBox.valueChanged.connect(
            lambda v: self.ui.torqueSlider.setValue(int(v * self.SCALE))
        )

    def refresh_serial_ports(self):
        ports = self.serial_manager.list_ports()
        self.ui.serialPortBox.clear()
        self.ui.serialPortBox.addItems(ports)

    def show_message(self, success, message):
        if success:
            QMessageBox.information(self.MainWindow, "Message", message)
        else:
            QMessageBox.warning(self.MainWindow, "Error", message)

    def connect_serial(self):
        port = self.ui.serialPortBox.currentText()
        success, msg = self.serial_manager.connect(port)
        self.show_message(success, msg)
        if success:
            self.reader_thread = SerialReaderThread(self.serial_manager.ser, self.status_parser)
            self.reader_thread.status_received.connect(self.update_output_labels)
            self.reader_thread.start()

    def disconnect_serial(self):
        if hasattr(self, 'reader_thread'):
            self.reader_thread.stop()
            self.reader_thread.wait()
        if self.serial_manager.ser and self.serial_manager.ser.is_open:
            self.serial_manager.ser.close()
            self.show_message(True, "Serial port disconnected.")
        self.refresh_serial_ports()

    def send_motor_id(self):
        motor_id = self.ui.IDText.toPlainText().strip()
        success, msg = self.serial_manager.send_motor_id(motor_id)
        if success:
            self.current_motor_id = int(motor_id)
            self.ui.IDText.clear()
        self.show_message(success, msg)

    def send_on_command(self):
        success, msg = self.serial_manager.send_on()
        self.show_message(success, msg)

    def send_off_command(self):
        success, msg = self.serial_manager.send_off()
        self.show_message(success, msg)

    def send_origin_command(self):
        success, msg = self.serial_manager.send_origin()
        self.show_message(success, msg)

    def show_help_dialog(self):
        self.dialog = QtWidgets.QDialog(self.MainWindow)
        self.help_ui = Ui_Dialog()
        self.help_ui.setupUi(self.dialog)
        pos = self.MainWindow.pos()
        self.dialog.move(pos.x() + self.MainWindow.width() + 10, pos.y())
        self.dialog.setWindowModality(QtCore.Qt.NonModal)
        self.dialog.show()

    def send_parameters(self, suppress_msg = False):
        pos = self.ui.posBox.value()
        vel = self.ui.velBox.value()
        kp  = self.ui.kpBox.value()
        kd  = self.ui.kdBox.value()
        tor = self.ui.torBox.value()

        pos_rad = pos * self.deg2rad
        vel_rad = vel * self.deg2rad

        success, msg = self.parameter_manager.send_parameters(pos_rad, vel_rad, kp, kd, tor)
        if not suppress_msg:
            self.show_message(success, msg)
        return success, msg

    def send_emergency_stop(self):
        self.ui.posBox.setValue(0.0)
        self.ui.velBox.setValue(0.0)
        self.ui.kpBox.setValue(0.0)
        self.ui.kdBox.setValue(0.0)
        self.ui.torBox.setValue(0.0)

        self.send_parameters(suppress_msg=True)

        QMessageBox.critical(
            self.MainWindow,
            "Emergency Stop",
            "Emergency Stop! Set All Parameters into Zero!"
        )

    def update_output_labels(self, parsed):
        pos_rad = float(parsed["pos"])
        vel_rad = float(parsed["vel"])

        pos_deg = pos_rad * self.rad2deg
        vel_deg = vel_rad * self.rad2deg

        self.ui.posOutValue.setText(f"{pos_deg:.2f}")
        self.ui.velOutValue.setText(f"{vel_deg:.2f}")
        self.ui.torOutValue.setText(parsed["tor"])

    def run(self):
        self.MainWindow.show()
        exit_code = self.app.exec_()
        if hasattr(self, 'reader_thread'):
            self.reader_thread.stop()
            self.reader_thread.wait()
        sys.exit(exit_code)

if __name__ == "__main__":
    Main().run()
