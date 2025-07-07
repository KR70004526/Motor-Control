"""
    This code is a GUI application for controlling CubeMars AK70-10 Motor via Arduino.
    It allows users to connect to the motor, send commands, and display motor status.
    For more information to use this GUI, Please refer to the help dialog which can be opened by clicking the help button.
"""

# IMPORT LIBRARIES
import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QMessageBox
from PyQt5 import QtCore, QtGui, QtWidgets
from Manual_Dialog import Ui_Dialog         # Import the help dialog UI class

# DEFINE FONT FOR GUI
font = QtGui.QFont()
font.setFamily("Arial")
font.setPointSize(12)
font.setBold(True)
font.setWeight(75)

# DEFINE SERIAL PORT MANAGER CLASS
# This class handles the serial port connection and communication with the motor.
class SerialPortManager:
    def __init__(self):
        self.ser = None
    
    # List available serial ports
    def list_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]
    
    # Connect to the specified serial port
    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            if self.ser.is_open:
                return True, "Connected to port successfully."
        except serial.SerialException as e:
            print(f"Error connecting to port {port}: {e}")
        return False, "Failed to connect to port."
    
    # Send motor ID to the motor
    def send_motor_id(self, motor_id: str):
        if not self.ser or not self.ser.is_open:
            return False, "Please Connect Serial Port First."
        if not motor_id.isdigit():
            return False, "Motor ID must be a number."
        motor_id = int(motor_id)
        motor_id = max(0, min(motor_id, 127))
        try:
            self.ser.write(f"ID:{motor_id}\n".encode())
            return True, f"Successfully Send Motor ID {motor_id}!!"
        except Exception as e:
            return False, str(e)

    # Send command to the motor    
    def send_command(self, command: str):
        if not self.ser or not self.ser.is_open:
            return False, "Please Connect Serial Port First."
        try:
            self.ser.write(f"{command}\n".encode())
            return True, f"Command '{command}' sent successfully."
        except Exception as e:
            return False, f"Failed to send command: {e}"

    # Send ON command to the motor    
    def send_on(self):
        return self.send_command("ON")
    
    # Send OFF command to the motor
    def send_off(self):
        return self.send_command("OFF")

    # Send ORIGIN command to the motor
    def send_origin(self):
        return self.send_command("ORIGIN")
    
# PARAMETER MANAGER CLASS
# This class handles sending parameters to the motor.    
class ParameterManager:
    def __init__(self, serial_manager):
        self.serial_manager = serial_manager

    # Send parameters to the motor
    def send_parameters(self, position, velocity, kp, kd, torque):
        if not self.serial_manager.ser or not self.serial_manager.ser.is_open:
            return False, "Please Connect Serial Port First."
        
        try:
            pos = float(position)
            vel = float(velocity)
            kp = float(kp)
            kd = float(kd)
            tor = float(torque)
        except ValueError:
            return False, "Invalid input. Please enter numeric values."
        
        command = f"SEND POS:{pos},VEL:{vel},KP:{kp},KD:{kd},TOR:{tor}"
        try:
            self.serial_manager.ser.write(f"{command}\n".encode())
            return True, "Parameters sent successfully."
        except Exception as e:
            return False, f"Failed to send parameters: {e}"

# MOTOR STATUS PARSER CLASS
# This class parses the motor status messages received from the motor.
class MotorStatusParser:
    def parse_line(self, line):
        if line.startswith("Position:"):
            try:
                parts = line.split(',')
                pos = parts[0].split(':')[1].strip()
                vel = parts[1].split(':')[1].strip()
                tor = parts[2].split(':')[1].strip()
                return {"pos": pos, "vel": vel, "tor": tor}
            except:
                return None
        return None

# SERIAL READER THREAD CLASS
# This class runs in a separate thread to read data from the serial port continuously.
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

# GUI CLASS
# This class sets up the GUI layout and connects the UI elements to the functionality.
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # Main Window Setup
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(430, 550)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # Setup Frame Section
        self.setupFrame = QtWidgets.QFrame(self.centralwidget)
        self.setupFrame.setGeometry(QtCore.QRect(10, 10, 410, 130))
        self.setupFrame.setStyleSheet("border: 2px solid gray")
        self.setupFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setupFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.setupFrame.setObjectName("setupFrame")

        # Help Button Section
        self.helpButton = QtWidgets.QPushButton(self.setupFrame)
        self.helpButton.setGeometry(QtCore.QRect(210, 90, 90, 30))
        self.helpButton.setFont(font)
        self.helpButton.setStyleSheet("border: 2px solid black")
        self.helpButton.setObjectName("helpButton")


        # Serial Port Section
        self.serialPortLabel = QtWidgets.QLabel(self.setupFrame)
        self.serialPortLabel.setGeometry(QtCore.QRect(10, 10, 90, 30))
        self.serialPortLabel.setFont(font)
        self.serialPortLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.serialPortLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.serialPortLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.serialPortLabel.setObjectName("serialPortLabel")

        # Motor ID Section
        self.motorIDLabel = QtWidgets.QLabel(self.setupFrame)
        self.motorIDLabel.setGeometry(QtCore.QRect(10, 50, 90, 30))
        self.motorIDLabel.setFont(font)
        self.motorIDLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.motorIDLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.motorIDLabel.setObjectName("motorIDLabel")

        # Connect Button Section
        self.connectButton = QtWidgets.QPushButton(self.setupFrame)
        self.connectButton.setGeometry(QtCore.QRect(310, 10, 90, 30))
        self.connectButton.setFont(font)
        self.connectButton.setStyleSheet("border: 2px solid black")
        self.connectButton.setObjectName("connectButton")

        # ON/OFF Buttons Section
        self.onButton = QtWidgets.QPushButton(self.setupFrame)
        self.onButton.setGeometry(QtCore.QRect(10, 90, 90, 30))
        self.onButton.setFont(font)
        self.onButton.setStyleSheet("border: 2px solid black")
        self.onButton.setObjectName("onButton")
        self.offButton = QtWidgets.QPushButton(self.setupFrame)
        self.offButton.setGeometry(QtCore.QRect(110, 90, 90, 30))
        self.offButton.setFont(font)
        self.offButton.setStyleSheet("border: 2px solid black")
        self.offButton.setObjectName("offButton")

        # Serial Port Selection Box
        self.serialPortBox = QtWidgets.QComboBox(self.setupFrame)
        self.serialPortBox.setGeometry(QtCore.QRect(110, 10, 150, 30))
        self.serialPortBox.setStyleSheet("background-color: rgb(255,255,255); border: 1px solid black;")
        self.serialPortBox.setObjectName("serialPortBox")
        self.serialPortBox.setFont(font)

        # Serial Port Refresh Button Section
        self.refreshButton = QtWidgets.QPushButton(self.setupFrame)
        self.refreshButton.setGeometry(QtCore.QRect(270, 10, 30, 30))
        self.refreshButton.setStyleSheet("border: 2px solid black")
        self.refreshButton.setObjectName("refreshButton")
        self.refreshButton.setIcon(QtGui.QIcon(r"C:\Users\USER\Desktop\Python\refresh.png"))
        self.refreshButton.setIconSize(QtCore.QSize(20, 20))

        # Motor ID Input Box
        self.IDText = QtWidgets.QLineEdit(self.setupFrame)
        self.IDText.setGeometry(QtCore.QRect(110, 50, 190, 30))
        self.IDText.setStyleSheet("border: 1px solid black")
        self.IDText.setObjectName("IDText")
        self.IDText.setFont(font)

        # Motor ID Send Button Section
        self.sendIDButton = QtWidgets.QPushButton(self.setupFrame)
        self.sendIDButton.setGeometry(QtCore.QRect(310, 50, 90, 30))
        self.sendIDButton.setFont(font)
        self.sendIDButton.setStyleSheet("border: 2px solid black")
        self.sendIDButton.setObjectName("sendIDButton")
        self.sendIDButton.setText("SEND ID")
        self.sendIDButton.setFont(font)

        # Input Frame Section
        self.inputFrame = QtWidgets.QFrame(self.centralwidget)
        self.inputFrame.setGeometry(QtCore.QRect(10, 150, 410, 250))
        self.inputFrame.setStyleSheet("border: 2px solid gray")
        self.inputFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.inputFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.inputFrame.setObjectName("inputFrame")

        # Position Label Section
        self.positionLabel = QtWidgets.QLabel(self.inputFrame)
        self.positionLabel.setGeometry(QtCore.QRect(10, 10, 110, 30))
        self.positionLabel.setFont(font)
        self.positionLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.positionLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.positionLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.positionLabel.setObjectName("positionLabel")

        # Velocity Label Section
        self.velocityLabel = QtWidgets.QLabel(self.inputFrame)
        self.velocityLabel.setGeometry(QtCore.QRect(10, 50, 110, 30))
        self.velocityLabel.setFont(font)
        self.velocityLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.velocityLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.velocityLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.velocityLabel.setObjectName("velocityLabel")

        # Kd Label Section
        self.KdLabel = QtWidgets.QLabel(self.inputFrame)
        self.KdLabel.setGeometry(QtCore.QRect(10, 130, 110, 30))
        self.KdLabel.setFont(font)
        self.KdLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.KdLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.KdLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.KdLabel.setObjectName("KdLabel")

        # Kp Label Section
        self.KpLabel = QtWidgets.QLabel(self.inputFrame)
        self.KpLabel.setGeometry(QtCore.QRect(10, 90, 110, 30))
        self.KpLabel.setFont(font)
        self.KpLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.KpLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.KpLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.KpLabel.setObjectName("KpLabel")

        # Torque Label Section
        self.TorqueLabel = QtWidgets.QLabel(self.inputFrame)
        self.TorqueLabel.setGeometry(QtCore.QRect(10, 170, 110, 30))
        self.TorqueLabel.setFont(font)
        self.TorqueLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.TorqueLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.TorqueLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.TorqueLabel.setObjectName("TorqueLabel")

        # Position Input Section
        self.positionText = QtWidgets.QLineEdit(self.inputFrame)
        self.positionText.setGeometry(QtCore.QRect(130, 10, 170, 30))
        self.positionText.setStyleSheet("border: 1px solid black")
        self.positionText.setObjectName("positionText")
        self.positionText.setFont(font)

        # Velocity Input Section
        self.velocityText = QtWidgets.QLineEdit(self.inputFrame)
        self.velocityText.setGeometry(QtCore.QRect(130, 50, 170, 30))
        self.velocityText.setStyleSheet("border: 1px solid black")
        self.velocityText.setObjectName("velocityText")
        self.velocityText.setFont(font)

        # Kp Input Section
        self.KpText = QtWidgets.QLineEdit(self.inputFrame)
        self.KpText.setGeometry(QtCore.QRect(130, 90, 170, 30))
        self.KpText.setStyleSheet("border: 1px solid black")
        self.KpText.setObjectName("KpText")
        self.KpText.setFont(font)

        # Kd Input Section
        self.KdText = QtWidgets.QLineEdit(self.inputFrame)
        self.KdText.setGeometry(QtCore.QRect(130, 130, 170, 30))
        self.KdText.setStyleSheet("border: 1px solid black")
        self.KdText.setObjectName("KdText")
        self.KdText.setFont(font)

        # Torque Input Section
        self.TorqueText = QtWidgets.QLineEdit(self.inputFrame)
        self.TorqueText.setGeometry(QtCore.QRect(130, 170, 170, 30))
        self.TorqueText.setStyleSheet("border: 1px solid black")
        self.TorqueText.setObjectName("TorqueText")
        self.TorqueText.setFont(font)

        # Command Send Label Section
        self.commandLabel = QtWidgets.QLabel(self.inputFrame)
        self.commandLabel.setGeometry(QtCore.QRect(10, 210, 390, 30))
        self.commandLabel.setFont(font)
        self.commandLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.commandLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.commandLabel.setObjectName("commandLabel")

        # Send Button Section
        self.sendButton = QtWidgets.QPushButton(self.inputFrame)
        self.sendButton.setGeometry(QtCore.QRect(310, 10, 90, 30))
        self.sendButton.setFont(font)
        self.sendButton.setStyleSheet("border: 2px solid black")
        self.sendButton.setObjectName("sendButton")

        self.originButton = QtWidgets.QPushButton(self.inputFrame)
        self.originButton.setGeometry(QtCore.QRect(310, 50, 90, 30))
        self.originButton.setFont(font)
        self.originButton.setStyleSheet("border: 2px solid black")
        self.originButton.setObjectName("originButton")
        self.originButton.setText("ORIGIN")

        # Output Frame Section
        self.outputFrame = QtWidgets.QFrame(self.centralwidget)
        self.outputFrame.setGeometry(QtCore.QRect(10, 410, 410, 130))
        self.outputFrame.setStyleSheet("border: 2px solid gray")
        self.outputFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.outputFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.outputFrame.setObjectName("outputFrame")

        # Position Output Label Section
        self.posOutLabel = QtWidgets.QLabel(self.outputFrame)
        self.posOutLabel.setGeometry(QtCore.QRect(10, 10, 111, 30))
        self.posOutLabel.setFont(font)
        self.posOutLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.posOutLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.posOutLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.posOutLabel.setObjectName("posOutLabel")

        # Velocity Output Label Section
        self.velOutLabel = QtWidgets.QLabel(self.outputFrame)
        self.velOutLabel.setGeometry(QtCore.QRect(10, 50, 111, 30))
        self.velOutLabel.setFont(font)
        self.velOutLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.velOutLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.velOutLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.velOutLabel.setObjectName("velOutLabel")

        # Torque Output Label Section
        self.torOutLabel = QtWidgets.QLabel(self.outputFrame)
        self.torOutLabel.setGeometry(QtCore.QRect(10, 90, 111, 30))
        self.torOutLabel.setFont(font)
        self.torOutLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.torOutLabel.setStyleSheet("background-color:rgb(255,255,255); border: 2px solid black;")
        self.torOutLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.torOutLabel.setObjectName("torOutLabel")

        # Position Output Value Section
        self.posOutLabel_2 = QtWidgets.QLabel(self.outputFrame)
        self.posOutLabel_2.setGeometry(QtCore.QRect(130, 10, 270, 30))
        self.posOutLabel_2.setStyleSheet("background-color:rgb(255,255,255); border: 1px solid black")
        self.posOutLabel_2.setText("")
        self.posOutLabel_2.setObjectName("posOutLabel_2")

        # Velocity Output Value Section
        self.velOutLabel_2 = QtWidgets.QLabel(self.outputFrame)
        self.velOutLabel_2.setGeometry(QtCore.QRect(130, 50, 270, 30))
        self.velOutLabel_2.setStyleSheet("background-color:rgb(255,255,255); border: 1px solid black")
        self.velOutLabel_2.setText("")
        self.velOutLabel_2.setObjectName("velOutLabel_2")

        # Torque Output Value Section
        self.torOutLabel_2 = QtWidgets.QLabel(self.outputFrame)
        self.torOutLabel_2.setGeometry(QtCore.QRect(130, 90, 270, 30))
        self.torOutLabel_2.setStyleSheet("background-color:rgb(255,255,255); border: 1px solid black")
        self.torOutLabel_2.setText("")
        self.torOutLabel_2.setObjectName("torOutLabel_2")

        # Set Main Window Central Widget
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Motor Control GUI"))
        self.helpButton.setText(_translate("MainWindow", "Help"))
        self.serialPortLabel.setText(_translate("MainWindow", "Serial Port"))
        self.motorIDLabel.setText(_translate("MainWindow", "Motor ID"))
        self.connectButton.setText(_translate("MainWindow", "CONNECT"))
        self.onButton.setText(_translate("MainWindow", "ON"))
        self.offButton.setText(_translate("MainWindow", "OFF"))
        self.positionLabel.setText(_translate("MainWindow", "Position"))
        self.velocityLabel.setText(_translate("MainWindow", "Velocity"))
        self.KdLabel.setText(_translate("MainWindow", "Kd"))
        self.KpLabel.setText(_translate("MainWindow", "Kp"))
        self.TorqueLabel.setText(_translate("MainWindow", "Torque"))
        self.sendButton.setText(_translate("MainWindow", "SEND"))
        self.posOutLabel.setText(_translate("MainWindow", "Position"))
        self.velOutLabel.setText(_translate("MainWindow", "Velocity"))
        self.torOutLabel.setText(_translate("MainWindow", "Torque"))

# MAIN CLASS
# This class initializes the application, sets up the main window, and connects the UI elements to their respective functions.
class Main:
    def __init__(self):
        self.app = QtWidgets.QApplication([])
        self.MainWindow = QtWidgets.QMainWindow()
        self.current_motor_id = None
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.MainWindow)

        # Connect to the classes for GUI application
        self.serial_manager = SerialPortManager()
        self.paramaeter_manager = ParameterManager(self.serial_manager)
        self.parse_motor_status = MotorStatusParser()

        # Refresh the serial ports on startup
        self.refresh_serial_ports()

        # Connect UI elements to their respective functions
        self.ui.connectButton.clicked.connect(self.connect_serial)          # Connect to the serial port
        self.ui.sendButton.clicked.connect(self.send_parameters)            # Send parameters to the motor
        self.ui.onButton.clicked.connect(self.send_on_command)              # Send ON command to the motor
        self.ui.offButton.clicked.connect(self.send_off_command)            # Send OFF command to the motor   
        self.ui.sendIDButton.clicked.connect(self.send_motor_id)            # Send motor ID to the motor
        self.ui.originButton.clicked.connect(self.send_origin_command)      # Send ORIGIN command to the motor
        self.ui.refreshButton.clicked.connect(self.refresh_serial_ports)    # Refresh the serial ports
        self.ui.helpButton.clicked.connect(self.show_help_dialog)           # Show help dialog
        self.ui.refreshButton.clicked.connect(self.refresh_serial_ports)    # Refresh the serial ports

    # Function to refresh the list of available serial ports
    def refresh_serial_ports(self):
        ports = self.serial_manager.list_ports()
        self.ui.serialPortBox.clear()
        self.ui.serialPortBox.addItems(ports)

    # Function to show messages to the user
    def show_message(self, success, message):
        if success:
            QMessageBox.information(None, "Message", message)
        else:
            QMessageBox.warning(None, "Error Message", message)

    # Function to connect to the selected serial port
    def connect_serial(self):
        port = self.ui.serialPortBox.currentText()
        success, msg = self.serial_manager.connect(port)
        self.show_message(success, msg)

        if success:
            self.reader_thread = SerialReaderThread(self.serial_manager.ser, self.parse_motor_status)
            self.reader_thread.status_received.connect(self.update_output_labels)
            self.reader_thread.start()
    
    # Function to send motor ID to the motor
    def send_motor_id(self):
        motor_id = self.ui.IDText.text().strip()
        success, msg = self.serial_manager.send_motor_id(motor_id)
        if success:
            self.current_motor_id = int(motor_id)
            self.ui.IDText.clear()
        self.show_message(success, msg)

    # Function to send ON command to the motor
    def send_on_command(self):
        success, msg = self.serial_manager.send_on()
        self.show_message(success, msg)
    
    # Function to send OFF command to the motor
    def send_off_command(self):
        success, msg = self.serial_manager.send_off()
        self.show_message(success, msg)

    # Function to send ORIGIN command to the motor
    def send_origin_command(self):
        success, msg = self.serial_manager.send_origin()
        self.show_message(success, msg)

    # Function to show the help dialog
    def show_help_dialog(self):
        self.dialog = QtWidgets.QDialog()
        self.help_ui = Ui_Dialog()
        self.help_ui.setupUi(self.dialog)
        parent_pos = self.MainWindow.pos() if hasattr(self, "MainWindow") else QtCore.QPoint(100, 100)
        self.dialog.move(parent_pos.x() + 435, parent_pos.y())
        self.dialog.setWindowModality(QtCore.Qt.NonModal)
        self.dialog.show()

    # Function to send parameters to the motor
    def send_parameters(self):
        pos = self.ui.positionText.text().strip()
        vel = self.ui.velocityText.text().strip()
        kp = self.ui.KpText.text().strip()
        kd = self.ui.KdText.text().strip()
        tor = self.ui.TorqueText.text().strip()

        motor_id = self.current_motor_id if self.current_motor_id else "N/A"
        command_str = f"ID: {motor_id} POS: {pos} VEL: {vel} KP: {kp} KD: {kd} TOR: {tor}"
        self.ui.commandLabel.setText(command_str)

        self.paramaeter_manager.send_parameters(pos, vel, kp, kd, tor)

        self.ui.positionText.clear()
        self.ui.velocityText.clear()
        self.ui.KpText.clear()
        self.ui.KdText.clear()
        self.ui.TorqueText.clear()
    
    # Function to update the output labels with the parsed motor status
    def update_output_labels(self, parsed):
        self.ui.posOutLabel_2.setText(parsed["pos"])
        self.ui.velOutLabel_2.setText(parsed["vel"])
        self.ui.torOutLabel_2.setText(parsed["tor"])

    def run(self):
        self.MainWindow.show()
        exit_code = self.app.exec_()
        if hasattr(self, 'reader_thread'):
            self.reader_thread.stop()
            self.reader_thread.wait()
        sys.exit(exit_code)

if __name__ == "__main__":
    main = Main()
    main.run()
