import time
from MotorBaseClass import MotorBase

if __name__ == "__main__":
    MotorBase.init_serial(port='COM4', baudrate=115200, timeout=1)          
    motor = MotorBase(motor_id=1)                                         
    motor.send_id()                                                         
    motor.on()                                                              

    for idx in range(4):
        Rest = (idx) % 2
        if Rest == 0:
            motor.update_parameters(pos=1.57, vel=2.0, kp=5.0, kd=1.0, tor=0.0)
            pos_out, vel_out, tor_out = motor.get_parameters()
            print(f"{idx+1}th Current Status: {pos_out} rad, {vel_out} rad/s, {tor_out} Nm")
            time.sleep(5)
        else:
            motor.update_parameters(pos=-1.57, vel=2.0, kp=5.0, kd=1.0, tor=0.0)
            pos_out, vel_out, tor_out = motor.get_parameters()
            print(f"{idx+1}th Current Status: {pos_out} rad, {vel_out} rad/s, {tor_out} Nm")
            time.sleep(5)

    time.sleep(2)
    motor.off()
        
