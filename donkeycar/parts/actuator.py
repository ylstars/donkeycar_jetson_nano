"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""
import sys
sys.path.append('/home/xiaor/work/donkeycar/donkeycar/parts')
import _XiaoRGEEK_about_ as Readme
import binascii
import os
import time
import threading
import hashlib
import donkeycar as dk

#path_jpg = os.path.dirname(os.path.abspath(sys.argv[0])) + '/XiaoRGEEK.jpg'
path_jpg = os.path.dirname(os.path.abspath(__file__)) + '/XiaoRGEEK.jpg'
with open(path_jpg,"rb") as f:
   s_md5 = hashlib.md5(f.read()).hexdigest()
if (Readme.__title__ == "XiaoRGEEK-raspberry pi-Python Src")&(Readme.__uri__ == "http://www.xiao-r.com")&(Readme.__email__ == "ray_yi@wifi-robots.com")&(s_md5 == '2db99946316b7f849e97fc8735ddc27e'):
   verif = 1
   print ('>>>>>actuator is load Success!')
else:
   verif = 0
   print ('>>>>>actuator is load fail!')



class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """
    def __init__(self, channel):
        #from _XiaoRGEEK_SERVO_ import XR_Servo
        from donkeycar.parts._XiaoRGEEK_SERVO_ import XR_Servo
        self.Servo = XR_Servo()
        self.channel = channel

    def set_pulse(self, pulse):
        self.Servo.XiaoRGEEK_SetServoAngle(self.channel,pulse)

    def savePWM(self):
        self.Servo.XiaoRGEEK_SaveServo();

    def run(self, pulse):
        self.set_pulse(pulse)

class motorPWM:
    def __init__(self,pulse):
        import Jetson.GPIO
        self.GPIO = Jetson.GPIO
        self.GPIO.setmode(self.GPIO.BCM)

        ########电机驱动接口定义#################
        self.ENA = 13	#//L298使能A
        self.ENB = 20	#//L298使能B
        self.IN1 = 19	#//电机接口1
        self.IN2 = 16	#//电机接口2
        self.IN3 = 21	#//电机接口3
        self.IN4 = 26	#//电机接口4
        self.GPIO.setwarnings(False)
        #########电机初始化为LOW##########
        self.GPIO.setup(self.ENA,self.GPIO.OUT,initial=self.GPIO.LOW)
        #self.ENA_pwm=self.GPIO.PWM(self.ENA,1000) 
        #self.ENA_pwm.start(0) 
        #self.ENA_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN1,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN2,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.ENB,self.GPIO.OUT,initial=self.GPIO.LOW)
        #self.ENB_pwm=self.GPIO.PWM(self.ENB,1000) 
        #self.ENB_pwm.start(0) 
        #self.ENB_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN3,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN4,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.zero_pulse = pulse
        self.Motor(self.zero_pulse)

        thread = threading.Thread(target=self.PWM_hangeDutyCycle,args=())
        thread.start()

    def PWM_hangeDutyCycle(self):
        global MOTOR_PWM
        count = 0
        while True:
            if count>100:
                count=0
            if count<MOTOR_PWM:
                self.GPIO.output(self.ENA,True)
                self.GPIO.output(self.ENB,True)
            else :
                self.GPIO.output(self.ENA,False)
                self.GPIO.output(self.ENB,False)
            time.sleep(0.00001)
            count=count+1

    def Motor_Forward(self):
        #self.GPIO.output(self.ENA,True)
        #self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,True)
        self.GPIO.output(self.IN2,False)
        self.GPIO.output(self.IN3,True)
        self.GPIO.output(self.IN4,False)

    def Motor_Backward(self):
        #self.GPIO.output(self.ENA,True)
        #self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,False)
        self.GPIO.output(self.IN2,True)
        self.GPIO.output(self.IN3,False)
        self.GPIO.output(self.IN4,True)

    def Motor(self,pulse):
        global MOTOR_PWM
        if pulse>=100:
            self.Motor_Forward()
            MOTOR_PWM=pulse-100
            #self.ENA_pwm.ChangeDutyCycle(pulse-100)
            #self.ENB_pwm.ChangeDutyCycle(pulse-100)
        else:
            self.Motor_Backward()
            MOTOR_PWM=100-pulse
            #self.ENA_pwm.ChangeDutyCycle(100-pulse)
            #self.ENB_pwm.ChangeDutyCycle(100-pulse)

    def run(self, pulse):
        self.Motor(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,left_pulse=60,
                       right_pulse=110):
        #from _XiaoRGEEK_SERVO_ import XR_Servo
        from donkeycar.parts._XiaoRGEEK_SERVO_ import XR_Servo
        self.Servo = XR_Servo()
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.lastAngle=100

    def run(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.util.data.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        #print(pulse)
        if((self.lastAngle-pulse)>1)|((pulse-self.lastAngle)>1):
            self.Servo.XiaoRGEEK_SetServoAngle(1,pulse)
            self.lastAngle=pulse
            

    def shutdown(self):
        self.run(0) #set steering straight



class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """

    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self,
                       max_pulse=200,
                       min_pulse=0,
                       zero_pulse=100):

        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.last_pulse = 100

        import Jetson.GPIO
        self.GPIO = Jetson.GPIO
        self.GPIO.setmode(self.GPIO.BCM)

        ########电机驱动接口定义#################
        self.ENA = 13	#//L298使能A
        self.ENB = 20	#//L298使能B
        self.IN1 = 19	#//电机接口1
        self.IN2 = 16	#//电机接口2
        self.IN3 = 21	#//电机接口3
        self.IN4 = 26	#//电机接口4
        self.GPIO.setwarnings(False)
        #########电机初始化为LOW##########
        self.GPIO.setup(self.ENA,self.GPIO.OUT,initial=self.GPIO.LOW)
        #self.ENA_pwm=self.GPIO.PWM(self.ENA,1000) 
        #self.ENA_pwm.start(0) 
        #self.ENA_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN1,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN2,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.ENB,self.GPIO.OUT,initial=self.GPIO.LOW)
        #self.ENB_pwm=self.GPIO.PWM(self.ENB,1000) 
        #self.ENB_pwm.start(0) 
        #self.ENB_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN3,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN4,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.zero_pulse = 100
        self.Motor(self.zero_pulse)

        thread = threading.Thread(target=self.PWM_hangeDutyCycle,args=())
        thread.start()

    def PWM_hangeDutyCycle(self):
        global MOTOR_PWM
        count = 0
        while True:
            if count>100:
                count=0
            if count<MOTOR_PWM:
                self.GPIO.output(self.ENA,True)
                self.GPIO.output(self.ENB,True)
            else :
                self.GPIO.output(self.ENA,False)
                self.GPIO.output(self.ENB,False)
            time.sleep(0.00001)
            count=count+1

    def Motor_Forward(self):
        #self.GPIO.output(self.ENA,True)
        #self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,True)
        self.GPIO.output(self.IN2,False)
        self.GPIO.output(self.IN3,True)
        self.GPIO.output(self.IN4,False)

    def Motor_Backward(self):
        #self.GPIO.output(self.ENA,True)
        #self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,False)
        self.GPIO.output(self.IN2,True)
        self.GPIO.output(self.IN3,False)
        self.GPIO.output(self.IN4,True)

    def Motor(self,pulse):
        global MOTOR_PWM
        if pulse>=100:
            self.Motor_Forward()
            MOTOR_PWM=pulse-100
            #self.ENA_pwm.ChangeDutyCycle(pulse-100)
            #self.ENB_pwm.ChangeDutyCycle(pulse-100)
        else:
            self.Motor_Backward()
            MOTOR_PWM=100-pulse
            #self.ENA_pwm.ChangeDutyCycle(100-pulse)
            #self.ENB_pwm.ChangeDutyCycle(100-pulse)

    def run(self, throttle):
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.util.data.map_range(throttle,
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)
        if((self.last_pulse-pulse)>1)|((pulse-self.last_pulse)>1):
            self.Motor(pulse)
            self.last_pulse=pulse
        

    def shutdown(self):
        self.run(0) #stop vehicle
