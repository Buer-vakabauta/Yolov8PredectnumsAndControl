import RPi.GPIO as GPIO
import lgpio
import time
import numpy as np
import math
from LaserTracker import laser_track
from camera import CameraCapture
# 舵机 GPIO 设置
H_SERVO_PIN = 17  # 左右
V_SERVO_PIN = 27  # 上下
MIN_ANGLE = 105
MAX_ANGLE = 150
import lgpio
import time
import numpy as np
import math
from LaserTracker import laser_track
from camera import CameraCapture

# 舵机 GPIO 设置
H_SERVO_PIN = 17  # 左右
V_SERVO_PIN = 27  # 上下
MIN_ANGLE = 90
MAX_ANGLE = 150

class Servo2:
    def __init__(self, h_angle=135, v_angle=135):
        '''
        初始化舵机参数
        Args:
            h_angle: 水平角度
            v_angle: 垂直角度
        '''
        self.inital_angle_x=h_angle
        self.inital_angle_y=v_angle
        self.h_angle = h_angle
        self.v_angle = v_angle
        self.disx = 0  # 激光到平面的垂直距离
        self.disy = 0
        self.inital_x = 0  # 初始X坐标
        self.inital_y = 0  # 初始Y坐标
        
        # 打开 GPIO 芯片
        self.h = lgpio.gpiochip_open(0)

        # 设置引脚为输出模式
        lgpio.gpio_claim_output(self.h, H_SERVO_PIN)
        lgpio.gpio_claim_output(self.h, V_SERVO_PIN)
        
        # 设置初始角度
        self.set_servo_angle(self.h_angle, self.v_angle, 0.5)
    
    def angle_to_pulse_width(self, angle):
        '''
        将角度转换为脉宽（微秒）
        Args:
            angle: 角度值
        Returns:
            pulse_width: 脉宽（微秒）
        '''
        angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
        # 舵机控制：0.5ms-2.5ms 对应 0-270度
        pulse_width = 500 + (angle / 270.0) * 2000  # 500-2500微秒
        return int(pulse_width)
    
    def send_servo_pulse(self, pin, angle, count=1):
        '''
        发送舵机控制脉冲
        Args:
            pin: GPIO引脚
            angle: 角度
            count: 发送脉冲次数
        '''
        if angle <= 0:  # 角度为-1或负数时不发送
            return
            
        pulse_width = self.angle_to_pulse_width(angle)
        period = 20000  # 20ms周期
        
        for _ in range(count):
            # 发送高电平脉冲
            lgpio.gpio_write(self.h, pin, 1)
            time.sleep(pulse_width / 1000000.0)  # 转换为秒
            
            # 发送低电平
            lgpio.gpio_write(self.h, pin, 0)
            time.sleep((period - pulse_width) / 1000000.0)
    
    def set_servo_angle(self, angle_h, angle_v, timesleep=0.2):
        '''
        控制舵机转动到指定角度
        角度传入-1表示不转动
        Args:
            angle_h: 水平转到的角度
            angle_v: 垂直转到的角度
            timesleep: 设定给舵机的反应时间
        '''
        # 发送多个脉冲确保舵机到位
        pulse_count = max(1, int(timesleep * 50))  # 根据延时计算脉冲数
        
        if angle_h > 0:
            self.send_servo_pulse(H_SERVO_PIN, angle_h, pulse_count)
            self.h_angle = angle_h
        
        if angle_v > 0:
            self.send_servo_pulse(V_SERVO_PIN, angle_v, pulse_count)
            self.v_angle = angle_v
        
        if timesleep > 0:
            time.sleep(timesleep)
        
        #print(self.h_angle, self.v_angle)
    
    def move2pos(self, x, y, posx, posy):
        '''
        PID将光点向指定位置靠近
        Args:
            x: 当前位置X
            y: 当前位置Y
            posx: 目标位置X
            posy: 目标位置Y
        '''
        if abs(posx-x)>15:
            anglex = self.h_angle - 0.005 * (posx - x)
        else:
            anglex=-1
        if abs(posy-y)>15:
            angley = self.v_angle + 0.005 * (posy - y)
        else:
            angley=-1
        self.set_servo_angle(anglex, angley, 0.1)
    
    def get_inital_pos(self, camera):
        '''
        获取初始位置
        Args:
            camera: 摄像头捕获类:CameraCapture
        '''
        while True:
            frame = camera.get_frame()
            if frame is None:
                continue
            self.inital_x, self.inital_y = laser_track(frame)
            if self.inital_x and self.inital_y:
                return
    
    def cal_dis(self, camera):
        '''
        计算激光笔到平面的垂直距离，需要先获取激光笔的初始位置
        Args:
            camera: 摄像头捕获类:CameraCapture
        '''
        self.set_servo_angle(self.h_angle-20, self.v_angle+10, 0.2)
        time.sleep(0.5)
        while True:
            frame = camera.get_frame()
            x2, y2 = laser_track(frame)
            if x2 and y2:
                self.disx = (x2 - self.inital_x) / math.tan(20 * math.pi / 180)
                self.disy = (y2 - self.inital_y) / math.tan(10 * math.pi / 180)
                print("距离测定完成,dis=", self.disx, self.disy)
                break
    
    def destroy(self):
        '''
        析构函数
        '''
        try:
            # 释放GPIO引脚
            lgpio.gpio_free(self.h, H_SERVO_PIN)
            lgpio.gpio_free(self.h, V_SERVO_PIN)
        except Exception as e:
            print(f"释放引脚时出错: {e}")
        
        try:
            # 关闭GPIO芯片
            lgpio.gpiochip_close(self.h)
        except Exception as e:
            print(f"关闭GPIO芯片时出错: {e}")
"""
class Servo:
    def __init__(self,h_angle=135,v_angle=135):
        '''
        初始化舵机参数
        Args:
            h_angle:水平角度
            v_angle:垂直角度

        '''
        self.h_angle=h_angle
        self.v_angle=v_angle
        self.disx=0#激光到平面的垂直距离
        self.disy=0
        self.inital_x=0#初始X坐标
        self.inital_y=0#初始Y坐标
        # 初始化 GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(H_SERVO_PIN, GPIO.OUT)
        GPIO.setup(V_SERVO_PIN, GPIO.OUT)

        # 初始化 PWM
        self.pwm_h = GPIO.PWM(H_SERVO_PIN, 50)
        self.pwm_v = GPIO.PWM(V_SERVO_PIN, 50)
        self.pwm_h.start(0)
        self.pwm_v.start(0)
        self.set_servo_angle(self.h_angle, self.v_angle,0.2)

    def set_servo_angle(self,angle_h,angle_v,timesleep=0.2):
        '''
        控制舵机转动到指定角度
        角度传入-1表示不转动
        Args:
            angle1:水平转到的角度
            angle2:垂直转到的角度
            timesleep:设定给舵机的反应时间

        '''
        if angle_h>0:
            angle = max(MIN_ANGLE, min(MAX_ANGLE, angle_h))
            duty = 2.5 + (angle / 270.0) * 10
            self.pwm_h.ChangeDutyCycle(duty)
            self.h_angle=angle_h
        if angle_v>0:
            angle = max(MIN_ANGLE, min(MAX_ANGLE, angle_v))
            duty = 2.5 + (angle / 270.0) * 10
            self.pwm_v.ChangeDutyCycle(duty)
            self.v_angle=angle_v
        if timesleep>0:
            time.sleep(timesleep)
            self.pwm_v.ChangeDutyCycle(0)
            self.pwm_h.ChangeDutyCycle(0)
        print(self.h_angle,self.v_angle)
    def move2pos(self,x,y,posx,posy):
        '''
        PID将光点向指定位置靠近
        Args:
            X:当前位置X
            Y:当前位置Y
            posx:目标位置X
            posy:目标位置Y
        '''
        
        if abs(posx-x)>50:#限幅，避免过大抖动
            anglex=self.h_angle-0.25
        else:
            anglex=self.h_angle-0.005*(posx-x)
        if abs(posy-y)>50:#限幅，避免过大抖动
            angley=self.v_angle+0.25
        else:
            angley=self.v_angle+0.005*(posy-y)
        
        #print(anglex,angley)
        self.set_servo_angle(anglex,angley,0.1)
    def get_inital_pos(self,camera):
        '''
        获取初始位置
        Args:
            camera:摄像头捕获类:CameraCapture
        '''
        while True:
            frame = camera.get_frame()
            if frame is None:
                continue
            self.inital_x, self.inital_y = laser_track(frame)
            if self.inital_x and self.inital_y:
                return
    def cal_dis(self,camera):
        '''
        计算激光笔到平面的垂直距离，需要先获取激光笔的初始位置
        Args:
            Camera:摄像头捕获i类:CameraCapture
        '''
        self.set_servo_angle(self.h_angle-20,self.v_angle+15,0.2)
        time.sleep(0.5)
        while True:
            frame=camera.get_frame()
            x2, y2 = laser_track(frame)
            #cv2.imshow("Servo_test", frame)
            #key = cv2.waitKey(100) & 0xFF
            if x2 and y2:
                self.disx=(x2-self.inital_x)/math.tan(20*math.pi/180)
                self.disy=(y2-self.inital_y)/math.tan(15*math.pi/180)
                print("距离测定完成,dis=",self.disx,self.disy)
                #servo.set_servo_angle(135,135,0.2)
                break

    def destory(self):
        '''
        析构函数yig
        '''
        self.pwm_h.stop()
        self.pwm_v.stop()
        GPIO.cleanup()
"""

"""
#测试代码：

# OpenCV窗口初始化
import cv2
from camera import CameraCapture
from LaserTracker import laser_track
# 全局目标点变量和追踪标志
target_x, target_y = 320, 320
tracking = 0

# 鼠标点击回调函数
def mouse_callback(event, x, y, flags, param):
    global target_x, target_y, tracking
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        tracking=1
        print(f"设置目标点：({target_x}, {target_y})")

servo=Servo2(125,135)
camera = CameraCapture(640, 640, 30)
camera.start()
STEP = 5
cv2.namedWindow("Servo_test", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Servo_test", 800, 600)
cv2.setMouseCallback("Servo_test", mouse_callback)
time.sleep(0.5)
servo.get_inital_pos(camera)
time.sleep(0.5)
servo.cal_dis(camera)
#servo.dis=750

while True:
    frame = camera.get_frame()
    if frame is None:
        continue

    cx, cy = laser_track(frame)
    if cx and cy:
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 若启用追踪，则进行角度调整
        if tracking:
            if abs(cx-target_x)<=15 and abs(cy-target_y)<=15:
                tracking=False
                #servo.pwm_h.ChangeDutyCycle(0)
                #servo.pwm_v.ChangeDutyCycle(0)
                print("追踪完成")
            else:
                if tracking==1:
                    #dis=(servo.disx+servo.disy)/2
                    a=math.atan((target_x-servo.inital_x)/(servo.disx))*180/math.pi
                    b=math.atan((target_y-servo.inital_y)/(servo.disy))*180/math.pi
                    #print("水平/垂直:",a,b)
                    servo.set_servo_angle(125-a,135+b,0.1)
                    tracking=2
                elif tracking==2:
                    servo.move2pos(cx, cy, target_x, target_y)

    # 显示目标点位置
    cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
    #cv2.putText(frame, f"TGT({target_x},{target_y})", (target_x+10, target_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow("Servo_test", frame)
    key = cv2.waitKey(100) & 0xFF

    if key == ord('w'):
        v_angle = min(servo.v_angle - STEP, MAX_ANGLE)
        servo.set_servo_angle(-1, v_angle)
    elif key == ord('s'):
        v_angle = max(servo.v_angle + STEP, MIN_ANGLE)
        servo.set_servo_angle(-1, v_angle)
    elif key == ord('a'):
        h_angle = max(servo.h_angle + STEP, MIN_ANGLE)
        servo.set_servo_angle(h_angle, -1)
    elif key == ord('d'):
        h_angle = min(servo.h_angle - STEP, MAX_ANGLE)
        servo.set_servo_angle(h_angle, -1)
    elif key == ord('t'):
        tracking = not tracking  # 开启/关闭追踪
        print(f"{'开始' if tracking else '停止'}追踪")
    elif key == ord('q'):
        break

#servo.pwm_h.stop()
#servo.pwm_v.stop()
camera.stop()
servo.destroy()
#GPIO.cleanup()
cv2.destroyAllWindows()

"""