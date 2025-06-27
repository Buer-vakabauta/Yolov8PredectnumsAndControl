from camera import CameraCapture
from Servo import Servo2
from yolodect import YOLODetector
from serial_task import Ser
#from LaserTracker import laser_track
from detect import detect_digit,tracking_digit,get_max_digit
import cv2
import time
import camera
#初始参数设置----------------------------------
MODEL_PATH = "best2.onnx"#模型位置            |                  
CAMERA_WIDTH = 640#摄像头宽                   |
CAMERA_HEIGHT = 640#摄像头高                  |
CAMERA_FPS = 30#摄像头帧率
#--------------------------------------------
#初始化-----------------------------------------------------------------
detector = YOLODetector(MODEL_PATH)
servo = Servo2(128,130)
camera = CameraCapture(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
camera.start()
ser=Ser()
ser.start()
servo.get_inital_pos(camera)#测定距离手动测定确保稳定性
#time.sleep(0.5)
#servo.cal_dis(camera)
servo.disx=810
servo.disy=820
print("初始化完成")
#----------------------------------------------------------------------

def task1():
    print("等待目标数字")
    while True:
        target_digit=ser.get_message()
        if camera.exit_flag:
            break
        elif target_digit=="task2":
            break
        if target_digit in detector.class_names:
                print("接受到目标:",ser.message)
                mode = "detect"  # 可为 'detect' 或 'tracking'
                target_x, target_y = 320, 320
                #cv2.namedWindow("num_track", cv2.WINDOW_NORMAL)
                #cv2.resizeWindow("num_track", 800, 640)
                while True:
                    if camera.exit_flag:
                        break
                    frame = camera.get_frame()
                    if frame is None:
                        continue
                    # 转换为BGR格式（OpenCV）
                    #frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    if mode=="detect":
                        target_x,target_y=detect_digit(frame,detector,target_digit)
                        if target_x and target_y:
                            mode="tracking"#进入tracking追踪模式
                            #cx,cy=laser_track(frame)#先进行一次位置预测，加快寻找速度
                            tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo,1)
                    if mode=="tracking":
                        #绘制红点位
                        #cx, cy = laser_track(frame)
#                        if cx and cy:
#                            cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)
#                            cv2.putText(frame_bgr, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        if camera.laser_x is None or camera.laser_y is None:
                            print("未检测到红外线，进行归位")
                            servo.set_servo_angle(128,130,0.5)
                            tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo,1)
                            continue
                        
                        if tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo):
                            ser.message=None
                            target_digit=None
                            ser.send("Y")
                            break    


def task2():
    flag=0
    target_digit=None
    while True:
        message=ser.get_message()
        if camera.exit_flag:
            break
        elif message=="task1":
            break
        if message=="X" and flag==0:
            print("开始识别数字")
            while True:
                if camera.exit_flag:
                    break
                frame=camera.get_frame()
                if frame is None:
                    continue
                target_digit=str(get_max_digit(detector,frame))
                if target_digit:
                    message=None
                    ser.message=None
                    flag=1
                    ser.send(target_digit)         
                    break
        if message=="X" and flag==1:
            print("开始追踪数字")
            mode = "detect"  # 可为 'detect' 或 'tracking'
            target_x, target_y = 320, 320
            while True:
                if camera.exit_flag:
                    break
                frame = camera.get_frame()
                if frame is None:
                    continue
                # 转换为BGR格式（OpenCV）
                #frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if mode=="detect":
                    target_x,target_y=detect_digit(frame,detector,target_digit)
                    if target_x and target_y:
                        mode="tracking"
                        tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo,1)
                    else:
                        print("未找到目标")
                if mode=="tracking":
                    #绘制红点位
                    #cx, cy = laser_track(frame)
#                        if cx and cy:
#                            cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)
#                            cv2.putText(frame_bgr, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    if camera.laser_x is None or camera.laser_y is None:
                        servo.set_servo_angle(128,130,0.5)
                        tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo,1)
                    if tracking_digit(camera.laser_x,camera.laser_y,target_x,target_y,servo):
                        ser.message=None
                        target_digit=None
                        flag=0
                        ser.send("Y")
                        break       
def destory():
    ser.stop()
    servo.destroy()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    print("选择任务")
    try:
        while True:
            if camera.exit_flag:
                print("END")
                break
            message=ser.get_message()
            if message:
                if message=="task1":
                    task1()
                elif message=="task2":
                    task2()
    finally:
        print("程序关闭")
        destory()