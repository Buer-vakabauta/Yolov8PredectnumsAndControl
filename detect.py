#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import time
import math
from yolodect import YOLODetector
from camera import CameraCapture
from Servo import Servo2
from LaserTracker import laser_track
#from serial_task import ser

def get_max_digit(detector, frame):
    '''
    获取最大置信度的数字
    Args:
        detector:检测器
        frame:检测帧
    Returns:
        返回检测到的数字
        没有检测到则返回None

    '''
    boxes, scores, class_ids = detector.detect(frame)
    best_score = -1
    best_index = None
    if boxes is not None and len(boxes):
        for box, score, class_id in zip(boxes, scores, class_ids):
            if score>best_score:
                best_score=score
                best_index=class_id
    if best_score > 0:
        print(f"找到目标 '{detector.class_names[class_id]}'，置信度: {best_score:.2f}")
        return detector.class_names[best_index]

    return None

    

def detect_digit(frame,detector,target_digit):
    '''
    运行模型检测数字
    Args:
        frame:检查帧
        detector:检测器类
        target_digit:要检测数字
    Returns:
        检测成功返回中心坐标cx,cy
        没检测到返回两个None
    '''
    boxes, scores, class_ids = detector.detect(frame)
    #annotated_frame = detector.draw_detections(frame_bgr.copy(), boxes, scores, class_ids)
    if boxes is not None and len(boxes):
        for box, score, class_id in zip(boxes, scores, class_ids):
            class_name = detector.class_names[class_id]
            if class_name == target_digit:
                cx = int((box[0] + box[2]) / 2)
                cy = int((box[1] + box[3]) / 2)
                print(f"找到目标 '{target_digit}'，中心坐标: ({cx}, {cy})")
                return cx,cy
    return None,None

def tracking_digit(cx,cy,target_x,target_y,servo,tracking=2):
    '''
    追踪目标位置
    
    Args:
        cx,cy:当前位置的x,y坐标
        target_x,target_y:目标位置的x坐标
        servo:舵机类
        tracking:1表示大调,2表示微调
    Return:
        到达目标附近返回Ture
        否则返回False
    '''
    if cx is None or cy is None or target_x is None or target_y is None:
        print("data error",cx,cy,target_x,target_y)
        return False
    if abs(cx-target_x)<15 and abs(cy-target_y)<15:
        #servo.pwm_h.ChangeDutyCycle(0)
        #servo.pwm_v.ChangeDutyCycle(0)
        print("追踪完成")
        return True
    else:
        if tracking==1:
            #dis=(servo.disx+servo.disy)/2
            a=math.atan((target_x-servo.inital_x)/(servo.disx))*180/math.pi
            b=math.atan((target_y-servo.inital_y)/(servo.disy))*180/math.pi
            #print("水平/垂直:",a,b)
            servo.set_servo_angle(servo.inital_angle_x-a,servo.inital_angle_y+b,0.1)
        elif tracking==2:
            servo.move2pos(cx, cy, target_x, target_y)
        return False





def model_test():
    """用于测试模型"""
    # 配置参数
    MODEL_PATH = "best2.onnx"  # 模型路径
    CAMERA_WIDTH = 640  # 增加分辨率
    CAMERA_HEIGHT = 640
    CAMERA_FPS = 30
    
    try:
        # 初始化检测器
        print("初始化YOLO检测器...")
        detector = YOLODetector(MODEL_PATH)
        print("检测器初始化完成")
        
        # 初始化摄像头
        print("初始化摄像头...")
        camera = CameraCapture(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
        camera.start()
        print("摄像头初始化完成")
        
        # 性能统计
        fps_counter = 0
        fps_start_time = time.time()
        
        print("开始实时检测十六进制数字... 按 'q' 退出")
        
        # 创建窗口并设置可调整大小
        #cv2.namedWindow("num_recongise", cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("num_recongise", 640, 640)
        
        while True:
            # 获取帧
            frame = camera.get_frame()
            if frame is None:
                continue
            
            # 确保图像格式正确 (RGB -> BGR for OpenCV)
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                # PiCamera2输出RGB格式，OpenCV需要BGR格式
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                frame_bgr = frame
            frame_bgr=frame
            
            # 记录开始时间
            start_time = time.time()
            
            # 执行检测 (使用原始RGB格式进行检测)
            boxes, scores, class_ids = detector.detect(frame)
            
            # 绘制检测结果 (在BGR格式图像上绘制)
            annotated_frame = detector.draw_detections(frame_bgr, boxes, scores, class_ids)
            
            # 计算处理时间
            processing_time = time.time() - start_time
            
            # 添加性能信息
            fps_counter += 1
            if time.time() - fps_start_time >= 1.0:
                current_fps = fps_counter / (time.time() - fps_start_time)
                fps_counter = 0
                fps_start_time = time.time()
            else:
                current_fps = 0
            
            # 在图像上显示性能信息
            info_text = f"FPS: {current_fps:.1f} | process time: {processing_time*1000:.1f}ms | num: {len(boxes)}"
            cv2.putText(
                annotated_frame,
                info_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            
            # 显示结果
            cv2.imshow("num_recongise", annotated_frame)
            
            # 检查退出条件
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                
            # 打印检测结果
            
            if boxes is not None and len(boxes) > 0:
                print(f"检测到 {len(boxes)} 个十六进制数字:")
                for i, (box, score, class_id) in enumerate(zip(boxes, scores, class_ids)):
                    hex_digit = detector.class_names[class_id] if class_id < len(detector.class_names) else f"Unknown_{class_id}"
                    print(f"  {i+1}. 数字 '{hex_digit}': {score:.3f} - 位置: ({box[0]:.1f}, {box[1]:.1f}, {box[2]:.1f}, {box[3]:.1f})")
            
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 清理资源
        if 'camera' in locals():
            camera.stop()
        cv2.destroyAllWindows()
        print("程序已退出")




if __name__ == "__main__":
    model_test()


#数字选择测试
'''
def select_target_digit(class_names):
    """选择目标数字（十六进制字符）
        class_names:目标数组    
    """
    while True:
        sel = input("请输入要追踪的目标数字").strip().upper()
        #sel = data.strip().upper()
        if sel in class_names:
            return sel
        print("输入无效，请重新输入。")

def task1_test():
    MODEL_PATH = "best1.onnx"
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 640
    CAMERA_FPS = 30

    detector = YOLODetector(MODEL_PATH)
    servo = Servo()
    camera = CameraCapture(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
    camera.start()

    target_digit = select_target_digit(detector.class_names)
    print(f"开始识别数字 '{target_digit}' ...")

    mode = "detect"  # 可为 'detect' 或 'track'
    target_x, target_y = 320, 320
    tracking = False

    cv2.namedWindow("num_track", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("num_track", 800, 600)

    while True:
        frame = camera.get_frame()
        if frame is None:
            continue

        # 转换为BGR格式（OpenCV）
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if mode == "detect":
            boxes, scores, class_ids = detector.detect(frame)
            #annotated_frame = detector.draw_detections(frame_bgr.copy(), boxes, scores, class_ids)
            found = False
            if boxes is not None and len(boxes):
                for box, score, class_id in zip(boxes, scores, class_ids):
                    class_name = detector.class_names[class_id]
                    if class_name == target_digit:
                        cx = int((box[0] + box[2]) / 2)
                        cy = int((box[1] + box[3]) / 2)
                        target_x, target_y = cx, cy
                        print(f"找到目标 '{target_digit}'，中心坐标: ({cx}, {cy})")
                        mode = "track"
                        tracking = True
                        break  # 找到第一个目标后退出检测

        elif mode == "track":
            cx, cy = laser_track(frame)
            if cx and cy:
                cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame_bgr, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                if tracking:
                    if abs(cx - target_x) < 10 and abs(cy - target_y) < 10:
                        tracking = False
                        mode = "detect"
                        print("追踪完成，重新进入识别阶段")
                        target_digit = select_target_digit(detector.class_names)
                        print(f"识别新目标 '{target_digit}' ...")
                    else:
                        servo.move2pos(cx, cy, target_x, target_y)

            # 显示目标中心点
            cv2.circle(frame_bgr, (target_x, target_y), 5, (0, 255, 0), -1)

        # 显示图像
        cv2.imshow("num_track", frame_bgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # 清理
    camera.stop()
    cv2.destroyAllWindows()
    servo.destory()
    print("程序退出")
'''