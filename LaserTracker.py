from picamera2 import Picamera2
from time import strftime
import time
import numpy as np
import cv2

def laser_track(frame):
    """
    追踪红外线光点
    没有检测到返回None

    Args:
        frame:帧图像
    Returns:
        cx:中心横坐标
        cy:中心竖坐标
    """
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    _,mask=cv2.threshold(gray,250,255,cv2.THRESH_BINARY)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if abs(cx-521)<=5 and abs(cy-1)<=5:
                return(None,None)
            return (cx,cy)
    return (None,None)

"""
#相机测试脚本

# 初始化相机
picam0 = Picamera2(0)
# 配置相机预览（分辨率可调整）
picam0.configure(picam0.create_preview_configuration(main={"format": 'RGB888', "size": (640, 640)}))
# 启动相机
picam0.start()
while True:
    # 获取当前帧
    frame = picam0.capture_array()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    _,mask=cv2.threshold(gray,250,255,cv2.THRESH_BINARY)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # 画出红点
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow("Laser Tracker", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("👋 程序退出")
        break

# 清理资源
cv2.destroyAllWindows()
picam0.close()
"""
