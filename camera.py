
from picamera2 import Picamera2
import threading
import queue
import time
from LaserTracker import laser_track
import cv2
class CameraCapture:
    def __init__(self, width=640, height=640, fps=30):
        """
        初始化摄像头捕获
        
        Args:
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False
        self.laser_x=-1
        self.laser_y=-1
        self.exit_flag=False
        # 初始化PiCamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(config)

    def start(self):
        """启动摄像头捕获线程"""
        self.running = True
        self.picam2.start()
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.start()

    def stop(self):
        """停止摄像头捕获"""
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        self.picam2.stop()

    def _capture_frames(self):
        """捕获帧的线程函数"""
        while self.running:
            try:
                frame = self.picam2.capture_array()
                # 检查帧是否有效
                if frame is not None and frame.size > 0:
                    # 如果队列满了，丢弃旧帧
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    
                    self.frame_queue.put(frame, block=False)
                    self.laser_x,self.laser_y=laser_track(frame)
                    if self.laser_x and self.laser_y:
                        #cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"({self.laser_x},{self.laser_y})", (self.laser_x+10, self.laser_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
#                        print(self.laser_x,self.laser_y)
                #time.sleep(1.0 / self.fps)
                cv2.imshow("num_track", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.exit_flag=True
                    self.stop()
                    break
                
            except Exception as e:
                print(f"摄像头捕获错误: {e}")
                time.sleep(0.1)  # 错误时稍等再试

    def get_frame(self):
        """获取最新帧"""
        try:
            return self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return None
