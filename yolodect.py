import numpy as np
import onnxruntime as ort
import cv2

class YOLODetector:
    def __init__(self, model_path, conf_threshold=0.25, nms_threshold=0.9):
        """
        初始化YOLO检测器
        
        Args:
            model_path: ONNX模型路径
            conf_threshold: 置信度阈值
            nms_threshold: NMS阈值
        """
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # 初始化ONNX运行时
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        # 模型输入尺寸
        self.input_width = 640
        self.input_height = 640
        
        # 十六进制数字类别名称 (0~F)
        self.class_names = [
            '0', '1', '2', '3', '4', '5', '6', '7', 
            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
        ]
        
        # 生成随机颜色
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.class_names), 3), dtype=np.uint8)

    def preprocess(self, image):
        """
        预处理图像
        
        Args:
            image: 输入图像
            
        Returns:
            preprocessed_image: 预处理后的图像
            scale: 缩放比例
            pad: 填充值
        """
        # 获取原始图像尺寸
        h, w = image.shape[:2]
        
        # 计算缩放比例
        scale = min(self.input_width / w, self.input_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        # 缩放图像
        resized = cv2.resize(image, (new_w, new_h))
        
        # 创建填充图像
        padded = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        
        # 计算填充位置
        pad_x = (self.input_width - new_w) // 2
        pad_y = (self.input_height - new_h) // 2
        
        # 将缩放后的图像放置到填充图像中心
        padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        
        # 转换为模型输入格式
        input_image = padded.astype(np.float32) / 255.0
        input_image = np.transpose(input_image, (2, 0, 1))  # HWC -> CHW
        input_image = np.expand_dims(input_image, axis=0)   # 增加batch维度
        
        return input_image, scale, (pad_x, pad_y)

    def postprocess(self, outputs, scale, pad):
        """
        后处理检测结果
        
        Args:
            outputs: 模型输出
            scale: 缩放比例
            pad: 填充值
            
        Returns:
            boxes: 检测框
            scores: 置信度
            class_ids: 类别ID
        """
        # 获取输出数据 shape: (1, 20, 8400)
        predictions = outputs[0][0]  # shape: (20, 8400)
        
        # 转置为 (8400, 20)
        predictions = predictions.T
        
        # 分离坐标和类别置信度
        boxes = predictions[:, :4]  # 前4列是坐标 (cx, cy, w, h)
        #obj_conf=predictions[:,4]
        #cls_conf=predictions[:,5:]
        #scores = obj_conf[:, None] * cls_conf  # (8400, 16)
        scores = predictions[:, 4:]  # 后16列是类别置信度 (对应16个十六进制数字类别)
        
        # 获取最大置信度和对应的类别
        class_scores = np.max(scores, axis=1)
        class_ids = np.argmax(scores, axis=1)
        
        # 过滤低置信度检测
        valid_indices = class_scores >= self.conf_threshold
        valid_boxes = boxes[valid_indices]
        valid_scores = class_scores[valid_indices]
        valid_class_ids = class_ids[valid_indices]
        
        if len(valid_boxes) == 0:
            return [], [], []
        
        # 转换坐标格式 (cx, cy, w, h) -> (x1, y1, x2, y2)
        x1 = valid_boxes[:, 0] - valid_boxes[:, 2] / 2
        y1 = valid_boxes[:, 1] - valid_boxes[:, 3] / 2
        x2 = valid_boxes[:, 0] + valid_boxes[:, 2] / 2
        y2 = valid_boxes[:, 1] + valid_boxes[:, 3] / 2
        
        # 调整坐标到原始图像尺寸
        pad_x, pad_y = pad
        x1 = (x1 - pad_x) / scale
        y1 = (y1 - pad_y) / scale
        x2 = (x2 - pad_x) / scale
        y2 = (y2 - pad_y) / scale
        
        # 组合坐标
        adjusted_boxes = np.column_stack([x1, y1, x2, y2])
        
        # 应用NMS
        indices = cv2.dnn.NMSBoxes(
            adjusted_boxes.tolist(),
            valid_scores.tolist(),
            self.conf_threshold,
            self.nms_threshold
        )
        
        if len(indices) > 0:
            indices = indices.flatten()
            return adjusted_boxes[indices], valid_scores[indices], valid_class_ids[indices]
        else:
            return [], [], []

    def detect(self, image):
        """
        执行目标检测
        
        Args:
            image: 输入图像
            
        Returns:
            boxes: 检测框
            scores: 置信度
            class_ids: 类别ID
        """
        # 预处理
        input_image, scale, pad = self.preprocess(image)
        
        # 推理
        outputs = self.session.run([self.output_name], {self.input_name: input_image})
        #print("outputs:",outputs[0].shape)
        # 后处理
        boxes, scores, class_ids = self.postprocess(outputs, scale, pad)
        
        return boxes, scores, class_ids

    def draw_detections(self, image, boxes, scores, class_ids):
        """
        在图像上绘制检测结果
        
        Args:
            image: 输入图像 (BGR格式)
            boxes: 检测框
            scores: 置信度
            class_ids: 类别ID
            
        Returns:
            annotated_image: 标注后的图像
        """
        annotated_image = image.copy()
        
        # 确保图像不为空
        if annotated_image is None or annotated_image.size == 0:
            print("警告: 输入图像为空")
            return annotated_image
        
        for box, score, class_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box.astype(int)
            
            # 确保坐标在图像范围内
            h, w = annotated_image.shape[:2]
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(0, min(x2, w-1))
            y2 = max(0, min(y2, h-1))
            
            # 获取类别名称和颜色
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"Class {class_id}"
            color = tuple(map(int, self.colors[class_id % len(self.colors)]))
            
            # 绘制边界框
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 3)
            
            # 绘制标签
            label = f"{class_name}: {score:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            
            # 绘制标签背景
            cv2.rectangle(
                annotated_image,
                (x1, y1 - label_size[1] - 15),
                (x1 + label_size[0] + 10, y1),
                color,
                -1
            )
            
            # 绘制标签文字
            cv2.putText(
                annotated_image,
                label,
                (x1 + 5, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )
        
        return annotated_image