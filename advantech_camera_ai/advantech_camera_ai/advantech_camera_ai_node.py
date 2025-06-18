#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import os
from ament_index_python.packages import get_package_share_directory


class YoloPersonDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_person_detection_node')
        
        # Get default model path from package installation
        try:
            package_share_directory = get_package_share_directory('advantech_camera_ai')
            default_model_path = os.path.join(package_share_directory, 'model', 'yolov8n.pt')
        except Exception:
            default_model_path = 'yolov8n.pt'  # Fallback to download
        
        # Declare parameters
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('width_threshold', 500)  # pixels
        self.declare_parameter('model_path', default_model_path)  # YOLO model path
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.width_threshold = self.get_parameter('width_threshold').get_parameter_value().integer_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'detection_image', 10)
        self.person_detected_pub = self.create_publisher(Bool, 'person_width_exceeded', 10)
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            return
        
        # Initialize video capture
        self.get_logger().info(f'Opening camera {self.camera_index}')
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_index}')
            return
        
        # Set camera properties (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Timer for processing at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        # Thread safety
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        
        # Start frame capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info(f'Node initialized - Rate: {self.publish_rate} Hz, Width threshold: {self.width_threshold}px')
    
    def capture_frames(self):
        """Continuously capture frames from camera in separate thread"""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame.copy()
            else:
                self.get_logger().warn('Failed to read frame from camera')
    
    def detect_persons(self, frame):
        """
        Detect persons in the frame using YOLO
        Returns: list of bounding boxes for persons
        """
        try:
            # Run YOLO inference
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
            
            person_boxes = []
            
            # Process results
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Class 0 is 'person' in COCO dataset
                        if int(box.cls[0]) == 0:  # person class
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            confidence = box.conf[0].cpu().numpy()
                            
                            person_boxes.append({
                                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                'confidence': float(confidence)
                            })
            
            return person_boxes
            
        except Exception as e:
            self.get_logger().error(f'Error in person detection: {str(e)}')
            return []
    
    def check_width_threshold(self, person_boxes):
        """
        Check if any person bounding box width exceeds threshold
        Returns: True if any person width > threshold, False otherwise
        """
        for person in person_boxes:
            x1, y1, x2, y2 = person['bbox']
            width = x2 - x1
            
            print(f'the widht is {width}')
            
            if width > self.width_threshold:
                self.get_logger().debug(f'Person width {width}px exceeds threshold {self.width_threshold}px')
                return True
        
        return False
    
    def draw_detections(self, frame, person_boxes):
        """
        Draw bounding boxes and labels on the frame
        Returns: annotated frame
        """
        annotated_frame = frame.copy()
        
        for person in person_boxes:
            x1, y1, x2, y2 = person['bbox']
            confidence = person['confidence']
            width = x2 - x1
            
            # Choose color based on width threshold
            color = (0, 0, 255) if width > self.width_threshold else (0, 255, 0)  # Red if exceeds, Green otherwise
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f'Person {confidence:.2f} W:{width}px'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            cv2.putText(annotated_frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Draw threshold info
        info_text = f'Threshold: {self.width_threshold}px | Rate: {self.publish_rate}Hz'
        cv2.putText(annotated_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return annotated_frame
    
    def process_frame(self):
        """Main processing function called by timer"""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        
        # Detect persons
        person_boxes = self.detect_persons(frame)
        
        # Check width threshold
        width_exceeded = self.check_width_threshold(person_boxes)
        
        # Draw detections on frame
        annotated_frame = self.draw_detections(frame, person_boxes)
        
        # Publish results
        try:
            # Publish annotated image
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(image_msg)
            
            # Publish boolean result
            bool_msg = Bool()
            bool_msg.data = width_exceeded
            self.person_detected_pub.publish(bool_msg)
            
            # Log detection info
            if person_boxes:
                self.get_logger().debug(f'Detected {len(person_boxes)} persons, width exceeded: {width_exceeded}')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing messages: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloPersonDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()