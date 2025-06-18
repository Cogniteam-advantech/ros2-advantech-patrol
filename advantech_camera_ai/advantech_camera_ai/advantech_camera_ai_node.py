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
import gc
import psutil
import torch
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
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('width_threshold', 500)  # pixels
        self.declare_parameter('model_path', default_model_path)  # YOLO model path
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_cuda', True)  # New parameter for CUDA usage
        self.declare_parameter('force_cpu', False)  # Force CPU usage if needed
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.width_threshold = self.get_parameter('width_threshold').get_parameter_value().integer_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.use_cuda = self.get_parameter('use_cuda').get_parameter_value().bool_value
        self.force_cpu = self.get_parameter('force_cpu').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'detection_image', 10)
        self.person_detected_pub = self.create_publisher(Bool, 'person_width_exceeded', 10)
        
        # Check and setup CUDA
        self.device = self.setup_device()
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        try:
            self.model = YOLO(self.model_path)
            # Move model to appropriate device
            if hasattr(self.model.model, 'to'):
                self.model.model.to(self.device)
            self.get_logger().info(f'YOLO model loaded successfully on device: {self.device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            return
        
        # Initialize video capture with better error handling
        self.get_logger().info(f'Opening camera {self.camera_index}')
        self.cap = None
        self.initialize_camera()
        
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.camera_index}')
            return
        
        # Timer for processing at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        # Thread safety and memory management
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        self.memory_check_counter = 0
        
        # Start frame capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info(f'Node initialized - Rate: {self.publish_rate} Hz, Width threshold: {self.width_threshold}px, Device: {self.device}')
    
    def setup_device(self):
        """Setup and validate CUDA device"""
        if self.force_cpu:
            self.get_logger().info("Forced CPU usage enabled")
            return 'cpu'
        
        if not self.use_cuda:
            self.get_logger().info("CUDA usage disabled by parameter")
            return 'cpu'
        
        try:
            # Check if CUDA is available
            cuda_available = torch.cuda.is_available()
            self.get_logger().info(f"PyTorch CUDA available: {cuda_available}")
            
            if cuda_available:
                # Test CUDA functionality safely
                try:
                    test_tensor = torch.tensor([1.0]).cuda()
                    result = test_tensor + 1
                    del test_tensor, result
                    torch.cuda.empty_cache()
                    
                    gpu_name = torch.cuda.get_device_name(0)
                    self.get_logger().info(f"Using GPU: {gpu_name}")
                    return 'cuda'
                    
                except Exception as e:
                    self.get_logger().error(f"CUDA test failed: {e}")
                    self.get_logger().info("Falling back to CPU")
                    return 'cpu'
            else:
                self.get_logger().info("CUDA not available, using CPU")
                return 'cpu'
                
        except Exception as e:
            self.get_logger().error(f"Error checking CUDA: {e}")
            return 'cpu'
    
    def initialize_camera(self):
        """Initialize camera with robust error handling"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera {self.camera_index}')
                return
            
            # Set camera properties with error checking
            properties = [
                (cv2.CAP_PROP_FRAME_WIDTH, 640),
                (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                (cv2.CAP_PROP_FPS, 30),
                (cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to prevent memory buildup
            ]
            
            for prop, value in properties:
                if not self.cap.set(prop, value):
                    self.get_logger().warn(f'Failed to set camera property {prop} to {value}')
            
            # Verify camera is working
            ret, test_frame = self.cap.read()
            if ret:
                self.get_logger().info(f'Camera initialized successfully - Frame shape: {test_frame.shape}')
                del test_frame  # Clean up test frame
            else:
                self.get_logger().error('Camera opened but cannot read frames')
                self.cap.release()
                self.cap = None
                
        except Exception as e:
            self.get_logger().error(f'Error initializing camera: {e}')
            if self.cap:
                self.cap.release()
            self.cap = None
    
    def capture_frames(self):
        """Continuously capture frames from camera in separate thread with memory management"""
        frame_count = 0
        
        while self.running and rclpy.ok():
            try:
                if self.cap is None or not self.cap.isOpened():
                    self.get_logger().warn('Camera not available, retrying...')
                    self.initialize_camera()
                    if self.cap is None:
                        rclpy.sleep_for(1.0)  # Wait before retry
                        continue
                
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        if self.latest_frame is not None:
                            del self.latest_frame
                        self.latest_frame = frame.copy()
                    
                    # Memory cleanup every 100 frames
                    frame_count += 1
                    if frame_count % 100 == 0:
                        gc.collect()
                        if self.device == 'cuda':
                            torch.cuda.empty_cache()
                else:
                    self.get_logger().warn('Failed to read frame from camera')
                    rclpy.sleep_for(0.1)
                    
            except Exception as e:
                self.get_logger().error(f'Error in capture thread: {e}')
                rclpy.sleep_for(0.1)
    
    def detect_persons(self, frame):
        """
        Detect persons in the frame using YOLO with CUDA support
        Returns: list of bounding boxes for persons
        """
        try:
            # Run YOLO inference with device specification
            results = self.model(frame, conf=self.confidence_threshold, verbose=False, device=self.device)
            
            person_boxes = []
            
            # Process results
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Class 0 is 'person' in COCO dataset
                        if int(box.cls[0]) == 0:  # person class
                            # Get bounding box coordinates (ensure CPU tensor)
                            if self.device == 'cuda':
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                confidence = box.conf[0].cpu().numpy()
                            else:
                                x1, y1, x2, y2 = box.xyxy[0].numpy()
                                confidence = box.conf[0].numpy()
                            
                            person_boxes.append({
                                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                'confidence': float(confidence)
                            })
            
            return person_boxes
            
        except Exception as e:
            self.get_logger().error(f'Error in person detection: {str(e)}')
            # Clear CUDA cache on error
            if self.device == 'cuda':
                torch.cuda.empty_cache()
            return []
    
    def check_width_threshold(self, person_boxes):
        """
        Check if any person bounding box width exceeds threshold
        Returns: True if any person width > threshold, False otherwise
        """
        for person in person_boxes:
            x1, y1, x2, y2 = person['bbox']
            width = x2 - x1
            
            self.get_logger().debug(f'Person width: {width}px')
            
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
        
        # Draw threshold and device info
        info_text = f'Threshold: {self.width_threshold}px | Rate: {self.publish_rate}Hz | Device: {self.device}'
        cv2.putText(annotated_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return annotated_frame
    
    def monitor_memory(self):
        """Monitor memory usage periodically"""
        self.memory_check_counter += 1
        if self.memory_check_counter % 100 == 0:
            try:
                process = psutil.Process(os.getpid())
                memory_mb = process.memory_info().rss / 1024 / 1024
                self.get_logger().debug(f'Memory usage: {memory_mb:.1f} MB')
                
                if self.device == 'cuda':
                    memory_allocated = torch.cuda.memory_allocated() / 1024**2
                    memory_reserved = torch.cuda.memory_reserved() / 1024**2
                    self.get_logger().debug(f'GPU Memory - Allocated: {memory_allocated:.1f} MB, Reserved: {memory_reserved:.1f} MB')
                    
            except Exception as e:
                self.get_logger().warn(f'Memory monitoring error: {e}')
    
    def process_frame(self):
        """Main processing function called by timer with memory management"""
        try:
            with self.frame_lock:
                if self.latest_frame is None:
                    return
                frame = self.latest_frame.copy()
            
            # Monitor memory usage
            self.monitor_memory()
            
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
            
            # Clean up frame references
            del frame, annotated_frame
            
            # Periodic garbage collection
            if self.memory_check_counter % 50 == 0:
                gc.collect()
                if self.device == 'cuda':
                    torch.cuda.empty_cache()
                    
        except Exception as e:
            self.get_logger().error(f'Error in process_frame: {str(e)}')
            # Emergency cleanup
            if self.device == 'cuda':
                torch.cuda.empty_cache()
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down node...')
        
        # Stop capture thread
        self.running = False
        
        # Wait for capture thread to finish
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        # Release camera
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')
        
        # Clear CUDA cache
        if self.device == 'cuda':
            torch.cuda.empty_cache()
            self.get_logger().info('CUDA cache cleared')
        
        # Final garbage collection
        gc.collect()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = YoloPersonDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()