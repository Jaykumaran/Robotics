#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
# To access and locate shared resource (configs, model, launch files) from the current package 
from ament_index_python.packages import get_package_share_directory
import time
# Custom message types for detection results which are defined under msg/
from sim_cam_pkg.msg import Detection, DetectionArray

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.get_logger().info("--- INITIALIZING YOLOv11n OBJECT DETECTOR NODE ---")

        self.prev_time = time.time()
        self.fps = 0.0
        
        # ==========================================================
        # | Model Name                    | Size (Params)  |
        # |-------------------------------|----------------|
        # | SSD MobileNetV3 Small         | 2.9M           |
        # | SSDLite320 MobileNetV3 Large  | 3.4M           |
        # | YOLO11n                       | 2.6M           | ✅
        # | YOLOv8n                       | 3.2M           |
        # | YOLOv5nu                      | 2.6M           |
        # ==========================================================


        # Get the configurable parameter via launch file or cli
        self.declare_parameter('confidence_threshold', 0.45) # Initialize with default confidence for detections
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.get_logger().info(f"Using configurable confidence_threshold: {self.confidence_threshold}")

        # Other parameters
        onnx_model_name = 'yolo11n.onnx'
        labels_name = 'coco.names'
        self.nms_threshold = 0.5
        self.input_width = 640
        self.input_height = 640
        
        # Show Detection results in a OpenCV Window
        self.cv_window_name = "YOLO11n Live Detections (ROS Node)"
        try:
            cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)
            self.get_logger().info(f"Successfully created OpenCV window: {self.cv_window_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to create OpenCV window: {e} - Display will not work.")
            self.cv_window_name = None

        # Model and label.txt path
        pkg_share_dir = get_package_share_directory('sim_cam_pkg')
        onnx_model_path = os.path.join(pkg_share_dir, 'models', onnx_model_name)
        labels_path = os.path.join(pkg_share_dir, 'models', labels_name)

        # Read class labels from coco.names txt file
        self.class_labels = []
        try:
            with open(labels_path, 'rt') as fpt:
                self.class_labels = [line.strip() for line in fpt.readlines()]
            self.get_logger().info(f"Loaded {len(self.class_labels)} class labels from {labels_path}.")
            if len(self.class_labels) != 80 and len(self.class_labels) != 0 : # Basic check for COCO
                 self.get_logger().warn(f"Expected 80 COCO classes, but found {len(self.class_labels)}. Ensure labels file is correct for YOLO11.")
        except Exception as e:
            self.get_logger().error(f"Failed to load labels from {labels_path}: {e}")
            rclpy.shutdown(); return
            
        # ==================================================================
         # Load the ONNX Model using OpenCV DNN Inference (CPU)
        self.get_logger().info(f"Loading ONNX model from: {onnx_model_path}")
       
        try:
            self.net = cv2.dnn.readNetFromONNX(onnx_model_path)

            self.get_logger().info("YOLO11n ONNX model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Error loading ONNX model: {e}")
            rclpy.shutdown(); return
        # ==================================================================

        # ROS2 Communicaton Setup
        self.bridge = CvBridge() # Tool for ROS Image <->OpenCV Compatible Conversion
        # Subscribe to processed images from the preprocessor node of topic /image_processed
        self.image_subscription = self.create_subscription(
            Image,
            '/image_processed', 
            self.image_callback, 
            10) # QoS profile depth or buffer size: how many msgs to keep in queue, i.e. stores 10 unprocessed messages for realtime stream
        
        # Publish the detection results over /object_detection topic for tracking
        self.detection_publisher = self.create_publisher(
            DetectionArray,
            '/object_detection', 
            10) 
        
        self.get_logger().info("Node setup complete. Waiting for images on /image_processed...")

    def image_callback(self, msg: Image):
        """
        Callback function for processing incoming image messages.
        Performs object detection and publishes det results as msg
        """
        
        start_time = time.time()
        
        try:
            # Convert ROS Image message to OpenCV BGR uint8 array format
            cv_image_original = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error converting incoming image: {e}")
            return

        if cv_image_original is None: return

        display_image = cv_image_original.copy() # Create a copy drawing visualizations
        original_height, original_width = cv_image_original.shape[:2]

        # ======================= Object Detection (Forward Pass)=====================================================
        # Resize to (640, 640) and normalize [0, 1] to be YOLO11n input compatible 
        blob = cv2.dnn.blobFromImage(cv_image_original, scalefactor=1/255.0,
                                     size=(self.input_width, self.input_height),
                                     mean=(0,0,0), swapRB=True, crop=False)
        
        self.net.setInput(blob) # Setup input and peform forward pass
        try:
            # Ultralytics YOLO ONNX models, net.forward() gives the [1, 84, 8400] output directly
            raw_dnn_output = self.net.forward() 
        except Exception as e:
            self.get_logger().error(f"DNN forward pass failed: {e}")
            return
       

        # --- -----------------------     YOLO Postprocessing    -------------------------------------------------------------
        # YOLO Detection output format: [batch_size, num_classes + 4 box coords, num_proposals]; 80 COCO classes
        # The total number of proposals of YOLO here is from multiple heads operating at different fixed grid sizes like 80x80, 40x40, 20x20 
        # 80*80 + 40*40 + 20*20 = 8400
        # raw_dnn_output shape is (1, 84, num_proposals) e.g. (1, 84, 8400) for 640x640 input
        if raw_dnn_output.shape[0] == 1 and len(raw_dnn_output.shape) == 3 : # (1, 84, 8400)
            predictions = np.squeeze(raw_dnn_output).T # Transpose to (num_proposals, 84) --> (8400, 84)
        else:
            self.get_logger().error(f"Unexpected DNN output shape: {raw_dnn_output.shape}")
            return

        # To store NMS filtered boxes
        boxes = []
        confidences = [] # Store class confidences here
        class_ids_list = []

        # Calculate scaling factors to map detections from normalized input size back to original image size
        x_factor = original_width / self.input_width
        y_factor = original_height / self.input_height

        for i in range(predictions.shape[0]): # Iterate through each of the 8400 proposals
            
            row_data = predictions[i] # single proposal accessed by indexS
            # First 4 elements are cx, cy, w, h (center_x, center_y, width, height)
            # Relative to self.input_width, self.input_height
            box_coords_norm = row_data[:4]
            # Next 80 elements are class prob.scores for all coco classes
            class_scores = row_data[4:]
            
            # Class ID that has max prob score for this proposal
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id] # This is P(class|object)


            if confidence > self.confidence_threshold: # Filter based on class confidence
                cx, cy, w, h = box_coords_norm
                
                # Scale box coordinates to original image size
                # (cx, cy, w, h) are for the network input (e.g., 640x640)
                orig_cx = cx * x_factor
                orig_cy = cy * y_factor
                orig_w = w * x_factor
                orig_h = h * y_factor

                # Convert from YOLO ann. format center_x, center_y, width, height to COCO ann. format: (top_left_x, top_left_y, width, height)
                x1 = int(orig_cx - orig_w / 2)
                y1 = int(orig_cy - orig_h / 2)
                
                # All of filtered boxes and their associated prob score, and class ID are stored in lists.
                boxes.append([x1, y1, int(orig_w), int(orig_h)]) # For NMSBoxes: x, y, width, height
                confidences.append(float(confidence))
                class_ids_list.append(int(class_id))

        # Apply Non-Maximum Suppression to filter Overalapping boxes
        indices = []
        if len(boxes) > 0:
            # NMSBoxes Returns: indices of boxes to keep
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
            if isinstance(indices, np.ndarray): # NMSBoxes will return tuple if empty
                indices = indices.flatten() # Convert from 2D array to 1D list of nms filtered indices
        
        # =====================================================================================================
        
        # ========================= Prepare ROS Message  ================================
        detected_objects_list_for_ros = [] # List to hold Detection msgs
        for i in indices:
            box = boxes[i]
            x, y, w, h = box[0], box[1], box[2], box[3]
            class_id = class_ids_list[i]
            score = confidences[i]
            class_name = self.class_labels[class_id] if 0 <= class_id < len(self.class_labels) else "unknown"

            # Draw on display_image (all NMS-filtered boxes)
            if self.cv_window_name:
                color = (0,255,0) # All boxes shown are "confirmed" by NMS
                cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
                label_text = f"{class_name}: {score:.2f}"
                text_y_pos = y - 10 if y > 20 else y + h + 20
                cv2.putText(display_image, label_text, (x, text_y_pos), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Create Detection Messsage and Add to ROS message list
            det = Detection()
            det.class_id = class_id
            det.class_name = class_name
            det.score = score
            det.box_x = x
            det.box_y = y
            det.box_width = w
            det.box_height = h
            det.og_image_height = self.input_width
            det.og_image_width = self.input_height
            detected_objects_list_for_ros.append(det)  
        
        # ======================================================================================
        
        # ---------------- Visualizations and FPS Calculation -------------------
        end_time = time.time()
        elapsed = end_time - start_time
        self.fps = 1.0 / elapsed if elapsed > 0 else 0.0
        
        if self.cv_window_name:
            fps_text = f"FPS: {self.fps:.2f}"
            cv2.putText(display_image, fps_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            try:
                cv2.imshow(self.cv_window_name, display_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error during imshow/waitKey: {e}")
                cv2.destroyWindow(self.cv_window_name)
                self.cv_window_name = None
        # -------------------------------------------------------------------------
        
        
        # Publish Detections
        detection_array_msg = DetectionArray(header=msg.header, detections=detected_objects_list_for_ros)
        self.detection_publisher.publish(detection_array_msg)
        if detected_objects_list_for_ros:
            self.get_logger().info(f"Published {len(detected_objects_list_for_ros)} YOLO11 detections.")

    # Gracefully clean up resources when a node is shutting down or by Ctrl+C
    def destroy_node(self):
        self.get_logger().info("Cleaning up YOLO11 OpenCV window...")
        if self.cv_window_name:
            cv2.destroyWindow(self.cv_window_name)
        super().destroy_node()
        self.get_logger().info("Node destroyed.")

def main(args=None):
    # Initialize ROS2 CLI library
    rclpy.init(args=args) # Must be called before any ROS functionality can be used
    # Create an instance of the node
    node = ObjectDetectorNode()  
    try:
        rclpy.spin(node) # To keep node responsive, alive and to process callbacks
    except KeyboardInterrupt: 
        node.get_logger().info("Keyboard interrupt received.")
    
    # When either one of exception ends, always execute
    finally: # To gracefully shutdown even abrupt interrupts when Ctrl+C is pressed
        if node and rclpy.ok() and node.context.ok():
             node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown() # Shitdown ROS2 on exit
    print("ROS Shutdown complete for Object Detection node.")

if __name__ == '__main__':
    main()  # Entry point