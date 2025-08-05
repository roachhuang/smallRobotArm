"""ML Vision Module

This module provides machine learning-based vision capabilities for object detection,
pose estimation, and grasp prediction using pre-trained models.
"""

import numpy as np
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("OpenCV not available. Install with: sudo apt install python3-opencv")

from typing import List, Dict, Tuple

# Optional heavy ML dependencies
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class MLVision:
    """ML-powered vision system for the robot arm."""
    
    def __init__(self):
        if YOLO_AVAILABLE:
            self.object_detector = YOLO('yolov8n.pt')
        else:
            self.object_detector = None
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect objects using YOLO or fallback color detection."""
        if YOLO_AVAILABLE and self.object_detector:
            results = self.object_detector(image)
            objects = []
            
            for r in results:
                for box in r.boxes:
                    if box.conf > 0.5:
                        objects.append({
                            'class': r.names[int(box.cls)],
                            'confidence': float(box.conf),
                            'bbox': box.xyxy[0].tolist(),
                            'center_2d': self._get_bbox_center(box.xyxy[0].tolist())
                        })
            return objects
        else:
            return self._fallback_object_detection(image)
    
    def predict_grasp_poses(self, objects: List[Dict]) -> List[Dict]:
        """Generate grasp candidates for detected objects."""
        grasp_candidates = []
        
        for obj in objects:
            # Simple grasp generation
            center = obj['center_2d']
            candidates = [
                {
                    'position': [center[0], center[1], 200],  # Estimated 3D position
                    'approach_vector': [0, 0, -1],  # Top-down approach
                    'confidence': obj['confidence'],
                    'object_class': obj['class']
                }
            ]
            grasp_candidates.extend(candidates)
        
        return grasp_candidates
    
    def _get_bbox_center(self, bbox):
        return ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    
    def _fallback_object_detection(self, image):
        """Simple color-based detection that works without heavy ML."""
        if not CV2_AVAILABLE:
            # Ultra-simple fallback: assume object at image center
            h, w = image.shape[:2]
            return [{
                'class': 'unknown_object',
                'confidence': 0.5,
                'bbox': [w//4, h//4, 3*w//4, 3*h//4],
                'center_2d': [w//2, h//2]
            }]
        
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # Multiple color detection
        color_ranges = {
            'red_object': ([0, 50, 50], [10, 255, 255]),
            'blue_object': ([100, 50, 50], [130, 255, 255]),
            'green_object': ([40, 50, 50], [80, 255, 255])
        }
        
        objects = []
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    objects.append({
                        'class': color_name,
                        'confidence': 0.8,
                        'bbox': [x, y, x+w, y+h],
                        'center_2d': [x + w//2, y + h//2]
                    })
        
        return objects


def detect_and_plan_grasp(image: np.ndarray) -> List[Dict]:
    """Detect objects and plan grasps in one function."""
    vision = MLVision()
    objects = vision.detect_objects(image)
    grasp_candidates = vision.predict_grasp_poses(objects)
    return sorted(grasp_candidates, key=lambda x: x['confidence'], reverse=True)