"""Vision Detection Module - Color and QR Detection"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from pyzbar import pyzbar


class ColorDetector:
    """Detect colored squares on arena walls"""
    
    # HSV color ranges for target colors
    COLOR_RANGES = {
        'red': [
            ((0, 100, 100), (10, 255, 255)),      # Lower red
            ((170, 100, 100), (180, 255, 255))    # Upper red
        ],
        'green': [
            ((40, 50, 50), (80, 255, 255))
        ],
        'blue': [
            ((100, 50, 50), (130, 255, 255))
        ]
    }
    
    def __init__(self, camera_intrinsics: Dict):
        """
        Initialize color detector
        
        Args:
            camera_intrinsics: Camera intrinsic parameters (fx, fy, cx, cy)
        """
        self.camera_intrinsics = camera_intrinsics
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    
    def detect_colored_squares(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> List[Dict]:
        """
        Detect all colored squares in image
        
        Args:
            rgb_image: RGB image from camera
            depth_image: Aligned depth image
            
        Returns:
            List of detected squares with color, 2D centroid, and 3D position
        """
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        detected = []
        
        for color_name, ranges in self.COLOR_RANGES.items():
            mask = self._create_color_mask(hsv, ranges)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                # Check if area matches expected 100x100mm square
                if self._is_valid_square_area(area, depth_image, cnt):
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = w / h
                    
                    # Check square shape
                    if 0.8 < aspect_ratio < 1.2:
                        cx = x + w // 2
                        cy = y + h // 2
                        
                        # Get 3D position
                        depth = depth_image[cy, cx]
                        if depth > 0:
                            point_3d = self._deproject_pixel(cx, cy, depth)
                            
                            detected.append({
                                'color': color_name,
                                'centroid_2d': (cx, cy),
                                'position': point_3d,
                                'area': area,
                                'bounding_box': (x, y, w, h)
                            })
                            break  # Only one square per color
        
        return detected
    
    def _create_color_mask(self, hsv_image: np.ndarray, ranges: List[Tuple]) -> np.ndarray:
        """Create binary mask for color detection"""
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in ranges:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask |= cv2.inRange(hsv_image, lower, upper)
        
        # Morphological operations to reduce noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        return mask
    
    def _is_valid_square_area(self, pixel_area: float, depth_image: np.ndarray, 
                             contour: np.ndarray) -> bool:
        """Check if contour area matches 100x100mm square at measured depth"""
        # Get average depth of contour
        mask = np.zeros(depth_image.shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], 0, 255, -1)
        depths = depth_image[mask > 0]
        
        if len(depths) == 0:
            return False
        
        avg_depth = np.median(depths) / 1000.0  # Convert mm to m
        
        if avg_depth <= 0:
            return False
        
        # Calculate expected pixel area for 100x100mm square at this depth
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        
        expected_width_pixels = (0.1 * fx) / avg_depth
        expected_height_pixels = (0.1 * fy) / avg_depth
        expected_area = expected_width_pixels * expected_height_pixels
        
        # Allow 30% tolerance
        return 0.7 * expected_area < pixel_area < 1.3 * expected_area
    
    def _deproject_pixel(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        """
        Convert 2D pixel + depth to 3D point
        
        Args:
            x, y: Pixel coordinates
            depth: Depth in mm
            
        Returns:
            (X, Y, Z) 3D point in meters
        """
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']
        
        z = depth / 1000.0  # Convert to meters
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        
        return (x_3d, y_3d, z)


class QRDetector:
    """Detect and decode QR codes"""
    
    def __init__(self, camera_intrinsics: Dict):
        """
        Initialize QR detector
        
        Args:
            camera_intrinsics: Camera intrinsic parameters
        """
        self.camera_intrinsics = camera_intrinsics
        self.qr_detector = cv2.QRCodeDetector()
    
    def detect_qr_codes(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> List[Dict]:
        """
        Detect and decode all QR codes in image
        
        Args:
            rgb_image: RGB image from camera
            depth_image: Aligned depth image
            
        Returns:
            List of detected QR codes with text, position, and bounding box
        """
        detected = []
        
        # Try OpenCV QR detector first
        retval, decoded_info, points, straight_qrcode = self.qr_detector.detectAndDecodeMulti(rgb_image)
        
        if retval and decoded_info is not None:
            for i, text in enumerate(decoded_info):
                if text and points is not None and i < len(points):
                    # Calculate centroid of QR code
                    qr_points = points[i]
                    cx = int(np.mean(qr_points[:, 0]))
                    cy = int(np.mean(qr_points[:, 1]))
                    
                    # Get 3D position
                    depth = depth_image[cy, cx]
                    if depth > 0:
                        point_3d = self._deproject_pixel(cx, cy, depth)
                        
                        detected.append({
                            'text': text,
                            'centroid_2d': (cx, cy),
                            'position': point_3d,
                            'corners': qr_points.tolist()
                        })
        
        # Fallback to pyzbar if OpenCV fails
        if len(detected) == 0:
            barcodes = pyzbar.decode(rgb_image)
            for barcode in barcodes:
                text = barcode.data.decode('utf-8')
                
                # Calculate centroid
                points = barcode.polygon
                cx = int(np.mean([p.x for p in points]))
                cy = int(np.mean([p.y for p in points]))
                
                depth = depth_image[cy, cx]
                if depth > 0:
                    point_3d = self._deproject_pixel(cx, cy, depth)
                    
                    detected.append({
                        'text': text,
                        'centroid_2d': (cx, cy),
                        'position': point_3d,
                        'corners': [(p.x, p.y) for p in points]
                    })
        
        return detected
    
    def _deproject_pixel(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        """Convert 2D pixel + depth to 3D point"""
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']
        
        z = depth / 1000.0
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        
        return (x_3d, y_3d, z)


def visualize_detections(image: np.ndarray, color_detections: List[Dict], 
                        qr_detections: List[Dict]) -> np.ndarray:
    """
    Draw detection results on image for visualization
    
    Args:
        image: Input image
        color_detections: List of detected color squares
        qr_detections: List of detected QR codes
        
    Returns:
        Annotated image
    """
    vis_image = image.copy()
    
    # Draw color detections
    for det in color_detections:
        x, y, w, h = det['bounding_box']
        color = det['color']
        
        # Choose drawing color
        draw_color = (0, 0, 255) if color == 'red' else \
                    (0, 255, 0) if color == 'green' else \
                    (255, 0, 0)
        
        cv2.rectangle(vis_image, (x, y), (x+w, y+h), draw_color, 2)
        cv2.putText(vis_image, color, (x, y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
    
    # Draw QR detections
    for det in qr_detections:
        corners = np.array(det['corners'], dtype=np.int32)
        cv2.polylines(vis_image, [corners], True, (255, 255, 0), 2)
        
        cx, cy = det['centroid_2d']
        cv2.putText(vis_image, det['text'], (cx-50, cy-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    return vis_image
