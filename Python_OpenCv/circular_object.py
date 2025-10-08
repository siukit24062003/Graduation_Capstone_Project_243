import cv2
import numpy as np
import imutils
from disk_coordinate_transformer import DiskCoordinateTransformer
import time


class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        """This function estimates the position of the object"""
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        predicted = self.kf.predict()
        self.kf.correct(measured)
        return predicted

class BallObjectDetector:
    def __init__(self, camera_index=0, width=640, height=480):
        self.kf = KalmanFilter()
        self.predicted_coords = np.zeros((2, 1), np.float32)
        
        self.cached_frame = None
        self.cache_timestamp = 0
        self.cache_timeout = 0.1

        self.camera_index = int(camera_index)
        self.cap = None
        self.width = width
        self.height = height
        self.transformer = DiskCoordinateTransformer(min_diameter_cm=18, max_diameter_cm=19)
        self.disk_contour = None
        self.disk_calibrated = False
        self.roi_mask = None
        
        self.ball_colors = {
            'yellow': {
                'lower': np.array([130, 34, 147]),
                'upper': np.array([180, 161, 255]),
                'name': 'Yellow'
            },
            'white': {
                'lower': np.array([60, 5, 148]),
                'upper': np.array([136, 175, 255]),
                'name': 'White'
            },
            'red': {
                'lower': np.array([130, 34, 147]),
                'upper': np.array([180, 161, 255]),
                'name': 'Red'
            }
        }
        
        self.current_ball_color = 'yellow'
        self.ball_hsv_lower = self.ball_colors[self.current_ball_color]['lower']
        self.ball_hsv_upper = self.ball_colors[self.current_ball_color]['upper']
        
        self.close_kernel = np.ones((7, 7), np.uint8)
        self.open_kernel = np.ones((3, 3), np.uint8)
        
        self.cap = None
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY, None):
            cap = cv2.VideoCapture(self.camera_index) if backend is None else cv2.VideoCapture(self.camera_index, backend)
            if cap.isOpened():
                self.cap = cap
                break

        if not self.cap or not self.cap.isOpened():
            if self.cap:
                self.cap.release()
            self.cap = None
            raise RuntimeError(f"Cannot open camera {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def recalibrate_disk(self):
        """Reset disk calibration flag to trigger recalibration."""
        self.disk_calibrated = False
        self.disk_contour = None
        self.roi_mask = None

    def find_and_calibrate_disk(self, frame):
        """Finds the disk, calibrates, and prepares ROI for faster processing."""
        self.disk_contour = self._find_disk(frame)
        if self.disk_contour is not None:
            if self.transformer.calibrate_with_disk(self.disk_contour):
                self.roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
                cv2.drawContours(self.roi_mask, [self.disk_contour], -1, 255, -1)
                self.disk_calibrated = True
                return True
        self.disk_calibrated = False
        return False
        
    def _open_camera(self, camera_index, width, height):
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap

    def _find_disk(self, frame):
        """Find the disk in the frame, focusing only on the circular disk"""
        original = frame.copy()
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        blurred = cv2.GaussianBlur(gray, (5, 5), 1)
        
        frame_height, frame_width = frame.shape[:2]
        expected_radius_px = min(frame_width, frame_height) * 0.3
        
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT, 
            dp=1.5,
            minDist=frame_height/2,
            param1=40,
            param2=35,
            minRadius=int(expected_radius_px*0.5),
            maxRadius=int(expected_radius_px*1.8)
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            
            circle = circles[0, 0]
            center = (circle[0], circle[1])
            radius = circle[2]
            
            angles = np.linspace(0, 2*np.pi, 100)
            x_points = center[0] + radius * np.cos(angles)
            y_points = center[1] + radius * np.sin(angles)
            points = np.vstack((x_points, y_points)).T
            disk_contour = np.array(points, dtype=np.int32)
            
            center_x, center_y = center
            distance_to_center = np.sqrt((center_x - frame_width/2)**2 + (center_y - frame_height/2)**2)
            if distance_to_center > frame_width/3:
                return None
            
            return disk_contour

        return None

    def _find_contours(self, frame):
        """Find contours in the frame with improved disk detection"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        ball_mask = cv2.inRange(hsv, self.ball_hsv_lower, self.ball_hsv_upper)

        if self.disk_calibrated and self.roi_mask is not None:
            ball_mask = cv2.bitwise_and(ball_mask, ball_mask, mask=self.roi_mask)
        
        ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_CLOSE, self.close_kernel)
        ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_OPEN, self.open_kernel)
        
        ball_cnts = cv2.findContours(ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_cnts = imutils.grab_contours(ball_cnts)
        
        return ball_cnts, ball_mask

    def _find_roundness(self, c):
        perimeter = cv2.arcLength(c, True)
        area = cv2.contourArea(c)
        if perimeter == 0:
            return 0
        return 4 * np.pi * (area / (perimeter ** 2))

    def set_ball_color(self, color_name):
        """Set the ball color to detect - called from GUI"""
        if color_name in self.ball_colors:
            self.current_ball_color = color_name
            self.ball_hsv_lower = self.ball_colors[color_name]['lower']
            self.ball_hsv_upper = self.ball_colors[color_name]['upper']
            return True
        return False

    def get_available_colors(self):
        """Get list of available ball colors for GUI"""
        return list(self.ball_colors.keys())

    def _process_contours(self, cnts):
        centers = []
        contour_data = []

        for c in cnts:
            area = cv2.contourArea(c)
            if area > 2000:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
                    circularity = self._find_roundness(c)
                    if circularity > 0.6:
                        rect = cv2.minAreaRect(c)
                        box = np.int32(cv2.boxPoints(rect))

                        contour_data.append({
                            'contour': c,
                            'center': (cx, cy),
                            'circularity': circularity,
                            'box': box,
                            'area': area
                        })
                        
        if contour_data:
            contour_data = sorted(contour_data, key=lambda x: x['circularity'], reverse=True)[:1]
            centers = [data['center'] for data in contour_data]
        return centers, contour_data

    def _draw_on_frame(self, frame, contour_data):
        display_frame = frame.copy()
        if self.disk_contour is not None:
            cv2.drawContours(display_frame, [self.disk_contour], -1, (255, 0, 0), 2)
            M = cv2.moments(self.disk_contour)
            if M["m00"] > 0:
                disk_cx = int(M["m10"] / M["m00"])
                disk_cy = int(M["m01"] / M["m00"])
                cv2.circle(display_frame, (disk_cx, disk_cy), 4, (255, 0, 0), -1)
        
        predicted_coordinates = None
        for data in contour_data:
            (x, y), radius = cv2.minEnclosingCircle(data['contour'])
            center = (int(x), int(y))
            radius = int(radius)
            
            cv2.circle(display_frame, center, radius, (0, 255, 0), 2)
            cx, cy = data['center']
            # cv2.circle(display_frame, (cx, cy), 4, (255, 0, 0), -1)
            
            self.predicted_coords = self.kf.Estimate(cx, cy)
            predicted_x = int(self.predicted_coords[0])
            predicted_y = int(self.predicted_coords[1])
            
            predicted_coordinates = (predicted_x, predicted_y)
            
            cv2.circle(display_frame, (predicted_x, predicted_y), 25, [255,255,255], 4, 8)
            # cv2.line(display_frame, (predicted_x + 20, predicted_y - 20), (predicted_x + 60, predicted_y - 40), [100, 10, 255], 3, 8)
            # cv2.putText(display_frame, "Predicted", (predicted_x + 60, predicted_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, [50, 200, 250], 2)
            
            cm_x, cm_y = self.transformer.image_to_cm_coordinates(cx, cy)
            if cm_x is not None and cm_y is not None:
                cv2.putText(display_frame, f"X={cm_x:.1f}cm", (cx - 40, cy + 15), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Y={cm_y:.1f}cm", (cx - 40, cy + 30), 
                          cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 0, 0), 2)
        return display_frame, predicted_coordinates

    def release(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()

    def get_gray_threshold(self):
        """Get binary threshold of current frame using cached frame."""
        current_time = time.time()
        
        if (self.cached_frame is not None and 
            current_time - self.cache_timestamp < self.cache_timeout):
            frame = self.cached_frame
        else:
            ret, frame = self.cap.read()
            if not ret:
                return None
            frame = cv2.flip(frame, 1)
            self.cached_frame = frame.copy()
            self.cache_timestamp = current_time
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        _, thresh = cv2.threshold(gray, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresh

    def update_hsv_range(self, lower, upper):
        """Update HSV range from external source (like GUI model)"""
        self.ball_hsv_lower = lower.copy() if hasattr(lower, 'copy') else np.array(lower)
        self.ball_hsv_upper = upper.copy() if hasattr(upper, 'copy') else np.array(upper)

    def get_hsv_preview(self):
        """Return HSV mask and filtered image for preview with frame caching."""
        current_time = time.time()
        
        if (self.cached_frame is not None and 
            current_time - self.cache_timestamp < self.cache_timeout):
            frame = self.cached_frame
        else:
            ret, frame = self.cap.read()
            if not ret:
                return None, None
            frame = cv2.flip(frame, 1)
            self.cached_frame = frame.copy()
            self.cache_timestamp = current_time
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.ball_hsv_lower, self.ball_hsv_upper)
        filtered = cv2.bitwise_and(frame, frame, mask=mask)
        return mask, filtered