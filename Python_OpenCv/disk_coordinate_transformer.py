import math
import cv2
import numpy as np

class DiskCoordinateTransformer:
    def __init__(self, min_diameter_cm=18.0, max_diameter_cm=19.0):
        self.min_radius_cm = min_diameter_cm / 2
        self.max_radius_cm = max_diameter_cm / 2
        self.disk_radius_cm = None
        self.center_x = 0
        self.center_y = 0
        self.disk_contour = None
        self.pixels_per_cm = None
        self.prev_center_x = 0
        self.prev_center_y = 0
        self.prev_radius = 0
        
        self.radius_history = []
        self.center_history = []
        self.history_size = 5
        self.stable_radius = None
        self.stable_center = None
        self.change_threshold = 0.08
        self._center_change_threshold = 15

    def set_disk_center(self, frame_width, frame_height):
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2

    def _smooth_radius(self, new_radius):
        """Smooth radius using moving average filter"""
        self.radius_history.append(new_radius)
        
        if len(self.radius_history) > self.history_size:
            self.radius_history.pop(0)
        
        weights = np.linspace(0.5, 1.0, len(self.radius_history))
        weighted_avg = np.average(self.radius_history, weights=weights)
        
        return weighted_avg

    def _smooth_center(self, new_center):
        """Smooth center using moving average filter"""
        self.center_history.append(new_center)
        
        if len(self.center_history) > self.history_size:
            self.center_history.pop(0)
        
        if len(self.center_history) > 0:
            centers_array = np.array(self.center_history)
            weights = np.linspace(0.5, 1.0, len(self.center_history))
            weighted_center_x = np.average(centers_array[:, 0], weights=weights)
            weighted_center_y = np.average(centers_array[:, 1], weights=weights)
            return (int(weighted_center_x), int(weighted_center_y))
        
        return new_center

    def _is_significant_change(self, new_radius, new_center):
        """Check if the change is significant"""
        if self.stable_radius is None or self.stable_center is None:
            return True
        
        radius_change = abs(new_radius - self.stable_radius) / self.stable_radius
        
        center_change = math.sqrt((new_center[0] - self.stable_center[0])**2 + 
                                (new_center[1] - self.stable_center[1])**2)
        
        center_threshold = getattr(self, '_center_change_threshold', 10)
        
        return radius_change > self.change_threshold or center_change > center_threshold

    def calibrate_with_disk(self, disk_contour):
        """Calibrate the pixel to cm ratio using detected disk with improved stability"""
        if disk_contour is None:
            if self.pixels_per_cm is not None and self.center_x is not None and self.center_y is not None:
                return True
            return False
            
        self.disk_contour = disk_contour
        
        try:
            ellipse = cv2.fitEllipse(disk_contour)
            (x, y), (width, height), angle = ellipse
            
            radius_pixels = (width + height) / 4
            
            center_x, center_y = int(x), int(y)
        except:
            (x, y), radius_pixels = cv2.minEnclosingCircle(disk_contour)
            center_x, center_y = int(x), int(y)
            
            M = cv2.moments(disk_contour)
            if M["m00"] > 0:
                moment_x = int(M["m10"] / M["m00"])
                moment_y = int(M["m01"] / M["m00"])
                
                if abs(center_x - moment_x) > 5 or abs(center_y - moment_y) > 5:
                    center_x = (center_x + moment_x) // 2
                    center_y = (center_y + moment_y) // 2

        area_pixels = cv2.contourArea(disk_contour)
        perimeter_pixels = cv2.arcLength(disk_contour, True)
        
        area_radius = np.sqrt(area_pixels / np.pi)
        
        perimeter_radius = perimeter_pixels / (2 * np.pi)
        
        if abs(radius_pixels - area_radius) < abs(radius_pixels - perimeter_radius):
            raw_radius = (radius_pixels + area_radius) / 2
        else:
            raw_radius = radius_pixels
        
        raw_center = (center_x, center_y)
        
        smoothed_radius = self._smooth_radius(raw_radius)
        smoothed_center = self._smooth_center(raw_center)
        
        if self._is_significant_change(smoothed_radius, smoothed_center):
            self.stable_radius = smoothed_radius
            self.stable_center = smoothed_center
        
        self.center_x, self.center_y = self.stable_center
        
        estimated_radius_cm = (self.min_radius_cm + self.max_radius_cm) / 2
        self.disk_radius_cm = estimated_radius_cm
        
        self.pixels_per_cm = self.stable_radius / estimated_radius_cm
        
        return True

    def image_to_cm_coordinates(self, img_x, img_y):
        """Convert image coordinates to cm coordinates relative to disk center"""
        if self.pixels_per_cm is None or self.disk_radius_cm is None:
            return None, None

        cm_x = (img_x - self.center_x) / self.pixels_per_cm
        cm_y = (img_y - self.center_y) / self.pixels_per_cm # Fixed: removed negative sign to flip Y axis

        # Apply rotation correction to align with servo coordinate system
        # Rotation angle adjusted for flipped Y axis: from (-8, -4.4) to (9.4, 0)
        rotation_angle = 2.6373 # +151.12 degrees in radians (flipped due to Y axis change)
        cos_angle = math.cos(rotation_angle)
        sin_angle = math.sin(rotation_angle)
        
        rotated_x = cm_x * cos_angle - cm_y * sin_angle
        rotated_y = cm_x * sin_angle + cm_y * cos_angle

        distance_cm = math.sqrt(rotated_x**2 + rotated_y**2)

        if distance_cm > self.disk_radius_cm:
            return None, None

        return rotated_x, rotated_y

    def cm_to_image_coordinates(self, rotated_cm_x, rotated_cm_y):
        """Convert cm coordinates back to image coordinates"""
        if self.pixels_per_cm is None:
            return None, None

        # Inverse rotation logic
        rotation_angle = 2.6373 
        cos_angle = math.cos(rotation_angle)
        sin_angle = math.sin(rotation_angle)

        # Apply inverse rotation
        cm_x = rotated_cm_x * cos_angle + rotated_cm_y * sin_angle
        cm_y = -rotated_cm_x * sin_angle + rotated_cm_y * cos_angle
        
        # Apply inverse scaling and translation
        img_x = cm_x * self.pixels_per_cm + self.center_x
        img_y = cm_y * self.pixels_per_cm + self.center_y
        
        return int(img_x), int(img_y)