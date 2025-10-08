import sys
import cv2
import numpy as np
import imutils
import math
from collections import deque
from datetime import datetime
import time
import threading

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSlider, QSizePolicy, QGroupBox, QDoubleSpinBox, QMessageBox, QFrame, QLineEdit, 
    QComboBox, QSpinBox, QTextEdit, QCheckBox, QScrollArea, QTabWidget
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from circular_object import BallObjectDetector
from serial_communication import SerialCommunication




# --- VIEW ------------------------------------------------------------------
class BallValuesWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Ball Position Values')
        self.setStyleSheet("background-color: white; color: black;")
        
        # Initialize data storage with more points for longer time window
        self.max_points = 5000
        self.x_values = deque(maxlen=self.max_points)
        self.y_values = deque(maxlen=self.max_points)
        self.time_points = deque(maxlen=self.max_points)
        self.start_time = None
        self.last_update = 0
        self.update_interval = 100
        self.time_window = 70  # Hiển thị 70 giây dữ liệu

        # Trajectory state (disk view)
        self.disk_radius_cm = 9.5
        self.traj_x = []
        self.traj_y = []
        self._in_bounds = False
        self.traj_max_points = 4000
        self.is_trajectory_drawn = False # To track if we need to clear paths
        
        # Setup matplotlib with optimization
        plt.style.use('fast')
        plt.rcParams['animation.html'] = 'none'
        
        self._build_ui()
        
        # Use QTimer with increased frequency
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(self.update_interval)
        
        # Double buffering for plot data
        self._buffer_time = []
        self._buffer_x = []
        self._buffer_y = []
        
        # Cache for background
        self._backgrounds = []
        
        # Target management
        self.target_x = 0.0
        self.target_y = 0.0
        self.main_window = None  # Will be set by parent window
        
        # Setpoint tracking for plots
        self.setpoint_x_mm = 0.0  # Current setpoint in mm
        self.setpoint_y_mm = 0.0
        
        # Circle trajectory control
        self.circle_active = False
        self.circle_radius = 3.0  # cm
        self.circle_speed = 0.3   # rad/s
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.circle_start_time = 0

    def _build_ui(self):
        layout = QVBoxLayout(self)
        
        # Add control panel at the top
        control_panel = QWidget()
        control_panel.setStyleSheet("background-color: #f0f0f0; border-radius: 5px;")
        control_panel_layout = QVBoxLayout(control_panel)
        control_panel_layout.setContentsMargins(10, 10, 10, 10)
        control_panel_layout.setSpacing(8)
        
        
        layout.addWidget(control_panel)
        
        # Optimize figure creation
        plt.rcParams['figure.dpi'] = 100
        plt.rcParams['figure.autolayout'] = True
        plt.rcParams.update({
            'text.color': 'black',
            'axes.labelcolor': 'black',
            'xtick.color': 'black',
            'ytick.color': 'black',
            'figure.facecolor': 'white',
            'axes.facecolor': 'white',
            'axes.grid': False,
        })
        
        # Create figure (2x2 layout): left = disk view, right = X/Y vs time
        self.figure = plt.figure(figsize=(10, 6), facecolor='white', constrained_layout=True)
        gs = self.figure.add_gridspec(2, 2, width_ratios=[2.0, 1.0], height_ratios=[1.0, 1.0])
        self.ax_disk = self.figure.add_subplot(gs[:, 0])
        self.ax1 = self.figure.add_subplot(gs[0, 1])
        self.ax2 = self.figure.add_subplot(gs[1, 1])
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setParent(self)
        layout.addWidget(self.canvas)
        
        # Configure disk view axis
        self.ax_disk.set_title('Ball Trajectory on Disk (Real View)', fontsize=12, color='black')
        self.ax_disk.set_xlabel('X Position (cm)', fontsize=9, color='black')
        self.ax_disk.set_ylabel('Y Position (cm)', fontsize=9, color='black')
        self.ax_disk.set_xlim(-10, 10)
        self.ax_disk.set_ylim(-10, 10)
        self.ax_disk.set_aspect('equal', adjustable='box')
        
        # Set grid to 1cm spacing (20x20 grid from -10 to 10) and make it bolder
        major_ticks = np.arange(-10, 11, 1)
        self.ax_disk.set_xticks(major_ticks)
        self.ax_disk.set_yticks(major_ticks)
        self.ax_disk.grid(True, which='major', linestyle='-', linewidth=0.8, alpha=0.7)
        
        # Draw disk boundary (dashed)
        theta = np.linspace(0, 2*np.pi, 400)
        circle_x = self.disk_radius_cm * np.cos(theta)
        circle_y = self.disk_radius_cm * np.sin(theta)
        self.ax_disk.plot(circle_x, circle_y, 'k--', linewidth=2, label='Disk Boundary')
        # Target center
        self.ax_disk.plot([0], [0], 'ro', label='Target Center')
        # Trajectory and current point artists
        self.line_traj, = self.ax_disk.plot([], [], color='blue', linewidth=1.5, alpha=0.6, label='Ball Path')
        self.point_current, = self.ax_disk.plot([], [], 'ro', markersize=6, label='Current Position')
        self.point_target, = self.ax_disk.plot([0], [0], 'go', markersize=10, label='Target Position')
        self.line_target_circle, = self.ax_disk.plot([], [], color='red', linestyle='-', linewidth=2, alpha=0.8, label='Target Circle')
        self.line_target_path, = self.ax_disk.plot([], [], color='red', linestyle='-', linewidth=2, alpha=0.8) # For square/ramp
        # Add text for grid coordinates
        self.grid_coord_text = self.ax_disk.text(0.02, 0.98, 'Grid: (N/A, N/A)', 
                                                 transform=self.ax_disk.transAxes, 
                                                 fontsize=9,
                                                 verticalalignment='top', 
                                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

        # Pre-configure axes for better performance
        for ax, title in [
            (self.ax1, 'X Position & Setpoint'),
            (self.ax2, 'Y Position & Setpoint')
        ]:
            ax.set_title(title, fontsize=10, color='black')
            ax.set_xlabel('Time (s)', fontsize=8, color='black')
            ax.set_ylabel('Position (cm)', fontsize=8, color='black')
            ax.set_ylim(-10, 10)
            
            # Set major ticks every 2cm with small font size
            major_ticks = np.arange(-10, 11, 2)
            ax.set_yticks(major_ticks)
            ax.set_yticklabels(major_ticks, fontsize=7)
            
            # Vẽ lưới chính mỗi 2cm
            ax.grid(True, which='major', linestyle='-', alpha=0.2)
            
            # Vẽ vạch chia phụ mỗi 1cm nhưng không hiển thị số
            minor_ticks = np.arange(-10, 11, 1)
            ax.set_yticks(minor_ticks, minor=True)
            ax.tick_params(axis='y', which='major', length=6)
            ax.tick_params(axis='y', which='minor', length=3)
            ax.set_yticklabels([], minor=True)
            
            ax.set_facecolor('white')
            
            # Create the pre-drawn setpoint path (initially empty)
            setpoint_path_line, = ax.plot([], [], color='red', linestyle='-', linewidth=1.5, alpha=0.7, label='Target Path')
            
            # Create dynamic setpoint line (red) - thinner
            setpoint_line, = ax.plot([], [], color='red', linestyle='-', linewidth=1, alpha=0.8, label='Setpoint')
            
            # Create empty line objects for updating - thicker
            line, = ax.plot([], [], '-', linewidth=1.2, color='blue', alpha=0.4, label='Ball Position')
            
            # Store line objects for updates
            if ax == self.ax1:
                self.line_x = line
                self.setpoint_line_x = setpoint_line
                self.setpoint_path_x = setpoint_path_line
            else:
                self.line_y = line
                self.setpoint_line_y = setpoint_line
                self.setpoint_path_y = setpoint_path_line
                
            # Initialize time axis from 0 to 10 seconds
            ax.set_xlim(0, self.time_window)
            
            # Set major ticks every 2 seconds
            major_ticks = np.arange(0, self.time_window + 1, 2)
            ax.set_xticks(major_ticks)
            ax.set_xticklabels([f"{t:.0f}" for t in major_ticks], fontsize=7)
            
            # Set minor ticks every 1 second
            minor_ticks = np.arange(0, self.time_window + 1, 1)
            ax.set_xticks(minor_ticks, minor=True)
            ax.tick_params(axis='x', which='major', length=6)
            ax.tick_params(axis='x', which='minor', length=3)
            
            # Only show grid for major ticks (2s intervals)
            ax.grid(True, which='major', axis='x', linestyle='-', alpha=0.2)
            ax.grid(False, which='minor', axis='x')
            
            # Add legend for each axis
            # ax.legend(loc='upper right', fontsize=7, framealpha=0.8)
        
        # Connect mouse click event
        self.canvas.mpl_connect('button_press_event', self.on_disk_click)
        
        # Cache the background for blitting
        self.canvas.draw()
        self._backgrounds = [self.figure.canvas.copy_from_bbox(ax.bbox) for ax in [self.ax_disk, self.ax1, self.ax2]]
        
        # Initialize setpoint at center (0,0)
        self.set_target(0.0, 0.0)

    def _update_plots(self):
        try:
            current_time = time.time()
            if current_time - self.last_update < self.update_interval / 1000:
                return
            
            # Update main data from buffer
            if self._buffer_time:
                self.time_points.extend(self._buffer_time)
                self.x_values.extend(self._buffer_x)
                self.y_values.extend(self._buffer_y)
                
                # Clear buffers
                self._buffer_time.clear()
                self._buffer_x.clear()
                self._buffer_y.clear()
            else:
                return  # No new data to plot
                
            # Circle trajectory is now handled by MainWindow
            
            # Convert deques to lists for plotting
            times = list(self.time_points)
            x_vals = list(self.x_values)
            y_vals = list(self.y_values)
            
            # Auto-reset data every 10 seconds
            if times and times[-1] > self.time_window:
                # Reset all data but keep current position
                last_x = self.x_values[-1] if self.x_values else 0
                last_y = self.y_values[-1] if self.y_values else 0
                
                # Clear main data
                self.time_points.clear()
                self.x_values.clear()
                self.y_values.clear()
                
                # Clear buffers to prevent old data from being added back
                self._buffer_time.clear()
                self._buffer_x.clear()
                self._buffer_y.clear()
                
                # Reset time reference
                self.start_time = datetime.now()
                
                # Add current position as first point in new time window
                new_t = 0.0
                self.time_points.append(new_t)
                self.x_values.append(last_x)
                self.y_values.append(last_y)
                
                # Update lists for plotting
                times = [new_t]
                x_vals = [last_x]
                y_vals = [last_y]
            
            # Restore backgrounds
            for ax, background in zip([self.ax_disk, self.ax1, self.ax2], self._backgrounds):
                self.figure.canvas.restore_region(background)
            
            # Update line data
            self.line_x.set_data(times, x_vals)
            self.line_y.set_data(times, y_vals)
            
            # Update setpoint lines
            if times:
                # Hide the dynamic setpoint line during square mode, as the path is pre-drawn
                if self.main_window and self.main_window.active_trajectory == 'square':
                    self.setpoint_line_x.set_data([], [])
                    self.setpoint_line_y.set_data([], [])
                else:
                    setpoint_times = [times[0], times[-1]]  # Start and end time
                    self.setpoint_line_x.set_data(setpoint_times, [self.setpoint_x_mm, self.setpoint_x_mm])
                    self.setpoint_line_y.set_data(setpoint_times, [self.setpoint_y_mm, self.setpoint_y_mm])

            # Update disk trajectory and current point
            if self.traj_x:
                self.line_traj.set_data(self.traj_x, self.traj_y)
                # Calculate and display grid coordinates
                x, y = self.traj_x[-1], self.traj_y[-1]
                col = int(math.floor(x + 10) + 1)
                row = int(math.floor(-y + 10) + 1)
                
                # Clamp values to be within [1, 20]
                col = max(1, min(20, col))
                row = max(1, min(20, row))
                
                self.grid_coord_text.set_text(f'Grid: (R:{row}, C:{col})')
            else:
                self.line_traj.set_data([], [])
                self.grid_coord_text.set_text('Grid: (N/A, N/A)')
            if self._in_bounds and self.traj_x:
                self.point_current.set_data([self.traj_x[-1]], [self.traj_y[-1]])
            else:
                self.point_current.set_data([], [])
            
            # Update x-axis window only (keep tick/grid static for performance)
            if times:
                current_time_plot = times[-1]
                start_time_plot = max(0, current_time_plot - self.time_window)
                end_time_plot = max(self.time_window, current_time_plot)
                self.ax1.set_xlim(start_time_plot, end_time_plot)
                self.ax2.set_xlim(start_time_plot, end_time_plot)
            
            # Draw only the updated artists
            self.ax_disk.draw_artist(self.line_traj)
            self.ax_disk.draw_artist(self.point_current)
            self.ax_disk.draw_artist(self.point_target)
            self.ax_disk.draw_artist(self.grid_coord_text)
            self.ax1.draw_artist(self.line_x)
            self.ax1.draw_artist(self.setpoint_line_x)
            self.ax2.draw_artist(self.line_y)
            self.ax2.draw_artist(self.setpoint_line_y)
            
            # Update the display
            self.canvas.blit(self.ax_disk.bbox)
            self.canvas.blit(self.ax1.bbox)
            self.canvas.blit(self.ax2.bbox)
            
            self.last_update = current_time
            
        except Exception as e:
            print(f"Error in _update_plots: {e}")
            import traceback
            traceback.print_exc()
            # Keep timer running even if there's an error
            return

    def showEvent(self, event):
        super().showEvent(event)
        # Re-cache background when window is shown
        self.canvas.draw()
        self._backgrounds = [self.figure.canvas.copy_from_bbox(ax.bbox) for ax in [self.ax_disk, self.ax1, self.ax2]]
        self.timer.start(self.update_interval)

    def hideEvent(self, event):
        super().hideEvent(event)
        self.timer.stop()

    def closeEvent(self, event):
        """Properly clean up resources when window is closed"""
        # Stop the timer first
        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        
        # Clean up matplotlib resources
        if hasattr(self, 'figure'):
            plt.close(self.figure)
            self.figure = None
        
        # Clear all data
        if hasattr(self, 'x_values'):
            self.x_values.clear()
        if hasattr(self, 'y_values'):
            self.y_values.clear()
        if hasattr(self, 'time_points'):
            self.time_points.clear()
        if hasattr(self, 'traj_x'):
            self.traj_x.clear()
        if hasattr(self, 'traj_y'):
            self.traj_y.clear()
        if hasattr(self, '_buffer_time'):
            self._buffer_time.clear()
        if hasattr(self, '_buffer_x'):
            self._buffer_x.clear()
        if hasattr(self, '_buffer_y'):
            self._buffer_y.clear()
        
        # Reset state
        self.start_time = None
        self._in_bounds = False
        
        # Call parent closeEvent
        super().closeEvent(event)

    def update_data(self, x, y):
        """Update plot data with new coordinates"""
        if self.start_time is None:
            self.start_time = datetime.now()
        
        # Keep data in cm units (no conversion needed)
        t = (datetime.now() - self.start_time).total_seconds()
        self._buffer_time.append(t)
        self._buffer_x.append(x)  # Keep in cm
        self._buffer_y.append(y)  # Keep in cm

        # Trajectory update logic
        if not self._in_bounds:
            # Re-entered: clear old trajectory and start a new one
            self.traj_x.clear()
            self.traj_y.clear()
            self._in_bounds = True
        self.traj_x.append(x)
        self.traj_y.append(y)
        # Cap trajectory length to keep drawing light
        if len(self.traj_x) > self.traj_max_points:
            self.traj_x.pop(0)
            self.traj_y.pop(0)

    def reset_trajectory(self):
        """Clear trajectory immediately (ball left plate)"""
        self.traj_x.clear()
        self.traj_y.clear()
        self._in_bounds = False

    def mark_ball_out(self):
        """Mark ball as out-of-plate: keep current trajectory frozen."""
        self._in_bounds = False
    
    def on_preset_clicked(self, x, y):
        """Handle preset button click to set target position"""
        if self.main_window:
             self.main_window.stop_trajectory()
        self.clear_target_trajectory()
        # Update target position
        self.set_target(x, y)
        
        # Send target to STM32 if main window is available
        if self.main_window and self.main_window.serial_comm.is_connected:
            success = self.main_window.serial_comm.send_target_coordinates(x, y)
            if success:
                print(f"Target sent: X={x:.2f}cm, Y={y:.2f}cm")
            else:
                print("Failed to send target coordinates")
        
        # Force redraw
        self.canvas.draw()
    
    def on_disk_click(self, event):
        """Handle mouse click on disk plot to set new target"""
        if event.inaxes == self.ax_disk:
            # Get click coordinates
            click_x = event.xdata
            click_y = event.ydata
            
            if click_x is not None and click_y is not None:
                # Check if click is within disk boundary
                distance = math.sqrt(click_x**2 + click_y**2)
                if distance <= self.disk_radius_cm:
                    if self.main_window:
                        self.main_window.stop_trajectory()
                    self.clear_target_trajectory()
                    # Update target position
                    self.target_x = click_x
                    self.target_y = click_y
                    
                    # Update setpoint for plots (keep in cm)
                    self.setpoint_x_mm = click_x
                    self.setpoint_y_mm = click_y
                    
                    # Update target point display
                    self.point_target.set_data([self.target_x], [self.target_y])
                    
                    # Send target to STM32 if main window is available
                    if self.main_window and self.main_window.serial_comm.is_connected:
                        success = self.main_window.serial_comm.send_target_coordinates(self.target_x, self.target_y)
                        if success:
                            print(f"Target sent: X={self.target_x:.2f}cm, Y={self.target_y:.2f}cm")
                        else:
                            print("Failed to send target coordinates")
                    
                    # Force redraw
                    self.canvas.draw()
    
    def set_target(self, x, y):
        """Set target position programmatically"""
        self.target_x = x
        self.target_y = y
        self.point_target.set_data([self.target_x], [self.target_y])
        
        # Update setpoint for plots (keep in cm)
        self.setpoint_x_mm = x
        self.setpoint_y_mm = y
        
    def update_setpoint_from_main(self, x, y):
        """Update setpoint display from main window (for circle mode)"""
        # Update target position for display only
        self.target_x = x
        self.target_y = y
        self.point_target.set_data([self.target_x], [self.target_y])
        
        # Update setpoint for plots (keep in cm)
        self.setpoint_x_mm = x
        self.setpoint_y_mm = y

    def draw_circle_trajectory(self, radius, speed, ramp_duration=5.0):
        """Pre-draws the target circle and sinusoidal setpoint paths."""
        # 1. Draw target circle and ramp on disk view
        theta = np.linspace(0, 2 * np.pi, 200)
        circle_x = radius * np.cos(theta)
        circle_y = radius * np.sin(theta)
        self.line_target_circle.set_data(circle_x, circle_y)
        self.line_target_path.set_data([0, radius], [0, 0])

        # 2. Draw combined ramp + circle paths for X and Y axes
        duration = self.time_window # 30s
        t_points = 1000
        t_path = np.linspace(0, duration, t_points) 
        x_path = np.zeros_like(t_path)
        y_path = np.zeros_like(t_path)

        # Find the index where ramp ends
        ramp_end_index = int((ramp_duration / duration) * t_points)

        # Ramp phase (0 to 5s)
        if ramp_end_index > 0:
            t_ramp = np.linspace(0, ramp_duration, ramp_end_index)
            x_path[:ramp_end_index] = (radius / ramp_duration) * t_ramp
            y_path[:ramp_end_index] = 0.0

        # Circle phase (5s to 30s)
        if ramp_end_index < t_points:
            t_circle = np.linspace(0, duration - ramp_duration, t_points - ramp_end_index)
            x_path[ramp_end_index:] = radius * np.cos(speed * t_circle)
            y_path[ramp_end_index:] = radius * np.sin(speed * t_circle)
        
        self.setpoint_path_x.set_data(t_path, x_path)
        self.setpoint_path_y.set_data(t_path, y_path)
        
        self.is_trajectory_drawn = True
        self.canvas.draw() # Force a full redraw to show the new paths

    def draw_square_trajectory(self, side, dwell_duration):
        """Pre-draws the target square path with dwell times and step changes."""
        R = side
        # 1. Define vertices and path for the disk view
        vertices = [(R, 0), (0, R), (-R, 0), (0, -R)]
        path_points_disk = [(0, 0)] + vertices + [vertices[0]]
        
        path_x_disk, path_y_disk = zip(*path_points_disk)
        self.line_target_path.set_data(path_x_disk, path_y_disk)

        # 3. Generate time-based paths for X and Y axes
        t_path, x_path, y_path = [], [], []
        current_time = 0.0
        
        # Cycle through vertices
        num_cycles = 3
        for _ in range(num_cycles):
            for i in range(len(vertices)):
                p_current = vertices[i]
                
                # Dwell at the current point
                t_start_dwell = current_time
                t_end_dwell = t_start_dwell + dwell_duration
                t_path.extend([t_start_dwell, t_end_dwell])
                x_path.extend([p_current[0], p_current[0]])
                y_path.extend([p_current[1], p_current[1]])
                
                current_time = t_end_dwell
                
                # Add points for vertical step to the *next* point
                p_next = vertices[(i + 1) % len(vertices)]
                t_path.append(current_time)
                x_path.append(p_current[0])
                y_path.append(p_current[1])
                
                t_path.append(current_time)
                x_path.append(p_next[0])
                y_path.append(p_next[1])

        self.setpoint_path_x.set_data(t_path, x_path)
        self.setpoint_path_y.set_data(t_path, y_path)

        self.is_trajectory_drawn = True
        self.canvas.draw()

    def clear_target_trajectory(self):
        """Clears the pre-drawn target paths."""
        if self.is_trajectory_drawn:
            self.line_target_circle.set_data([], [])
            self.line_target_path.set_data([], [])
            self.setpoint_path_x.set_data([], [])
            self.setpoint_path_y.set_data([], [])
            self.is_trajectory_drawn = False
            self.canvas.draw()



class HSVAdjustmentWindow(QWidget):
    def __init__(self, ball_detector):
        super().__init__()
        self.ball_detector = ball_detector
        self.setWindowTitle('HSV & Gray Threshold Preview')
        self.setStyleSheet("background-color: white; color: black;")
        self._build_ui()
        
        # Reduce timer frequency to 300ms (3.3 FPS) instead of 100ms (10 FPS) to save CPU
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_previews)
        self.timer.start(300)  # Increased from 100ms to 300ms
        
        # Flag to track if window is visible
        self.is_visible = False

    def showEvent(self, event):
        """Called when window becomes visible"""
        super().showEvent(event)
        self.is_visible = True
        # Start timer when window is shown
        if not self.timer.isActive():
            self.timer.start(300)

    def hideEvent(self, event):
        """Called when window becomes hidden"""
        super().hideEvent(event)
        self.is_visible = False
        # Stop timer when window is hidden to save CPU
        self.timer.stop()

    def _build_ui(self):
        main = QVBoxLayout(self)

        # hai màn hình preview
        hl = QHBoxLayout()
        self.gray_label = QLabel()
        self.hsv_mask_label = QLabel()
        for lbl in (self.gray_label, self.hsv_mask_label):
            lbl.setFixedSize(200,200)
            lbl.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
            lbl.setStyleSheet("color: black;")
            hl.addWidget(lbl)
        main.addLayout(hl)

        # sliders HSV lower
        self._make_sliders(main, "Lower HSV", self.ball_detector.ball_hsv_lower, True)
        # sliders HSV upper
        self._make_sliders(main, "Upper HSV", self.ball_detector.ball_hsv_upper, False)

    def _make_sliders(self, layout, title, arr, is_lower):
        layout.addWidget(QLabel(title, styleSheet="color: black; font-weight: bold;"))
        for i, comp in enumerate(('H','S','V')):
            box = QHBoxLayout()
            label = QLabel(comp)
            label.setStyleSheet("color: black;")
            box.addWidget(label)
            s = QSlider(Qt.Orientation.Horizontal)
            s.setRange(0, 180 if comp=='H' else 255)
            s.setValue(int(arr[i]))
            v = QLabel(str(int(arr[i])))
            v.setStyleSheet("color: black;")
            s.valueChanged.connect(
                lambda val, idx=i, lbl=v, low=is_lower: self._on_hsv_slider(val, idx, lbl, low)
            )
            box.addWidget(s)
            box.addWidget(v)
            layout.addLayout(box)

    def _on_hsv_slider(self, val, idx, label, is_lower):
        label.setText(str(val))
        if is_lower:
            self.ball_detector.ball_hsv_lower[idx] = val
        else:
            self.ball_detector.ball_hsv_upper[idx] = val

    def _refresh_previews(self):
        # Only refresh if window is visible to save CPU
        if not self.is_visible or not self.isVisible():
            return
            
        try:
            # Gray threshold preview
            thresh = self.ball_detector.get_gray_threshold()
            if thresh is not None:
                img = QImage(thresh.data, thresh.shape[1], thresh.shape[0], thresh.strides[0], QImage.Format.Format_Grayscale8)
                self.gray_label.setPixmap(QPixmap.fromImage(img).scaled(
                    self.gray_label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation))

            # HSV mask preview
            mask, filt = self.ball_detector.get_hsv_preview()
            if mask is not None:
                img2 = QImage(mask.data, mask.shape[1], mask.shape[0], mask.strides[0], QImage.Format.Format_Grayscale8)
                self.hsv_mask_label.setPixmap(QPixmap.fromImage(img2).scaled(
                    self.hsv_mask_label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation))
        except Exception as e:
            # If any error occurs, don't crash the preview
            pass

    def closeEvent(self, event):
        """Called when window is closed"""
        # Stop timer when window is closed to save CPU
        if self.timer.isActive():
            self.timer.stop()
        self.is_visible = False
        super().closeEvent(event)





class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Ball Detection GUI')
        
        # Initialize frame counter and FPS calculation variables
        self.frame_count = 0
        self.fps = 0
        self.last_fps_update = datetime.now()
        self.fps_update_interval = 0.5  # Update FPS every 0.5 second
        self.frame_skip_counter = 0
        self.frame_skip_rate = 1  # Giữ nguyên xử lý tất cả frame để đảm bảo phản hồi nhanh
        
        # Initialize serial communication
        self.serial_comm = SerialCommunication()
        
        # Khởi tạo các biến trạng thái
        self.camera_running = False
        self.current_camera_index = 0  # Camera Webcam
        self.ball_detector = None
        self.hsv_window = None
        self.values_window = None
        self.current_ball_pos = (0, 0)
        self.current_target = (0.0, 0.0)  # Target position
        
        # Giá trị mặc định cho độ phân giải camera
        self.current_width = 640
        self.current_height = 480
        self.current_fps = 60
        
        # Thêm biến để theo dõi thời gian gửi
        self.last_send_time = 0
        self.send_interval = 0.02
        
        # Trajectory control state
        self.active_trajectory = None # None, "ramping-circle", "circle", "square"
        self.trajectory_start_time = 0
        
        # Circle trajectory parameters
        self.circle_radius = 4.0
        self.circle_speed = 0.2
        self.ramp_duration = 5.0

        # Square trajectory parameters
        self.square_side = 4.0
        self.square_dwell_duration = 10.0
        
        # Tracking Error Plot data
        self.error_max_points = 2000
        self.error_time = deque(maxlen=self.error_max_points)
        self.error_x = deque(maxlen=self.error_max_points)
        self.error_y = deque(maxlen=self.error_max_points)
        self.error_start_time = None
        
        # Khởi tạo giao diện người dùng
        self.setup_ui()
        
        # Create timer for video update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer_interval = 16
        
        # Timer for plot updates (less frequent than frame updates)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_update_interval = 100 # ms, 10 FPS
        
        self.update_resolution_display()
        
        # Thử kết nối với camera ban đầu sau khi giao diện đã được tạo
        self.try_connect_camera(0)  # Mặc định kết nối Webcam

    def setup_ui(self):
        # Set background color
        self.setStyleSheet("background-color: white;")
        
        # Create central widget with horizontal layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left side control panel
        left_panel = QWidget()
        left_panel.setFixedWidth(280)  # Tăng từ 250 lên 280 để hiển thị đầy đủ các nút
        left_panel.setStyleSheet("background-color: #f0f0f0; border-radius: 10px;")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(8)  # Reduced spacing from 10
        left_layout.setContentsMargins(8, 15, 8, 15)  # Adjusted margins
        
        # Controls label (centered) - REMOVED
        # controls_label = QLabel("Controls")
        # controls_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #444;")
        # controls_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # left_layout.addWidget(controls_label)
        
        # Camera selection group
        camera_group = QGroupBox()
        camera_group.setStyleSheet("QGroupBox { border: none; }")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.setSpacing(5)
        
        # Camera selection
        camera_selection_layout = QHBoxLayout()
        camera_label = QLabel("Camera:")
        camera_label.setStyleSheet("font-weight: bold; color: #000000;")
        self.camera_combo = QComboBox()
        self.camera_combo.setStyleSheet("""
            QComboBox {
                background-color: #ffffff;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                color: #000000;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: #000000;
                background-color: #ffffff;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: #000000;
                background-color: #e8f8e8;
                font-weight: bold;
            }
            QComboBox QAbstractItemView {
                color: #000000;
                background-color: #ffffff;
                selection-background-color: #e8f8e8;
                selection-color: #000000;
            }
        """)
        self.camera_combo.addItems(["Camera 0 (Webcam)", "Camera 1 (USB)", "Camera 2", "Camera 3"])
        self.camera_combo.setCurrentIndex(0)  # Mặc định chọn Webcam
        camera_selection_layout.addWidget(camera_label)
        camera_selection_layout.addWidget(self.camera_combo)
        camera_layout.addLayout(camera_selection_layout)
        
        # Camera control buttons
        button_layout = QHBoxLayout()
        button_layout.setSpacing(5)
        
        self.start_camera_button = QPushButton('Start Camera')
        self.start_camera_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60; 
                color: white; 
                border-radius: 3px; 
                padding: 5px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #2ecc71; }
            QPushButton:disabled { background-color: #95a5a6; }
        """)
        
        self.stop_camera_button = QPushButton('Stop Camera')
        self.stop_camera_button.setStyleSheet("""
            QPushButton {
                background-color: #c0392b; 
                color: white; 
                border-radius: 3px; 
                padding: 5px; 
                font-weight: bold;
            }
            QPushButton:hover { background-color: #e74c3c; }
            QPushButton:disabled { background-color: #95a5a6; }
        """)
        
        button_layout.addWidget(self.start_camera_button)
        button_layout.addWidget(self.stop_camera_button)
        camera_layout.addLayout(button_layout)
        left_layout.addWidget(camera_group)
        
        # Camera Parameters group
        camera_params_group = QGroupBox("Camera Parameters")
        camera_params_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        params_layout = QVBoxLayout(camera_params_group)
        params_layout.setSpacing(5)
        
        self.resolution_label = QLabel("Resolution: --x--")
        self.fps_label = QLabel("FPS: --")
        
        for label in [self.resolution_label, self.fps_label]:
            label.setStyleSheet("""
                color: black;
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 3px;
                padding: 3px;
                margin: 2px;
            """)
            params_layout.addWidget(label)
        
        left_layout.addWidget(camera_params_group)
        
        # Control buttons group
        control_buttons_group = QGroupBox()
        control_buttons_group.setStyleSheet("QGroupBox { border: none; }")
        buttons_layout = QVBoxLayout(control_buttons_group)
        buttons_layout.setSpacing(0)  # Đặt về 0 để tự quản lý spacing
        
        # Create control buttons with consistent styling
        button_configs = [
            ('Adjust HSV', '#3498db', '#2980b9'),
            ('Ball Values', '#2ecc71', '#27ae60'),
            ('Recalibrate Disk', '#f39c12', '#e67e22')
        ]
        
        for text, bg_color, hover_color in button_configs:
            btn = QPushButton(text)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {bg_color};
                    color: white;
                    border-radius: 5px;
                    padding: 5px;
                    font-weight: bold;
                    font-size: 10px;
                    min-height: 26px;
                    text-align: center;
                }}
                QPushButton:hover {{
                    background-color: {hover_color};
                    transform: translateY(-1px);
                }}
                QPushButton:pressed {{
                    transform: translateY(1px);
                }}
            """)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            buttons_layout.addWidget(btn)
            
            # Thêm khoảng cách giống như giữa Connect button và status label
            buttons_layout.addSpacing(5)
            
            # Connect button signals
            if text == 'Adjust HSV':
                btn.clicked.connect(self.show_hsv_adjustment)
            elif text == 'Ball Values':
                btn.clicked.connect(self.show_ball_values)
            elif text == 'Recalibrate Disk':
                btn.clicked.connect(self.recalibrate_disk)
        
        left_layout.addWidget(control_buttons_group)
        
        # Ball Position group
        position_group = QGroupBox("Ball Position")
        position_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        position_layout = QVBoxLayout(position_group)
        position_layout.setSpacing(5)
        
        # Ball Color selection
        color_layout = QHBoxLayout()
        color_label = QLabel("Ball Color:")
        color_label.setStyleSheet("font-weight: bold; color: #8e44ad; font-size: 12px;")
        
        self.ball_color_combo = QComboBox()
        self.ball_color_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: black;
                background-color: white;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: black;
                background-color: #e8f8e8;
            }
            QComboBox QAbstractItemView {
                color: black;
                background-color: white;
                selection-background-color: #e8f8e8;
                selection-color: black;
            }
        """)
        self.ball_color_combo.addItems(["Yellow", "White", "Red"])
        self.ball_color_combo.setCurrentText("Red")  # Default to red
        self.ball_color_combo.currentTextChanged.connect(self.change_ball_color)
        
        color_layout.addWidget(color_label)
        color_layout.addWidget(self.ball_color_combo)
        position_layout.addLayout(color_layout)
        
        # X and Y coordinates
        for coord, color in [('X:', '#e74c3c'), ('Y:', '#3498db')]:
            coord_layout = QHBoxLayout()
            label = QLabel(coord)
            label.setStyleSheet(f"font-weight: bold; color: {color}; font-size: 14px;")
            
            value_edit = QLineEdit()
            value_edit.setReadOnly(True)
            value_edit.setStyleSheet("""
                background-color: white;
                color: black;
                padding: 5px;
                border: 1px solid #ddd;
                border-radius: 3px;
                font-weight: bold;
            """)
            value_edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value_edit.setText("0.0 cm")
            
            if coord == 'X:':
                self.x_value_edit = value_edit
            else:
                self.y_value_edit = value_edit
                
            coord_layout.addWidget(label)
            coord_layout.addWidget(value_edit)
            position_layout.addLayout(coord_layout)
        
        # Add Ball Position group after control buttons
        left_layout.addWidget(position_group)
        
        # Target Position group - Simplified with only target coordinates
        target_group = QGroupBox("Target Position")
        target_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 10px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        # Adjusted minimum height for Target Position group to give it more space
        target_group.setMinimumHeight(180)
        
        target_layout = QVBoxLayout(target_group)
        target_layout.setSpacing(4)
        
        # Target X and Y coordinates
        for coord, color in [('Target X:', '#27ae60'), ('Target Y:', '#27ae60')]:
            coord_layout = QHBoxLayout()
            label = QLabel(coord)
            label.setStyleSheet(f"font-weight: bold; color: {color}; font-size: 12px;")
            
            value_edit = QLineEdit()
            value_edit.setReadOnly(True)
            value_edit.setStyleSheet("""
                background-color: #e8f5e8;
                color: black;
                padding: 4px;
                border: 1px solid #27ae60;
                border-radius: 3px;
                font-weight: bold;
            """)
            value_edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value_edit.setText("0.0 cm")
            
            if coord == 'Target X:':
                self.target_x_edit = value_edit
            else:
                self.target_y_edit = value_edit
                
            coord_layout.addWidget(label)
            coord_layout.addWidget(value_edit)
            target_layout.addLayout(coord_layout)
        
        # Target coordinates only in this section
        target_layout.addSpacing(5)
        
        # Compact Target Position group for the camera panel - REMOVED
        
        # Preset target buttons
        preset_layout = QHBoxLayout()
        preset_layout.setSpacing(6)
        preset_buttons = [
            ('Center', 0.0, 0.0),
            ('Up', 0.0, 4.0),
            ('Down', 0.0, -4.0),
            ('Left', -4.0, 0.0),
            ('Right', 4.0, 0.0)
        ]
        
        for text, x, y in preset_buttons:
            btn = QPushButton(text)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    border-radius: 3px;
                    padding: 3px;
                    font-weight: bold;
                    font-size: 9px;
                    min-height: 18px;
                    min-width: 40px;
                    max-height: 22px;
                    max-width: 45px;
                }
                QPushButton:hover {
                    background-color: #2ecc71;
                }
                QPushButton:pressed {
                    background-color: #1e8449;
                }
            """)
            btn.clicked.connect(lambda checked, tx=x, ty=y: self.set_target_position(tx, ty))
            preset_layout.addWidget(btn)
        
        target_layout.addLayout(preset_layout)
        left_layout.addWidget(target_group)
        
        # Serial Connection group
        serial_group = QGroupBox("Serial Connection")
        serial_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 5px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        serial_layout = QVBoxLayout()
        serial_layout.setSpacing(5)
        
        # COM Port selection
        port_layout = QHBoxLayout()
        port_label = QLabel("COM Port:")
        port_label.setStyleSheet("color: black;")
        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet("""
            QComboBox {
                background-color: #ffffff;
                color: #2c3e50;
                border: 2px solid #3498db;
                border-radius: 5px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
                min-height: 20px;
            }
            QComboBox:hover {
                border: 2px solid #2980b9;
                background-color: #ecf0f1;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #2c3e50;
                width: 0;
                height: 0;
            }
            QComboBox::item {
                color: #2c3e50;
                background-color: #ffffff;
                padding: 8px;
                font-weight: bold;
                border-bottom: 1px solid #ecf0f1;
            }
            QComboBox::item:hover {
                color: #ffffff;
                background-color: #3498db;
            }
            QComboBox::item:selected {
                color: #ffffff;
                background-color: #2980b9;
                font-weight: bold;
            }
            QComboBox QAbstractItemView {
                color: #2c3e50;
                background-color: #ffffff;
                selection-background-color: #2980b9;
                selection-color: #ffffff;
                border: 2px solid #3498db;
                border-radius: 5px;
                outline: none;
            }
        """)
        # Add available COM ports
        self.port_combo.addItems(self.serial_comm.get_available_ports())
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)
        
        # Baud Rate selection
        baud_layout = QHBoxLayout()
        baud_label = QLabel("Baud Rate:")
        baud_label.setStyleSheet("color: black;")
        self.baud_combo = QComboBox()
        self.baud_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: black;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::item {
                color: black;
                background-color: white;
                padding: 5px;
            }
            QComboBox::item:selected {
                color: black;
                background-color: #e8f8e8;
            }
            QComboBox QAbstractItemView {
                color: black;
                background-color: white;
                selection-background-color: #e8f8e8;
                selection-color: black;
            }
        """)
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baud_combo.setCurrentText('115200')  # Default baud rate
        baud_layout.addWidget(baud_label)
        baud_layout.addWidget(self.baud_combo)
        
        # Connect button
        self.connect_button = QPushButton('Connect')
        self.connect_button.setStyleSheet("""
            QPushButton {
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton[state="disconnected"] {
                background-color: #27ae60;
                color: white;
            }
            QPushButton[state="disconnected"]:hover {
                background-color: #2ecc71;
            }
            QPushButton[state="connected"] {
                background-color: #c0392b;
                color: white;
            }
            QPushButton[state="connected"]:hover {
                background-color: #e74c3c;
            }
        """)
        self.connect_button.setProperty("state", "disconnected")
        
        # Add all to serial layout
        serial_layout.addLayout(port_layout)
        serial_layout.addLayout(baud_layout)
        serial_layout.addWidget(self.connect_button)
        
        serial_group.setLayout(serial_layout)
        left_layout.addWidget(serial_group)
        
        # Add stretch at the end to push everything up
        left_layout.addStretch()
        
        # Right side - video display and controls
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(2, 2, 2, 2)  # Reduced margins
        
        # Title - made more compact
        title_label = QLabel("BALL AND PLATE BALANCING SYSTEM")
        title_label.setStyleSheet("""
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 20px; 
            font-weight: 700;
            color: #FF0000;
            padding: 8px;
            background-color: white;
            border-radius: 3px;
            border: 1px solid #bdc3c7;
            margin-bottom: 2px;
            letter-spacing: 1px;
            text-rendering: optimizeLegibility;
            -webkit-font-smoothing: antialiased;
        """)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setMaximumHeight(45)  # Increased height slightly to accommodate new font size
        right_layout.addWidget(title_label)
        
        # Create a split layout for camera and control panel
        right_split_widget = QWidget()
        right_split_layout = QHBoxLayout(right_split_widget)
        right_split_layout.setContentsMargins(0, 0, 0, 0)
        right_split_layout.setSpacing(5)
        
        # Left side of split - Camera view
        camera_panel = QWidget()
        camera_panel.setStyleSheet("background-color: white; border: 1px solid #bdc3c7; border-radius: 3px;")
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(2, 2, 2, 2)
        camera_layout.setSpacing(0)
        
        # Camera view title
        camera_title = QLabel("Camera View")
        camera_title.setStyleSheet("""
            font-weight: bold;
            color: #3498db;
            font-size: 12px;
            padding: 3px;
            background-color: #ecf0f1;
            border-bottom: 1px solid #bdc3c7;
        """)
        camera_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        camera_layout.addWidget(camera_title)
        
        # Right side of split - Circle Trajectory Control
        circle_control_container = QWidget()
        circle_control_layout = QVBoxLayout(circle_control_container)
        circle_control_layout.setContentsMargins(0, 0, 0, 0)
        
        # Create Circle Trajectory Control group
        circle_control_group = QGroupBox("Circle Trajectory Control")
        circle_control_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #9b59b6;
                border: 1px solid #9b59b6;
                border-radius: 3px;
                margin-top: 5px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
            }
        """)
        circle_group_layout = QVBoxLayout(circle_control_group)
        circle_group_layout.setSpacing(10)
        
        # Circle parameters layout
        circle_params_layout = QGridLayout()
        circle_params_layout.setSpacing(8)
        
        # Radius parameter
        radius_label = QLabel("Radius (cm):")
        radius_label.setStyleSheet("font-weight: bold; color: black; font-size: 11px;")
        self.radius_spinbox = QDoubleSpinBox()
        self.radius_spinbox.setRange(1.0, 10.0)
        self.radius_spinbox.setSingleStep(0.5)
        self.radius_spinbox.setValue(4.0)
        self.radius_spinbox.setStyleSheet("""
            background-color: #f5eef8;
            color: black;
            border: 1px solid black;
            border-radius: 3px;
            padding: 4px;
            min-height: 22px;
        """)
        
        # Speed parameter
        speed_label = QLabel("Speed (rad/s):")
        speed_label.setStyleSheet("font-weight: bold; color: black; font-size: 11px;")
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setRange(0.1, 2.0)
        self.speed_spinbox.setSingleStep(0.1)
        self.speed_spinbox.setValue(0.2)
        self.speed_spinbox.setStyleSheet("""
            background-color: #f5eef8;
            color: black;
            border: 1px solid black;
            border-radius: 3px;
            padding: 4px;
            min-height: 22px;
        """)
        
        # Add parameters to grid layout
        circle_params_layout.addWidget(radius_label, 0, 0)
        circle_params_layout.addWidget(self.radius_spinbox, 0, 1)
        circle_params_layout.addWidget(speed_label, 1, 0)
        circle_params_layout.addWidget(self.speed_spinbox, 1, 1)
        
        # Circle control buttons
        circle_buttons_layout = QVBoxLayout()
        circle_buttons_layout.setSpacing(8)
        
        self.start_circle_button = QPushButton("Start Circle")
        self.start_circle_button.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6; color: white; border-radius: 3px;
                padding: 6px; font-weight: bold; font-size: 11px; min-height: 25px;
            }
            QPushButton:hover { background-color: #8e44ad; }
            QPushButton:pressed { background-color: #6c3483; }
        """)
        self.start_circle_button.clicked.connect(self.start_circle_trajectory)
        
        self.stop_circle_button = QPushButton("Stop Circle")
        self.stop_circle_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; color: white; border-radius: 3px;
                padding: 6px; font-weight: bold; font-size: 11px; min-height: 25px;
            }
            QPushButton:hover { background-color: #c0392b; }
            QPushButton:pressed { background-color: #a93226; }
        """)
        self.stop_circle_button.clicked.connect(self.stop_circle_trajectory)
        
        circle_buttons_layout.addWidget(self.start_circle_button)
        circle_buttons_layout.addWidget(self.stop_circle_button)
        
        # Add layouts to main circle control group layout
        circle_group_layout.addLayout(circle_params_layout)
        circle_group_layout.addLayout(circle_buttons_layout)
        circle_group_layout.addStretch() # Push controls to the top
        
        # Add group to the container
        circle_control_layout.addWidget(circle_control_group)
        
        # Square Trajectory Control group
        square_control_group = QGroupBox("Square Trajectory Control")
        square_control_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #8e44ad;
                border: 1px solid #8e44ad;
                border-radius: 3px;
                margin-top: 5px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
            }
        """)
        square_group_layout = QVBoxLayout(square_control_group)
        square_group_layout.setSpacing(10)
        
        # Square parameters layout
        square_params_layout = QGridLayout()
        square_params_layout.setSpacing(8)
        
        # Side parameter
        side_label = QLabel("Side (cm):")
        side_label.setStyleSheet("font-weight: bold; color: black; font-size: 11px;")
        self.side_spinbox = QDoubleSpinBox()
        self.side_spinbox.setRange(1.0, 10.0)
        self.side_spinbox.setSingleStep(0.5)
        self.side_spinbox.setValue(4.0)
        self.side_spinbox.setStyleSheet("""
            background-color: #f5eef8;
            color: black;
            border: 1px solid black;
            border-radius: 3px;
            padding: 4px;
            min-height: 22px;
        """)
        
        # Add parameters to grid layout
        square_params_layout.addWidget(side_label, 0, 0)
        square_params_layout.addWidget(self.side_spinbox, 0, 1)
        
        # Square control buttons
        square_buttons_layout = QVBoxLayout()
        square_buttons_layout.setSpacing(8)
        
        self.start_square_button = QPushButton("Start Square")
        self.start_square_button.setStyleSheet("""
            QPushButton {
                background-color: #8e44ad; color: white; border-radius: 3px;
                padding: 6px; font-weight: bold; font-size: 11px; min-height: 25px;
            }
            QPushButton:hover { background-color: #9b59b6; }
            QPushButton:pressed { background-color: #6c3483; }
        """)
        self.start_square_button.clicked.connect(self.start_square_trajectory)
        
        self.stop_square_button = QPushButton("Stop Square")
        self.stop_square_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; color: white; border-radius: 3px;
                padding: 6px; font-weight: bold; font-size: 11px; min-height: 25px;
            }
            QPushButton:hover { background-color: #c0392b; }
            QPushButton:pressed { background-color: #a93226; }
        """)
        self.stop_square_button.clicked.connect(self.stop_square_trajectory)
        
        square_buttons_layout.addWidget(self.start_square_button)
        square_buttons_layout.addWidget(self.stop_square_button)
        
        # Add layouts to main square control group layout
        square_group_layout.addLayout(square_params_layout)
        square_group_layout.addLayout(square_buttons_layout)
        square_group_layout.addStretch() # Push controls to the top
        
        # Add group to the container
        circle_control_layout.addWidget(square_control_group)
        
        # Status group (Moved from left panel)
        status_group = QGroupBox("Status")
        status_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                margin-top: 5px;
                padding: 8px;
                color: black;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 3px;
                color: black;
            }
        """)
        
        status_layout = QVBoxLayout(status_group)
        self.status_text = QLabel("Ready")
        # Set up status text with CSS classes for different states
        self.status_text.setStyleSheet("""
            QLabel {
                padding: 5px;
                border-radius: 3px;
                font-weight: bold;
                text-align: center;
            }
            QLabel[status="normal"] {
                color: #27ae60;
                background-color: #e8f8e8;
            }
            QLabel[status="error"] {
                color: #c0392b;
                background-color: #fadbd8;
            }
            QLabel[status="tracking"] {
                color: #27ae60;
                background-color: #e8f8e8;
                padding: 8px;
            }
            QLabel[status="no-data"] {
                color: #c0392b;
                background-color: #fadbd8;
                padding: 8px;
            }
        """)
        # Set initial status
        self.status_text.setProperty("status", "normal")
        self.status_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Helper method for efficient style updates
        def update_widget_style(widget, property_name, property_value):
            """Update widget style efficiently using properties instead of full StyleSheet"""
            widget.setProperty(property_name, property_value)
            widget.style().unpolish(widget)
            widget.style().polish(widget)
        
        # Store the helper method for use throughout the class
        self._update_widget_style = update_widget_style
        status_layout.addWidget(self.status_text)
        circle_control_layout.addWidget(status_group)
        
        # Add both panels to the split layout
        right_split_layout.addWidget(camera_panel, 3)  # Camera gets 3/4 of width
        right_split_layout.addWidget(circle_control_container, 1) # Circle controls get 1/4
        
        # Add the split layout to the main right layout
        right_layout.addWidget(right_split_widget, 1)  # stretch factor = 1

        # Create camera image display
        self.image_label = QLabel()
        self.image_label.setStyleSheet("""
            border: 1px solid #bdc3c7;
            background-color: #f5f6fa;
            border-radius: 3px;
        """)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # Set image label to match camera resolution to avoid scaling
        self.image_label.setFixedSize(640, 480)  # Match camera resolution for optimal performance
        
        # Add image label to camera panel
        camera_layout.addWidget(self.image_label, alignment=Qt.AlignmentFlag.AlignCenter)
        self.image_label.mousePressEvent = self.on_camera_view_click
        
        # Create Tracking Error Plot panel below camera
        error_plot_panel = QWidget()
        error_plot_panel.setStyleSheet("background-color: white; border: 1px solid #bdc3c7; border-radius: 3px;")
        error_plot_layout = QVBoxLayout(error_plot_panel)
        error_plot_layout.setContentsMargins(5, 5, 5, 5)
        
        # Create Matplotlib canvas for the error plot
        self.error_figure = plt.figure(figsize=(6, 2), dpi=100)
        self.ax_error = self.error_figure.add_subplot(111)
        self.error_canvas = FigureCanvas(self.error_figure)
        
        # Configure error plot aesthetics
        self.ax_error.set_title("Tracking Error (X, Y)", fontsize=10)
        self.ax_error.set_xlabel("Time (s)", fontsize=8)
        self.ax_error.set_ylabel("Error (cm)", fontsize=8)
        self.ax_error.set_ylim(-10, 10)
        self.ax_error.set_xlim(0, 30)
        self.ax_error.grid(True, linestyle='--', alpha=0.6)
        
        # Create empty line artists for error data
        self.line_error_x, = self.ax_error.plot([], [], color='r', lw=1.5, label='Error X')
        self.line_error_y, = self.ax_error.plot([], [], color='b', lw=1.5, label='Error Y')
        # self.ax_error.legend(fontsize='x-small') # Bỏ chú thích để sửa lỗi hiển thị thừa
        
        # self.error_figure.tight_layout() # Bỏ dòng này để sửa lỗi hiển thị thừa legend
        error_plot_layout.addWidget(self.error_canvas)
        
        # Add error plot panel below camera
        camera_layout.addWidget(error_plot_panel)
        
        # --- REMOVED TERMINAL AND TAB WIDGET ---
        
        # Main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, 1)  # Give more space to right panel
        
        # Window settings
        self.setMinimumSize(980, 650)  # Optimized size for compact layout
        self.center_window()
        
        # Cache background for error plot blitting
        self.error_canvas.draw()
        self.error_plot_background = self.error_canvas.copy_from_bbox(self.ax_error.bbox)

        # Connect all signals after UI is fully created
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        self.start_camera_button.clicked.connect(self.start_camera)
        self.stop_camera_button.clicked.connect(self.stop_camera)
        self.connect_button.clicked.connect(self.toggle_serial_connection)



    
    def center_window(self):
        """Center the window on the screen"""
        window_geometry = self.frameGeometry()
        screen_center = QApplication.primaryScreen().geometry().center()
        window_geometry.moveCenter(screen_center)
        self.move(window_geometry.topLeft())

    def update_status(self, message, is_error=False):
        """Update status message with color based on type"""
        if is_error:
            self._update_widget_style(self.status_text, "status", "error")
        else:
            self._update_widget_style(self.status_text, "status", "normal")
        self.status_text.setText(message)

    def show_hsv_adjustment(self):
        if self.hsv_window is None:
            self.hsv_window = HSVAdjustmentWindow(self.ball_detector)
        self.hsv_window.show()
        self.update_status("HSV adjustment window opened")

    def show_ball_values(self):
        """Create a new BallValuesWindow instance each time"""
        # Close existing window if it exists
        if self.values_window is not None:
            self.values_window.close()
            self.values_window = None
        
        # Create a fresh window instance
        self.values_window = BallValuesWindow()
        
        # Set reference to main window for target communication
        self.values_window.main_window = self
        
        # Connect the window's destroyed signal to clear the reference
        self.values_window.destroyed.connect(self._on_values_window_destroyed)
        
        # If trajectory is already running, draw it on the new window
        if self.active_trajectory == 'circle' or self.active_trajectory == 'ramping-circle':
            self.values_window.draw_circle_trajectory(self.circle_radius, self.circle_speed, self.ramp_duration)
        elif self.active_trajectory == 'square':
            self.values_window.draw_square_trajectory(
                self.square_side, self.square_dwell_duration
            )
        
        # Show the window
        self.values_window.show()
        self.update_status("Ball values graph opened - fresh start")

    def _on_values_window_destroyed(self):
        """Handle BallValuesWindow destruction - clear the reference"""
        self.values_window = None
        
    def reset_ball_values_data(self):
        """Reset ball values data when window is recreated"""
        if self.values_window is not None:
            self.values_window.reset_trajectory()
            # Clear all data buffers
            if hasattr(self.values_window, 'x_values'):
                self.values_window.x_values.clear()
            if hasattr(self.values_window, 'y_values'):
                self.values_window.y_values.clear()
            if hasattr(self.values_window, 'time_points'):
                self.values_window.time_points.clear()
            if hasattr(self.values_window, '_buffer_time'):
                self.values_window._buffer_time.clear()
            if hasattr(self.values_window, '_buffer_x'):
                self.values_window._buffer_x.clear()
            if hasattr(self.values_window, '_buffer_y'):
                self.values_window._buffer_y.clear()
            # Reset start time
            self.values_window.start_time = None

    def recalibrate_disk(self):
        """Signal the ball detector to recalibrate the disk."""
        if self.ball_detector:
            self.ball_detector.recalibrate_disk()
            self.update_status("Disk recalibration triggered.")
        else:
            self.update_status("Camera not running.", True)

    def change_ball_color(self, color_text):
        """Handle ball color selection change"""
        if self.ball_detector is None:
            return
            
        # Map display text to internal color names
        color_map = {
            "Yellow": "yellow",
            "White": "white", 
            "Red": "red"
        }
        
        if color_text in color_map:
            internal_color = color_map[color_text]
            # Set the ball color in the detector
            if hasattr(self.ball_detector, 'ball_colors'):
                self.ball_detector.current_ball_color = internal_color
                self.ball_detector.ball_hsv_lower = self.ball_detector.ball_colors[internal_color]['lower']
                self.ball_detector.ball_hsv_upper = self.ball_detector.ball_colors[internal_color]['upper']
                self.update_status(f"Ball color changed to: {color_text}")
            else:
                self.update_status("Ball detector not initialized", True)

    def change_camera(self):
        """Change the camera based on selection"""
        # Get selected camera index (0-3)
        selected_index = self.camera_combo.currentIndex()
        
        # Stop the current camera first
        self.stop_camera()
        
        # Try to connect to selected camera
        try:
            # Create new detector with selected camera
            self.ball_detector = BallObjectDetector(camera_index=selected_index)
            
            if self.ball_detector.cap is None or not self.ball_detector.cap.isOpened():
                raise Exception(f"Could not open Camera {selected_index}")
            
            # Update camera index
            self.current_camera_index = selected_index
            
            # Get camera properties
            self.current_width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.current_height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.current_fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
            
            # Update display
            self.update_resolution_display()
            
            # Start camera
            self.camera_running = True
            self.timer.start(self.timer_interval)
            self.plot_timer.start(self.plot_update_interval)
            
            # Update button states
            self.start_camera_button.setEnabled(False)
            self.stop_camera_button.setEnabled(True)
            
            # Update status
            self.update_status(f"Changed to Camera {selected_index}")
            
        except Exception as e:
            self.camera_running = False
            if self.ball_detector:
                self.ball_detector.release()
            self.ball_detector = None
            
            # Update status
            self.update_status(f"Could not connect to Camera {selected_index}", True)
            self.show_error_image(f"COULD NOT CONNECT TO CAMERA {selected_index}")
            
            # Reset button states
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)
    def try_connect_camera(self, camera_index):
        """Try to connect to camera with given index"""
        self.update_status(f"Connecting to Camera {camera_index}...")
        
        try:
            # Tạo đối tượng detector mới với camera mới
            self.ball_detector = BallObjectDetector(camera_index=camera_index)
            self.current_camera_index = camera_index
            
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap:
                self.current_width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                self.current_height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                self.current_fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
                self.update_resolution_display()
            
            self.camera_running = True
            self.update_status(f"Connected to Camera {camera_index}")
            
            self.start_camera_button.setEnabled(False)
            self.stop_camera_button.setEnabled(True)
            
            # Bắt đầu timer
            self.timer.start(self.timer_interval)
            self.plot_timer.start(self.plot_update_interval)
            
        except Exception as e:
            self.camera_running = False
            self.ball_detector = None  # Đảm bảo đối tượng detector là None nếu có lỗi
            self.update_status(f"Error connecting to camera {camera_index}", True)
            

            
            # Hiển thị ảnh thông báo lỗi
            self.show_error_image(f"COULD NOT CONNECT TO CAMERA {camera_index}")
            
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)

    def show_error_image(self, message):
        """Hiển thị ảnh thông báo lỗi"""
        if hasattr(self, 'image_label'):
            # Tạo hình ảnh thông báo
            error_img = np.ones((480, 640, 3), dtype=np.uint8) * 240
            
            # Vẽ chữ chính giữa với font to và rõ ràng
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale_main = 1.2
            thickness_main = 3
            text_color = (0, 0, 255)  # Màu đỏ
            
            # Map English messages to Vietnamese
            message_map = {
                "COULD NOT CONNECT TO CAMERA": "COULD NOT CONNECT TO CAMERA",
                "CAMERA DISCONNECTED": "CAMERA DISCONNECTED",
                "COULD NOT CONNECT TO CAMERA 0": "COULD NOT CONNECT TO CAMERA 0",
                "COULD NOT CONNECT TO CAMERA 1": "COULD NOT CONNECT TO CAMERA 1",
                "COULD NOT CONNECT TO CAMERA 2": "COULD NOT CONNECT TO CAMERA 2",
                "COULD NOT CONNECT TO CAMERA 3": "COULD NOT CONNECT TO CAMERA 3"
            }
            
            # Convert message if it exists in the map
            display_message = message_map.get(message, message)
            
            # Căn giữa dòng text chính
            text_size = cv2.getTextSize(display_message, font, font_scale_main, thickness_main)[0]
            text_x = (640 - text_size[0]) // 2
            text_y = 220
            
            # Vẽ text chính
            cv2.putText(error_img, display_message, (text_x, text_y), 
                       font, font_scale_main, text_color, thickness_main, cv2.LINE_AA)
            
            # Vẽ dòng text phụ
            sub_message = "Please check camera connection"  # Changed to Telex
            font_scale_sub = 0.8
            thickness_sub = 2
            
            # Căn giữa dòng text phụ
            sub_text_size = cv2.getTextSize(sub_message, font, font_scale_sub, thickness_sub)[0]
            sub_text_x = (640 - sub_text_size[0]) // 2
            sub_text_y = 280
            
            cv2.putText(error_img, sub_message, (sub_text_x, sub_text_y), 
                       font, font_scale_sub, text_color, thickness_sub, cv2.LINE_AA)
            
            height, width = error_img.shape[:2]
            bytes_per_line = 3 * width
            q_image = QImage(error_img.data, width, height, bytes_per_line, 
                            QImage.Format.Format_RGB888).rgbSwapped()
            
            # Since error image is 640x480 and image_label is now 640x480, no scaling needed
            self.image_label.setPixmap(QPixmap.fromImage(q_image))

    def start_camera(self):
        """Start camera capture"""
        if not self.camera_running:
            # Thử kết nối lại với camera hiện tại
            self.try_connect_camera(self.current_camera_index)

    def stop_camera(self):
        """Stop camera capture"""
        if self.camera_running:
            # Stop timers
            self.timer.stop()
            self.plot_timer.stop()
            self.camera_running = False
            
            # Giải phóng camera nếu có
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap:
                self.ball_detector.cap.release()
            
            # Update button states
            self.start_camera_button.setEnabled(True)
            self.stop_camera_button.setEnabled(False)
            
            # Hiển thị ảnh thông báo camera đã dừng
            self.show_error_image("CAMERA DISCONNECTED")
            
            self.update_status("Camera stopped")

    def update_resolution_display(self):
        """Update the resolution display labels"""
        if hasattr(self, 'resolution_label') and hasattr(self, 'fps_label'):
            if self.ball_detector and hasattr(self.ball_detector, 'cap') and self.ball_detector.cap and self.ball_detector.cap.isOpened():
                width = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = self.ball_detector.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = self.ball_detector.cap.get(cv2.CAP_PROP_FPS)
                
                self.resolution_label.setText(f"Resolution: {int(width)}x{int(height)}")
                self.fps_label.setText(f"FPS: {int(fps)}")
            else:
                # Hiển thị giá trị mặc định nếu không có camera
                self.resolution_label.setText(f"Resolution: {int(self.current_width)}x{int(self.current_height)}")
                self.fps_label.setText(f"FPS: {int(self.current_fps)}")


    def update_frame(self):
        try:
            # Update frame counters
            self.frame_count += 1
            self.frame_skip_counter += 1
            
            if self.frame_skip_counter % self.frame_skip_rate != 0:
                return
            
            # Calculate FPS
            current_time = datetime.now()
            time_diff = (current_time - self.last_fps_update).total_seconds()
            if time_diff >= self.fps_update_interval:
                self.fps = int(self.frame_count / time_diff)
                self.frame_count = 0  # Reset chỉ frame_count để tính FPS
                self.last_fps_update = current_time
                
                # Update FPS display
                self.fps_label.setText(f"FPS: {self.fps}")
            
            # Update active trajectory (this sets self.current_target)
            self.update_trajectory()
            
            if not self.camera_running or not self.ball_detector or not hasattr(self.ball_detector, 'cap') or not self.ball_detector.cap or not self.ball_detector.cap.isOpened():
                # Dừng timer nếu camera không còn hoạt động
                if self.timer.isActive():
                    self.timer.stop()
                if self.plot_timer.isActive():
                    self.plot_timer.stop()
                self.camera_running = False
                
                # Hiển thị thông báo không có camera
                self.show_error_image("COULD NOT CONNECT TO CAMERA")
                
                self.update_status("Camera not active", True)
                
                self.start_camera_button.setEnabled(True)
                self.stop_camera_button.setEnabled(False)
                return
                
            ret, frame = self.ball_detector.cap.read()
            if not ret:
                # Try to reopen camera
                try:
                    self.ball_detector.cap.release()
                    
                    # Thử kết nối lại với camera hiện tại
                    camera_idx = self.current_camera_index
                    self.ball_detector.cap = cv2.VideoCapture(camera_idx)
                    
                    if not self.ball_detector.cap.isOpened():
                        self.update_status("Could not reopen camera", True)
                        self.camera_running = False
                        self.start_camera_button.setEnabled(True)
                        self.stop_camera_button.setEnabled(False)
                        # Hiển thị thông báo
                        self.show_error_image(f"COULD NOT REOPEN CAMERA {camera_idx}")
                        return
                        
                except Exception as e:
                    self.update_status(f"Error reopening: {str(e)}", True)
                    self.camera_running = False
                    self.start_camera_button.setEnabled(True)
                    self.stop_camera_button.setEnabled(False)
                    # Hiển thị thông báo
                    self.show_error_image("ERROR REOPENING CAMERA")
                    return
                return
                
            frame = cv2.flip(frame, 1)
            
            # Calibrate disk if not already done
            if not self.ball_detector.disk_calibrated:
                if self.ball_detector.find_and_calibrate_disk(frame):
                    self.update_status("Disk calibrated successfully")
                # No else needed, it will just try again next frame
            
            # Process frame
            cnts, ball_mask = self.ball_detector._find_contours(frame)
            centers, contour_data = self.ball_detector._process_contours(cnts)
            display_frame, predicted_coordinates = self.ball_detector._draw_on_frame(frame, contour_data)
            
            # Draw target setpoint and trajectory on the camera view
            if self.ball_detector and self.ball_detector.disk_calibrated:
                transformer = self.ball_detector.transformer

                # 1. Draw current target (setpoint)
                target_x_cm, target_y_cm = self.current_target
                target_px, target_py = transformer.cm_to_image_coordinates(target_x_cm, target_y_cm)
                if target_px is not None:
                    cv2.circle(display_frame, (target_px, target_py), 8, (0, 0, 255), -1)
                    cv2.circle(display_frame, (target_px, target_py), 9, (255, 255, 255), 1)

                # 2. Draw trajectory path
                if self.active_trajectory in ['circle', 'ramping-circle']:
                    radius_cm = self.circle_radius
                    path_points_px = []
                    for angle_deg in range(0, 361, 5):
                        angle_rad = math.radians(angle_deg)
                        x_cm = radius_cm * math.cos(angle_rad)
                        y_cm = radius_cm * math.sin(angle_rad)
                        px, py = transformer.cm_to_image_coordinates(x_cm, y_cm)
                        if px is not None:
                            path_points_px.append((px, py))
                    
                    if len(path_points_px) > 1:
                        pts = np.array(path_points_px, np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(display_frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

                elif self.active_trajectory == 'square':
                    R = self.square_side
                    vertices_cm = [(R, 0), (0, R), (-R, 0), (0, -R)]
                    
                    path_points_px = []
                    for x_cm, y_cm in vertices_cm:
                        px, py = transformer.cm_to_image_coordinates(x_cm, y_cm)
                        if px is not None:
                            path_points_px.append((px, py))
                    
                    if len(path_points_px) == 4:
                        pts = np.array(path_points_px, np.int32)
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(display_frame, [pts], isClosed=True, color=(255, 0, 255), thickness=2)
            
            # Update ball position values if Kalman predicted coordinates are available
            if predicted_coordinates is not None:
                predicted_x, predicted_y = predicted_coordinates
                cm_x, cm_y = self.ball_detector.transformer.image_to_cm_coordinates(predicted_x, predicted_y)
                if cm_x is not None and cm_y is not None:
                    self.current_ball_pos = (cm_x, cm_y)
                    
                    # Calculate and store tracking error
                    if self.error_start_time is None:
                        self.error_start_time = datetime.now()
                    t = (datetime.now() - self.error_start_time).total_seconds()
                    err_x = self.current_target[0] - cm_x
                    err_y = self.current_target[1] - cm_y
                    self.error_time.append(t)
                    self.error_x.append(err_x)
                    self.error_y.append(err_y)

                    # Update position display on UI
                    if hasattr(self, 'x_value_edit') and hasattr(self, 'y_value_edit'):
                        # Format text
                        x_text = f"{cm_x:.2f} cm"
                        y_text = f"{cm_y:.2f} cm"
                        
                        # Update labels
                        self.x_value_edit.setText(x_text)
                        self.y_value_edit.setText(y_text)
                        
                        # Gửi tọa độ qua UART nếu đã đủ thời gian và đã kết nối
                        current_time = time.time()
                        if self.serial_comm.is_connected:
                            should_send = (current_time - self.last_send_time >= self.send_interval)
                            if should_send and self.serial_comm.send_ball_coordinates(cm_x, cm_y):
                                self.last_send_time = current_time
                        
                        # UI updates are handled automatically by QTimer
                    
                    if self.values_window is not None:
                        # Update trajectory when inside; freeze when outside
                        if cm_x is None or cm_y is None:
                            self.values_window.mark_ball_out()
                        else:
                            self.values_window.update_data(cm_x, cm_y)
                    
                    # Update status display if available
                    if hasattr(self, 'status_text'):
                        self.status_text.setText("Kalman Tracking")
                        self._update_widget_style(self.status_text, "status", "tracking")
            else:
                # Clear position display when no Kalman predicted coordinates available
                if hasattr(self, 'x_value_edit') and hasattr(self, 'y_value_edit'):
                    self.x_value_edit.setText("--.- cm")
                    self.y_value_edit.setText("--.- cm")
                
                # Update status - no Kalman prediction
                if hasattr(self, 'status_text'):
                    self.status_text.setText("No Kalman Data")
                    self._update_widget_style(self.status_text, "status", "no-data")
            
            # Display the image - modified to maintain aspect ratio
            if hasattr(self, 'image_label'):
                height, width, channel = display_frame.shape
                bytes_per_line = 3 * width
                q_image = QImage(display_frame.data, width, height, bytes_per_line, 
                               QImage.Format.Format_RGB888).rgbSwapped()
                
                # Direct display without scaling for optimal performance
                # Since image_label size matches camera resolution (640x480)
                pixmap = QPixmap.fromImage(q_image)
                self.image_label.setPixmap(pixmap)
        except Exception as e:
            # Update status with error
            if hasattr(self, 'status_text'):
                self.status_text.setText(f"Error: {str(e)}")
                self._update_widget_style(self.status_text, "status", "error")

    def toggle_serial_connection(self):
        """Handle serial connection/disconnection"""
        if not self.serial_comm.is_connected:
            # Try to connect
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if self.serial_comm.connect(port, baudrate=baud):
                self.connect_button.setText('Disconnect')
                self._update_widget_style(self.connect_button, "state", "connected")
                self.port_combo.setEnabled(False)
                self.baud_combo.setEnabled(False)
                self.update_status("Serial connected successfully")
            else:
                self.update_status("Failed to connect to serial port", True)
        else:
            # Disconnect
            self.serial_comm.disconnect()
            self.connect_button.setText('Connect')
            self._update_widget_style(self.connect_button, "state", "disconnected")
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.update_status("Serial disconnected")
    
    def set_target_position(self, x, y):
        """Set target position and send to STM32"""
        self.current_target = (x, y)
        
        # Update target display in main UI
        if hasattr(self, 'target_x_edit') and hasattr(self, 'target_y_edit'):
            self.target_x_edit.setText(f"{x:.1f} cm")
            self.target_y_edit.setText(f"{y:.1f} cm")
        
        # Update compact target display in camera panel - REMOVED
        # if hasattr(self, 'compact_target_x_edit') and hasattr(self, 'compact_target_y_edit'):
        #     self.compact_target_x_edit.setText(f"{x:.1f} cm")
        #     self.compact_target_y_edit.setText(f"{y:.1f} cm")
        
        # Update values window target if open
        if self.values_window is not None:
            self.values_window.set_target(x, y)
        
        # Send target to STM32 if connected
        if self.serial_comm.is_connected:
            success = self.serial_comm.send_target_coordinates(x, y)
            if success:
                self.update_status(f"Target set: X={x:.1f}cm, Y={y:.1f}cm")
            else:
                self.update_status("Failed to send target", True)
        else:
            self.update_status(f"Target set (not connected): X={x:.1f}cm, Y={y:.1f}cm")
    
    def on_camera_view_click(self, event):
        """Handle mouse click on camera view to set new target"""
        if event.button() == Qt.MouseButton.LeftButton:
            if self.ball_detector and self.ball_detector.disk_calibrated:
                click_x_px = event.pos().x()
                click_y_px = event.pos().y()

                cm_x, cm_y = self.ball_detector.transformer.image_to_cm_coordinates(click_x_px, click_y_px)

                if cm_x is not None and cm_y is not None:
                    self.stop_trajectory()
                    self.set_target_position(cm_x, cm_y)
    
    def update_serial_status(self, connected, message):
        """Update serial connection status display"""
        if connected:
            self.update_status(f"Serial: {message}")
        else:
            self.update_status(f"Serial: {message}", True)

    def calculate_circle_trajectory_position(self):
        """Calculate position on circular trajectory with initial ramp-up."""
        if self.active_trajectory == "ramping-circle":
            time_elapsed = time.time() - self.trajectory_start_time
            if time_elapsed >= self.ramp_duration:
                # Transition to running state, recalculate start time
                self.active_trajectory = "circle"
                self.trajectory_start_time = self.trajectory_start_time + self.ramp_duration
                self.update_status(f"Circle trajectory running (R={self.circle_radius}cm)")

                # The ramp has finished, so we are at (R, 0)
                return (self.circle_radius, 0.0)
            else:
                # Linear interpolation from (0,0) to (R,0)
                progress = time_elapsed / self.ramp_duration
                x = self.circle_radius * progress
                y = 0.0
                return (x, y)

        elif self.active_trajectory == "circle":
            time_elapsed = time.time() - self.trajectory_start_time
            radius = self.circle_radius
            speed = self.circle_speed
            
            angle = speed * time_elapsed
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            return (x, y)
        
        return self.current_target

    def calculate_square_trajectory_position(self):
        """Calculate position on the square trajectory with dwells and step changes."""
        R = self.square_side
        vertices = [(R, 0), (0, R), (-R, 0), (0, -R)]
        num_vertices = len(vertices)
        
        time_elapsed = time.time() - self.trajectory_start_time

        # 1. Initial ramp-up phase removed for immediate setpoint changes.

        # 2. Main loop: Dwell at each vertex
        # The duration of one phase is just the dwell time
        phase_duration = self.square_dwell_duration
        loop_duration = num_vertices * phase_duration
        
        time_in_loop = time_elapsed % loop_duration
        
        # Determine current vertex index
        vertex_index = int(time_in_loop // phase_duration)
        if vertex_index >= num_vertices:
            vertex_index = num_vertices - 1

        # The setpoint is simply the current vertex for the whole dwell duration
        return vertices[vertex_index]
        
    def start_circle_trajectory(self):
        """Start circular trajectory motion with a ramp-up phase."""
        self.stop_trajectory()

        self.circle_radius = self.radius_spinbox.value()
        self.circle_speed = self.speed_spinbox.value()

        if self.values_window:
            self.values_window.draw_circle_trajectory(self.circle_radius, self.circle_speed, self.ramp_duration)

        self.active_trajectory = "ramping-circle"
        self.trajectory_start_time = time.time()
        
        self.update_status(f"Ramping to circle (R={self.circle_radius}cm)")
    
    def start_square_trajectory(self):
        """Start square trajectory motion."""
        self.stop_trajectory()
        
        self.square_side = self.side_spinbox.value()
        
        if self.values_window:
            self.values_window.draw_square_trajectory(
                self.square_side, self.square_dwell_duration
            )
            
        self.active_trajectory = 'square'
        self.trajectory_start_time = time.time()
        
        self.update_status(f"Square trajectory started (Side={self.square_side}cm)")

    def stop_trajectory(self):
        """Stop any active trajectory."""
        if self.active_trajectory is not None:
            self.active_trajectory = None
            self.clear_error_data()
            
            if self.values_window:
                self.values_window.clear_target_trajectory()
                
            self.update_status("Trajectory stopped")
                
    def stop_circle_trajectory(self):
        """Stop circular trajectory motion"""
        self.stop_trajectory()
        
    def stop_square_trajectory(self):
        """Stop square trajectory motion"""
        self.stop_trajectory()
                
    def update_trajectory(self):
        """Update target position along any active trajectory path."""
        if self.active_trajectory is None:
            return

        if self.active_trajectory in ['ramping-circle', 'circle']:
            x, y = self.calculate_circle_trajectory_position()
        elif self.active_trajectory == 'square':
            x, y = self.calculate_square_trajectory_position()
        else:
            return

        self.set_target_position(x, y)
        
        if self.values_window is not None:
            self.values_window.update_setpoint_from_main(x, y)

    def closeEvent(self, event):
        # Stop trajectory if active
        self.stop_trajectory()
            
        # Stop camera frame timer if running
        if self.timer.isActive():
            self.timer.stop()
            
        # Disconnect serial port before closing
        if self.serial_comm:
            self.serial_comm.disconnect()
        if self.ball_detector and hasattr(self.ball_detector, 'release'):
            self.ball_detector.release()
        # Close values window if it exists
        if self.values_window:
            self.values_window.close()
            self.values_window = None
        event.accept()

    def _update_plots(self):
        """Update all Matplotlib plots using blitting for performance."""
        # Update error plot
        if not hasattr(self, 'ax_error'):
            return
        
        try:
            # Get data for plotting
            times = list(self.error_time)
            errors_x = list(self.error_x)
            errors_y = list(self.error_y)

            # Handle time axis reset: clear and redraw completely
            if times and times[-1] > 30:
                self.clear_error_data()
                
                # Update lines with empty data
                self.line_error_x.set_data([], [])
                self.line_error_y.set_data([], [])
                
                # Reset x-axis view
                self.ax_error.set_xlim(0, 30)
                
                # Force a full redraw of the canvas for a clean reset
                self.error_canvas.draw()
                
                # After redrawing the empty plot, re-cache the background for blitting
                self.error_plot_background = self.error_canvas.copy_from_bbox(self.ax_error.bbox)
                return

            # If not resetting, proceed with high-performance blitting
            self.error_canvas.restore_region(self.error_plot_background)
            
            # Update line data
            self.line_error_x.set_data(times, errors_x)
            self.line_error_y.set_data(times, errors_y)

            # Redraw artists
            self.ax_error.draw_artist(self.line_error_x)
            self.ax_error.draw_artist(self.line_error_y)

            # Blit the updated area
            self.error_canvas.blit(self.ax_error.bbox)
            self.error_canvas.flush_events()

        except Exception as e:
            # This can happen if the plot is closed while the timer is running
            pass

    def clear_error_data(self):
        """Clears data for the tracking error plot."""
        self.error_time.clear()
        self.error_x.clear()
        self.error_y.clear()
        self.error_start_time = None





