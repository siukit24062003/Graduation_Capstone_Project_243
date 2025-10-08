import serial
import struct
import time
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal

class SerialCommunication(QObject):
    connection_status = pyqtSignal(bool, str)
    data_received = pyqtSignal(str)
    
    # Protocol constants
    START_BIT = 0x40
    STOP_BIT = 0x58
    
    # Coordinate constants
    COORDINATE_SCALE = 10
    MAX_COORDINATE = 9.5
    
    # Angle constants
    ANGLE_SCALE = 100
    MAX_ANGLE = 30.0
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.port_name = None
        
        self.default_settings = {
            'baudrate': 115200,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_NONE,
            'stopbits': serial.STOPBITS_ONE,
            'timeout': 0.1,
            'write_timeout': 0.1
        }
        
        self.receive_buffer = bytearray()
    
    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFFFF
    
    def pack_data_frame(self, parameter_data):
        if isinstance(parameter_data, str):
            parameter_data = parameter_data.encode('ascii')
        elif not isinstance(parameter_data, (bytes, bytearray)):
            parameter_data = bytes(parameter_data)
        
        frame = bytearray()
        frame.append(self.START_BIT)
        frame.extend(parameter_data)
        frame.append(self.STOP_BIT)
        
        checksum_data = frame[:]
        checksum = self.calculate_checksum(checksum_data)
        
        frame.extend(struct.pack('<H', checksum))
        
        return frame
    
    def send_packed_frame(self, parameter_data):
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            packed_frame = self.pack_data_frame(parameter_data)
            self.serial_port.write(packed_frame)
            self.serial_port.flush()
            return True
        except Exception as e:
            self.connection_status.emit(False, f"Send error: {str(e)}")
            return False
    
    def send_ball_coordinates(self, x, y):
        try:
            if abs(x) > self.MAX_COORDINATE or abs(y) > self.MAX_COORDINATE:
                return False
            
            x_int = int(round(x * self.COORDINATE_SCALE))
            y_int = int(round(y * self.COORDINATE_SCALE))
            
            coord_data = struct.pack('<bb', x_int, y_int)
            
            return self.send_packed_frame(coord_data)
            
        except Exception as e:
            return False
    
    def send_target_coordinates(self, x, y):
        try:
            if abs(x) > self.MAX_COORDINATE or abs(y) > self.MAX_COORDINATE:
                return False
            
            x_int = int(round(x * self.COORDINATE_SCALE))
            y_int = int(round(y * self.COORDINATE_SCALE))
            
            target_data = struct.pack('<cbb', b'T', x_int, y_int)
            
            return self.send_packed_frame(target_data)
            
        except Exception as e:
            return False
    
    def send_string_frame(self, message):
        try:
            return self.send_packed_frame(message)
        except Exception as e:
            return False
    
    def unpack_data_frame(self, frame):
        try:
            if len(frame) < 4:
                return False, None, "Frame too short"
            
            if frame[0] != self.START_BIT:
                return False, None, f"Invalid start bit. Expected: {self.START_BIT}, Got: {frame[0]}"
            
            stop_index = -1
            for i in range(1, len(frame) - 2):
                if frame[i] == self.STOP_BIT:
                    stop_index = i
                    break
            
            if stop_index == -1:
                return False, None, "Stop bit not found"
            
            if len(frame) < stop_index + 3:
                return False, None, "Frame too short for checksum"
            
            parameter_data = frame[1:stop_index]
            
            received_checksum = struct.unpack('<H', frame[stop_index+1:stop_index+3])[0]
            
            checksum_data = frame[:stop_index+1]
            calculated_checksum = self.calculate_checksum(checksum_data)
            
            if received_checksum != calculated_checksum:
                return False, None, f"Checksum mismatch. Expected: {calculated_checksum}, Got: {received_checksum}"
            
            return True, bytes(parameter_data), "Success"
            
        except Exception as e:
            return False, None, f"Unpack error: {str(e)}"
    
    def read_and_process_frames(self):
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return []
        
        valid_frames = []
        
        try:
            if self.serial_port.in_waiting > 0:
                new_data = self.serial_port.read(self.serial_port.in_waiting)
                self.receive_buffer.extend(new_data)
            
            while len(self.receive_buffer) >= 4:
                start_index = -1
                for i in range(len(self.receive_buffer)):
                    if self.receive_buffer[i] == self.START_BIT:
                        start_index = i
                        break
                
                if start_index == -1:
                    self.receive_buffer.clear()
                    break
                
                if start_index > 0:
                    self.receive_buffer = self.receive_buffer[start_index:]
                
                stop_index = -1
                for i in range(1, len(self.receive_buffer) - 2):
                    if self.receive_buffer[i] == self.STOP_BIT:
                        stop_index = i
                        break
                
                if stop_index == -1:
                    break
                
                frame_end = stop_index + 3
                if len(self.receive_buffer) < frame_end:
                    break
                
                frame = self.receive_buffer[:frame_end]
                self.receive_buffer = self.receive_buffer[frame_end:]
                
                success, parameter_data, error_msg = self.unpack_data_frame(frame)
                if success:
                    decoded_data = self.decode_parameter_data(parameter_data)
                    if decoded_data:
                        valid_frames.append(decoded_data)
                        self.data_received.emit(decoded_data)
                    
        except Exception as e:
            pass
        
        return valid_frames

    def decode_parameter_data(self, parameter_data):
        if len(parameter_data) < 1:
            return None
        
        try:
            if len(parameter_data) == 2:
                x_int, y_int = struct.unpack('<bb', parameter_data)
                x = x_int / self.COORDINATE_SCALE
                y = y_int / self.COORDINATE_SCALE
                return f"Ball Position: X={x:.1f}cm, Y={y:.1f}cm"
                
            elif len(parameter_data) == 3 and parameter_data[0:1] == b'T':
                _, x_int, y_int = struct.unpack('<cbb', parameter_data)
                x = x_int / self.COORDINATE_SCALE
                y = y_int / self.COORDINATE_SCALE
                return f"Target Set: X={x:.1f}cm, Y={y:.1f}cm"
                
            elif len(parameter_data) == 4:
                theta_int, phi_int = struct.unpack('<hh', parameter_data)
                theta = theta_int / self.ANGLE_SCALE
                phi = phi_int / self.ANGLE_SCALE
                return f"Angles: θ={theta:.2f}°, φ={phi:.2f}°"
                
            elif len(parameter_data) == 10:
                theta_int, phi_int, pwm_servo1, pwm_servo2, pwm_servo3 = struct.unpack('<hhHHH', parameter_data)
                theta = theta_int / self.ANGLE_SCALE
                phi = phi_int / self.ANGLE_SCALE
                return f"Angles: θ={theta:.2f}°, φ={phi:.2f}° | PWM: S1={pwm_servo1}, S2={pwm_servo2}, S3={pwm_servo3}"
            
            return None
            
        except Exception as e:
            return None


   
    def get_available_ports(self):
        ports = []
        try:
            available_ports = serial.tools.list_ports.comports()
            
            for port in available_ports:
                ports.append(port.device)
            
        except Exception as e:
            pass
            
        return sorted(ports)
    
    def connect(self, port_name, baudrate=115200, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE):
        try:
            if self.is_connected:
                self.disconnect()
                
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=self.default_settings['timeout'],
                write_timeout=self.default_settings['write_timeout']
            )
            
            if self.serial_port.is_open:
                self.is_connected = True
                self.port_name = port_name
                self.connection_status.emit(True, f"Connected to {port_name}")
                return True
            else:
                self.connection_status.emit(False, f"Failed to open {port_name}")
                return False
                
        except Exception as e:
            self.connection_status.emit(False, f"Connection error: {str(e)}")
            return False
    
    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                pass
            finally:
                self.is_connected = False
                self.port_name = None
                self.receive_buffer.clear()
                self.connection_status.emit(False, "Disconnected")
    
    def read_data(self):
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return ""
        
        try:
            if self.serial_port.in_waiting > 0:
                raw_data = self.serial_port.read(self.serial_port.in_waiting)
                try:
                    return raw_data.decode('utf-8')
                except UnicodeDecodeError:
                    return raw_data.decode('latin-1', errors='replace')
            return ""
        except Exception as e:
            return ""
    
    def __del__(self):
        self.disconnect() 