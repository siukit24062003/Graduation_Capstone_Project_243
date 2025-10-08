# Multiple Target Control Feature

## Tính năng điều khiển nhiều target

Tính năng này cho phép bạn điều khiển vị trí target của quả bóng bằng cách:

### 1. Click chuột trên đồ thị disk (Ball Values Window)
- Mở Ball Values window từ main GUI
- Click vào bất kỳ vị trí nào trong vòng tròn disk
- Hệ thống sẽ tự động gửi tọa độ target mới tới STM32

### 2. Sử dụng preset buttons
- Center: (0, 0)
- Up: (0, 3)
- Down: (0, -3)
- Left: (-3, 0)
- Right: (3, 0)

### 3. Giao thức truyền dữ liệu
#### Ball coordinates (6 bytes):
```
@<x_int8><y_int8>X<checksum_2bytes>
```

#### Target coordinates (7 bytes):
```
@T<x_int8><y_int8>X<checksum_2bytes>
```

### 4. Cách hoạt động
1. GUI gửi target coordinates qua UART1 với format đặc biệt
2. STM32 nhận và xử lý target frame
3. PID controller được cập nhật với target mới
4. Quả bóng sẽ được điều khiển đến vị trí target mới

### 5. Các thay đổi trong code

#### Python GUI:
- `serial_communication.py`: Thêm `send_target_coordinates()`
- `gui.py`: Thêm target controls và mouse click handling
- `BallValuesWindow`: Thêm target visualization và click handling

#### STM32:
- `uart_handler.h/c`: Thêm target frame processing
- `main.c`: Thêm target frame checking trong main loop
- `pid_controller.h/c`: Sử dụng existing `PIDSetTarget()` function

### 6. Test tính năng
1. Compile và flash STM32 code
2. Chạy Python GUI
3. Kết nối UART
4. Mở Ball Values window
5. Click vào các vị trí khác nhau trên disk
6. Hoặc dùng preset buttons trên main GUI

### 7. Debug
- Target coordinates sẽ hiển thị trong Terminal tab
- Status sẽ update khi target được gửi thành công
- STM32 sẽ cập nhật PID setpoint tự động
