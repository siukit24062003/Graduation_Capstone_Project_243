#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stm32f4xx_hal.h"
#include "ball_position.h"
#include <stdbool.h>

// Kich thuoc bo dem UART
#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 128

// Ky tu ket thuc dong
#define UART_CR '\r'
#define UART_LF '\n'

// Frame protocol constants (shared by Camera and legacy PSO)
#define FRAME_START_BIT 0x40    // '@' character (64 decimal)
#define FRAME_STOP_BIT 0x58     // 'X' character (88 decimal)

// Legacy PSO constants - kept for compilation compatibility
#define PSO_FRAME_BUFFER_SIZE 200
#define PSO_COMMAND_MAX_LENGTH 100

// Camera data frame structure
#define CAMERA_FRAME_START FRAME_START_BIT
#define CAMERA_FRAME_STOP  FRAME_STOP_BIT
#define CAMERA_FRAME_SIZE  6       // 1+2+1+2 bytes: @<x_int8><y_int8>X<checksum_2bytes>
#define TARGET_FRAME_SIZE  7       // 1+3+1+2 bytes: @T<x_int8><y_int8>X<checksum_2bytes>
#define COORDINATE_SCALE   10      // Same as camera app
#define TARGET_IDENTIFIER  'T'     // Target frame identifier



typedef struct {
    float x;                   // X coordinate in cm
    float y;                   // Y coordinate in cm
    bool valid;               // Data validity flag
    uint32_t timestamp;       // Timestamp when received
} CameraData;

// Target position data structure
typedef struct {
    float x;                   // Target X coordinate in cm
    float y;                   // Target Y coordinate in cm
    bool valid;               // Data validity flag
    uint32_t timestamp;       // Timestamp when received
    bool updated;             // Flag indicating new target received
} TargetData;

// Camera data buffer for multiple values
#define CAMERA_BUFFER_SIZE 5
typedef struct {
    CameraData data[CAMERA_BUFFER_SIZE];
    uint8_t head;             // Write index
    uint8_t count;            // Number of valid entries
    uint32_t last_received;   // Last receive timestamp
    bool timeout;             // Timeout flag
} CameraDataBuffer;

// Cau truc UART Handler
typedef struct {
    UART_HandleTypeDef *huart;       // UART handle
    DMA_HandleTypeDef *hdma_rx;      // DMA handle RX

    uint8_t rxBuffer[UART_RX_BUFFER_SIZE];          // Vong DMA buffer
    uint8_t rxProcessBuffer[UART_RX_BUFFER_SIZE];   // Dem xu ly
    uint8_t rxLineBuffer[UART_RX_BUFFER_SIZE];      // Dem xu ly dong
    uint16_t rxLineSize;         // Do dai dong nhan duoc
    uint16_t oldPos;             // Vi tri cu trong buffer DMA
    uint16_t newPos;             // Vi tri moi
    bool lineReady;              // Co dong hoan chinh chua

    uint8_t txBuffer[UART_TX_BUFFER_SIZE]; // Bo dem truyen
    bool txBusy;                 // Co dang truyen khong

    // Camera specific data (for UART1)
    CameraDataBuffer camera_buffer;
    uint8_t camera_frame_buffer[CAMERA_FRAME_SIZE];
    uint8_t camera_frame_pos;
    bool is_camera_uart;         // Flag to identify camera UART
    
    // Target position data (for UART1)
    TargetData target_data;
    uint8_t target_frame_buffer[TARGET_FRAME_SIZE];
    uint8_t target_frame_pos;
    
    // Debug specific data (for UART2) 
    bool is_debug_uart;          // Flag to identify debug UART
    
    // Legacy PSO data - kept for compilation compatibility but not used
    uint8_t pso_frame_buffer[PSO_FRAME_BUFFER_SIZE];
    uint16_t pso_frame_pos;
    bool pso_frame_start_found;
    bool is_pso_uart;            // Legacy flag
} UartHandler;

// Nguyen mau ham
void UartInit(UartHandler *handler, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);

void UartStartReceive(UartHandler *handler);
bool UartProcessData(UartHandler *handler, BallPosition *ball);

void UartTransmit(UartHandler *handler, const char *data, uint16_t size);
void UartTransmitString(UartHandler *handler, const char *str);
void UartDmaRxCpltCallback(UartHandler *handler);
void UartDmaTxCpltCallback(UartHandler *handler);
void UartIdleCallback(UartHandler *handler);
void UartProcessBuffer(UartHandler *handler, uint16_t len);
bool UartCheckDMA(UartHandler *handler);

// New function prototypes for camera data
bool UartProcessCameraFrame(UartHandler *handler);
bool UartGetLatestCameraData(UartHandler *handler, CameraData *data);
bool UartGetAverageCameraData(UartHandler *handler, CameraData *data, uint8_t samples);
void UartCameraTimeout(UartHandler *handler, uint32_t timeout_ms);
uint16_t CalculateChecksum(uint8_t *data, uint16_t length);
bool ValidateCameraData(float x, float y);

// Target position function prototypes
bool UartProcessTargetFrame(UartHandler *handler);
bool UartGetLatestTargetData(UartHandler *handler, TargetData *data);
bool ValidateTargetData(float x, float y);

// PSO stub functions - functionality removed but declarations kept for compatibility
bool UartProcessPsoFrame(UartHandler *handler);
bool UartValidatePsoFrame(UartHandler *handler);
bool UartPackPsoResponse(const char *response, uint8_t *output_buffer, uint16_t *output_length);
void UartSendPsoResponse(UartHandler *handler, const char *response);
bool UartProcessPsoOnboardCommands(UartHandler *handler);



#endif /* UART_HANDLER_H */
