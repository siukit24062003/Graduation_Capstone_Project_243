#include "uart_handler.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Include main.h to get macro definitions
#include "main.h"



// Khoi tao UART handler
void UartInit(UartHandler *handler, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx) {
    handler->huart = huart;
    handler->hdma_rx = hdma_rx;

    memset(handler->rxBuffer, 0, UART_RX_BUFFER_SIZE);
    memset(handler->rxProcessBuffer, 0, UART_RX_BUFFER_SIZE);
    memset(handler->rxLineBuffer, 0, UART_RX_BUFFER_SIZE);
    handler->rxLineSize = 0;
    handler->oldPos = 0;
    handler->newPos = 0;
    handler->lineReady = false;
    handler->txBusy = false;

    // Initialize UART type flags
    handler->is_camera_uart = (huart->Instance == USART1); // UART1 for camera
    handler->is_debug_uart = (huart->Instance == USART2);  // UART2 for debug
    handler->is_pso_uart = (huart->Instance == USART2);    // Legacy PSO flag
    
    // Initialize camera specific data
    if (handler->is_camera_uart) {
        memset(&handler->camera_buffer, 0, sizeof(CameraDataBuffer));
        memset(handler->camera_frame_buffer, 0, CAMERA_FRAME_SIZE);
        handler->camera_frame_pos = 0;
        
        // Initialize target data
        memset(&handler->target_data, 0, sizeof(TargetData));
        memset(handler->target_frame_buffer, 0, TARGET_FRAME_SIZE);
        handler->target_frame_pos = 0;
    }
    
    // Initialize legacy PSO data (for compilation compatibility)
    if (handler->is_pso_uart) {
        memset(handler->pso_frame_buffer, 0, PSO_FRAME_BUFFER_SIZE);
        handler->pso_frame_pos = 0;
        handler->pso_frame_start_found = false;
    }
    


    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // Ngat khi IDLE
}



// Bat dau nhan DMA vong
void UartStartReceive(UartHandler *handler) {
    HAL_UART_Receive_DMA(handler->huart, handler->rxBuffer, UART_RX_BUFFER_SIZE);
}

// Xu ly chuoi du lieu nhan duoc -> chi cap nhat vi tri bong (manual input only)
bool UartProcessData(UartHandler *handler, BallPosition *ball) {
    if (!handler->lineReady) return false;
    handler->rxLineBuffer[handler->rxLineSize] = '\0';
    
    char *line = (char *)handler->rxLineBuffer;
    
    // Only process coordinate data: "x,y" format
    char *comma = strchr(line, ',');
    if (comma == NULL) {
        handler->lineReady = false;
        handler->rxLineSize = 0;
        return false;
    }
    *comma = '\0';
    float x = atof(line);
    float y = atof(comma + 1);
    BallPositionUpdate(ball, x, y);
    handler->lineReady = false;
    handler->rxLineSize = 0;
    
    return true;
}



// Truyen du lieu bang DMA
void UartTransmit(UartHandler *handler, const char *data, uint16_t size) {
    uint32_t timeout = HAL_GetTick() + 100;

    while (handler->txBusy) {
        if (HAL_GetTick() > timeout) {
            HAL_UART_AbortTransmit(handler->huart);
            handler->txBusy = false;
            break;
        }
        HAL_Delay(1);
    }

    uint16_t copySize = (size < UART_TX_BUFFER_SIZE) ? size : UART_TX_BUFFER_SIZE - 1;
    memcpy(handler->txBuffer, data, copySize);
    handler->txBusy = true;

    // Try DMA transmission first
    HAL_StatusTypeDef dma_status = HAL_UART_Transmit_DMA(handler->huart, handler->txBuffer, copySize);

    // If DMA fails, use blocking transmission as fallback
    if (dma_status != HAL_OK) {
        handler->txBusy = false;  // Reset busy flag
        HAL_UART_Transmit(handler->huart, handler->txBuffer, copySize, 1000);
    }
}

// Truyen chuoi null-terminated
void UartTransmitString(UartHandler *handler, const char *str) {
    UartTransmit(handler, str, strlen(str));
}

// Callback khi DMA RX hoan tat (goi tu HAL_UART_RxCpltCallback)
void UartDmaRxCpltCallback(UartHandler *handler) {
    UartCheckDMA(handler);
}

// Callback khi DMA TX hoan tat (goi tu HAL_UART_TxCpltCallback)
void UartDmaTxCpltCallback(UartHandler *handler) {
    handler->txBusy = false;
}

// Kiem tra DMA buffer xem co du lieu moi khong
bool UartCheckDMA(UartHandler *handler) {
    handler->newPos = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(handler->hdma_rx);

    if (handler->newPos != handler->oldPos) {
        // For camera UART (UART1), keep existing copy-then-process behavior
        if (handler->is_camera_uart) {
            if (handler->newPos > handler->oldPos) {
                memcpy(handler->rxProcessBuffer, &handler->rxBuffer[handler->oldPos], handler->newPos - handler->oldPos);
            } else {
                memcpy(handler->rxProcessBuffer, &handler->rxBuffer[handler->oldPos], UART_RX_BUFFER_SIZE - handler->oldPos);
                memcpy(&handler->rxProcessBuffer[UART_RX_BUFFER_SIZE - handler->oldPos], handler->rxBuffer, handler->newPos);
            }

            // Process the newly received chunk of data
            UartProcessBuffer(handler, handler->newPos > handler->oldPos ? (handler->newPos - handler->oldPos) : (UART_RX_BUFFER_SIZE - handler->oldPos + handler->newPos));

            // Advance oldPos only for camera UART
            handler->oldPos = handler->newPos;
            return true; // Indicate that new data was found
        }

        // For debug UART (UART2), do not consume bytes here.
        // Leave rxBuffer indices intact so UartProcessPsoFrame() can parse
        // frames directly from the ring buffer without losing data.
        return true;
    }
    return false; // No new data
}

// Xu ly du lieu nhan de lay chuoi ket thuc bang CR hoac LF
void UartProcessBuffer(UartHandler *handler, uint16_t len) {
    if (handler->is_camera_uart) {
        // Process binary camera and target frames
        for (uint16_t i = 0; i < len; i++) {
            uint8_t byte = handler->rxProcessBuffer[i];
            
            if (byte == CAMERA_FRAME_START) {
                // Start new frame - could be camera or target
                handler->camera_frame_pos = 0;
                handler->target_frame_pos = 0;
                handler->camera_frame_buffer[handler->camera_frame_pos++] = byte;
                handler->target_frame_buffer[handler->target_frame_pos++] = byte;
            } else if (handler->camera_frame_pos > 0 || handler->target_frame_pos > 0) {
                // Check if this is a target frame (second byte is 'T')
                if ((handler->camera_frame_pos == 1 || handler->target_frame_pos == 1) && byte == TARGET_IDENTIFIER) {
                    // This is a target frame
                    handler->camera_frame_pos = 0; // Reset camera frame
                    if (handler->target_frame_pos < TARGET_FRAME_SIZE) {
                        handler->target_frame_buffer[handler->target_frame_pos++] = byte;
                    }
                } else if (handler->target_frame_pos > 1) {
                    // Continue building target frame
                    if (handler->target_frame_pos < TARGET_FRAME_SIZE) {
                        handler->target_frame_buffer[handler->target_frame_pos++] = byte;
                    }
                    
                    // Check if target frame is complete
                    if (handler->target_frame_pos == TARGET_FRAME_SIZE) {
                        UartProcessTargetFrame(handler);
                        handler->target_frame_pos = 0; // Reset for next frame
                    }
                } else {
                    // Continue building camera frame
                    if (handler->camera_frame_pos < CAMERA_FRAME_SIZE) {
                        handler->camera_frame_buffer[handler->camera_frame_pos++] = byte;
                    }
                    
                    // Check if camera frame is complete
                    if (handler->camera_frame_pos == CAMERA_FRAME_SIZE) {
                        UartProcessCameraFrame(handler);
                        handler->camera_frame_pos = 0; // Reset for next frame
                    }
                }
            }
        }
    } else if (handler->is_debug_uart) {
        // Text-based command processing for UART2 (Debug)
        for (uint16_t i = 0; i < len; i++) {
            char c = handler->rxProcessBuffer[i];
            if (c == '\r' || c == '\n') { // End of command
                if (handler->rxLineSize > 0) {
                    handler->rxLineBuffer[handler->rxLineSize] = '\0'; // Null-terminate
                    handler->lineReady = true; // Mark line as ready for processing
                    handler->rxLineSize = 0; // Reset for next line
                }
            } else {
                if (handler->rxLineSize < (UART_RX_BUFFER_SIZE - 1)) {
                    handler->rxLineBuffer[handler->rxLineSize++] = c;
                }
            }
        }
    }
    memset(handler->rxProcessBuffer, 0, len);
}

// Duoc goi khi co ngat UART IDLE line
void UartIdleCallback(UartHandler *handler) {
    __HAL_UART_CLEAR_IDLEFLAG(handler->huart);
    UartCheckDMA(handler);
}

// Calculate 16-bit checksum
uint16_t CalculateChecksum(uint8_t *data, uint16_t length) {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Validate camera data range
bool ValidateCameraData(float x, float y) {
    const float MAX_COORDINATE = 9.5f;  // ±9.5cm range
    return (x >= -MAX_COORDINATE && x <= MAX_COORDINATE &&
            y >= -MAX_COORDINATE && y <= MAX_COORDINATE);
}

// Process camera binary frame
bool UartProcessCameraFrame(UartHandler *handler) {
    if (!handler->is_camera_uart) return false;

    uint8_t *frame = handler->camera_frame_buffer;

    // Verify frame structure: @<x_int8><y_int8>X<checksum_2bytes>
    if (frame[0] != CAMERA_FRAME_START || frame[3] != CAMERA_FRAME_STOP) {
        return false;
    }

    // Calculate checksum từ START đến STOP (4 bytes đầu) - khớp với Python
    uint16_t calculated_checksum = CalculateChecksum(frame, 4);
    uint16_t received_checksum = frame[4] | ((uint16_t)frame[5] << 8); // Little endian
    
    if (calculated_checksum != received_checksum) {
        // Checksum mismatch - frame corrupted
        return false;
    }

    // Extract coordinates (int8 format)
    int8_t x_int = (int8_t)frame[1];
    int8_t y_int = (int8_t)frame[2];
    
    // Convert to float (divide by scale factor)
    float x = (float)x_int / COORDINATE_SCALE;
    float y = (float)y_int / COORDINATE_SCALE;

    // Validate range
    if (!ValidateCameraData(x, y)) {
        return false;
    }

    // Store in buffer
    CameraDataBuffer *buffer = &handler->camera_buffer;
    buffer->data[buffer->head].x = x;
    buffer->data[buffer->head].y = y;
    buffer->data[buffer->head].valid = true;
    buffer->data[buffer->head].timestamp = HAL_GetTick();

    buffer->head = (buffer->head + 1) % CAMERA_BUFFER_SIZE;
    if (buffer->count < CAMERA_BUFFER_SIZE) {
        buffer->count++;
    }
    buffer->last_received = HAL_GetTick();
    buffer->timeout = false;

    return true;
}

// Get latest camera data
bool UartGetLatestCameraData(UartHandler *handler, CameraData *data) {
    if (!handler->is_camera_uart || handler->camera_buffer.count == 0) {
        return false;
    }

    // Get most recent data
    uint8_t latest_index = (handler->camera_buffer.head - 1 + CAMERA_BUFFER_SIZE) % CAMERA_BUFFER_SIZE;
    *data = handler->camera_buffer.data[latest_index];
    return data->valid;
}

// Get average camera data from multiple samples
bool UartGetAverageCameraData(UartHandler *handler, CameraData *data, uint8_t samples) {
    if (!handler->is_camera_uart || handler->camera_buffer.count == 0) {
        return false;
    }

    uint8_t count = (samples > handler->camera_buffer.count) ? handler->camera_buffer.count : samples;
    float sum_x = 0, sum_y = 0;
    uint32_t latest_timestamp = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t index = (handler->camera_buffer.head - 1 - i + CAMERA_BUFFER_SIZE) % CAMERA_BUFFER_SIZE;
        sum_x += handler->camera_buffer.data[index].x;
        sum_y += handler->camera_buffer.data[index].y;
        if (handler->camera_buffer.data[index].timestamp > latest_timestamp) {
            latest_timestamp = handler->camera_buffer.data[index].timestamp;
        }
    }

    data->x = sum_x / count;
    data->y = sum_y / count;
    data->valid = true;
    data->timestamp = latest_timestamp;

    return true;
}

// Check camera timeout
void UartCameraTimeout(UartHandler *handler, uint32_t timeout_ms) {
    if (!handler->is_camera_uart) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - handler->camera_buffer.last_received > timeout_ms) {
        handler->camera_buffer.timeout = true;
        // Clear buffer on timeout
        handler->camera_buffer.count = 0;
        handler->camera_buffer.head = 0;
    }
}

// Stub function - PSO functionality removed  
bool UartProcessPsoFrame(UartHandler *handler) {
    return false;
}

// Stub function - PSO functionality removed
bool UartValidatePsoFrame(UartHandler *handler) {
    return false;
}

// Stub function - PSO functionality removed
bool UartPackPsoResponse(const char *response, uint8_t *output_buffer, uint16_t *output_length) {
    return false;
}

// Stub function - PSO functionality removed
void UartSendPsoResponse(UartHandler *handler, const char *response) {
    // Do nothing - PSO functionality removed
}

// Process target binary frame
bool UartProcessTargetFrame(UartHandler *handler) {
    if (!handler->is_camera_uart) return false;

    uint8_t *frame = handler->target_frame_buffer;

    // Verify frame structure: @T<x_int8><y_int8>X<checksum_2bytes>
    if (frame[0] != CAMERA_FRAME_START || frame[1] != TARGET_IDENTIFIER || frame[4] != CAMERA_FRAME_STOP) {
        return false;
    }

    // Calculate checksum từ START đến STOP (5 bytes đầu) - khớp với Python
    uint16_t calculated_checksum = CalculateChecksum(frame, 5);
    uint16_t received_checksum = frame[5] | ((uint16_t)frame[6] << 8); // Little endian
    
    if (calculated_checksum != received_checksum) {
        // Checksum mismatch - frame corrupted
        return false;
    }

    // Extract coordinates (int8 format)
    int8_t x_int = (int8_t)frame[2];
    int8_t y_int = (int8_t)frame[3];
    
    // Convert to float (divide by scale factor)
    float x = (float)x_int / COORDINATE_SCALE;
    float y = (float)y_int / COORDINATE_SCALE;

    // Validate range
    if (!ValidateTargetData(x, y)) {
        return false;
    }

    // Store target data
    handler->target_data.x = x;
    handler->target_data.y = y;
    handler->target_data.valid = true;
    handler->target_data.timestamp = HAL_GetTick();
    handler->target_data.updated = true; // Mark as newly updated

    return true;
}

// Get latest target data
bool UartGetLatestTargetData(UartHandler *handler, TargetData *data) {
    if (!handler->is_camera_uart || !handler->target_data.valid) {
        return false;
    }

    *data = handler->target_data;
    handler->target_data.updated = false; // Clear update flag after reading
    return true;
}

// Validate target data range
bool ValidateTargetData(float x, float y) {
    const float MAX_COORDINATE = 9.5f;  // ±9.5cm range
    return (x >= -MAX_COORDINATE && x <= MAX_COORDINATE &&
            y >= -MAX_COORDINATE && y <= MAX_COORDINATE);
	}

// Stub function - PSO functionality removed
bool UartProcessPsoOnboardCommands(UartHandler *handler) {
    return false;
}
