/*
 * l6470_driver.h
 *
 *  Created on: Jun 21, 2025
 *      Author: digia
 */

#ifndef INC_L6470_DRIVER_H_
#define INC_L6470_DRIVER_H_

#include "main.h"
#include <stdbool.h>
/* L6470 Commands */
#define L6470_NOP                   0x00
#define L6470_SET_PARAM             0x00
#define L6470_GET_PARAM             0x20
#define L6470_GOTO                  0x60
#define L6470_RESET_POS             0xD8
#define L6470_RESET_DEVICE          0xC0
#define L6470_GET_STATUS            0xD0

/* L6470 Parameters */
#define L6470_ABS_POS               0x01
#define L6470_SPEED                 0x04
#define L6470_ACC                   0x05
#define L6470_DEC                   0x06
#define L6470_MAX_SPEED             0x07
#define L6470_MIN_SPEED             0x08
#define L6470_STEP_MODE             0x16
#define L6470_STATUS                0x19

/* Step modes */
#define L6470_STEP_MODE_FULL        0x00
#define L6470_STEP_MODE_HALF        0x01
#define L6470_STEP_MODE_1_4         0x02
#define L6470_STEP_MODE_1_8         0x03
#define L6470_STEP_MODE_1_16        0x04
#define L6470_STEP_MODE_1_32        0x05
#define L6470_STEP_MODE_1_64        0x06
#define L6470_STEP_MODE_1_128       0x07

/* Status Register Bit Masks */
#define L6470_STATUS_BUSY           0x0002
#define L6470_STATUS_MOT_STATUS     0x0060
#define L6470_STATUS_NOTPERF_CMD    0x0080
#define L6470_STATUS_WRONG_CMD      0x0100

// DMA Buffer sizes
#define DMA_BUFFER_SIZE     8

/* Configuration Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *busy_port;
    uint16_t busy_pin;
    //DMA section
    uint8_t tx_buffer[DMA_BUFFER_SIZE];
		uint8_t rx_buffer[DMA_BUFFER_SIZE];
		volatile bool transfer_complete;
		volatile bool transfer_error;
} L6470_Config_t;

/* Driver Handle Structure */
typedef struct {
    L6470_Config_t config;
    uint8_t step_mode;
    uint32_t max_speed;
    uint32_t min_speed;
    uint32_t acceleration;
    uint32_t deceleration;
} L6470_Handle_t;

/* Error Codes */
typedef enum {
    L6470_OK = 0,
    L6470_ERROR,
    L6470_BUSY,
    L6470_TIMEOUT,
    L6470_WRONG_CMD,
    L6470_NOTPERF_CMD
} L6470_Status_t;

/* Function Prototypes */
L6470_Status_t L6470_Init(L6470_Handle_t *hmotor, L6470_Config_t *config, bool with_DMA);
L6470_Status_t L6470_Reset(L6470_Handle_t *hmotor);
L6470_Status_t L6470_SetParam(L6470_Handle_t *hmotor, uint8_t param, uint32_t value);
uint32_t L6470_GetParam(L6470_Handle_t *hmotor, uint8_t param);
void L6470_GoTo(L6470_Handle_t *hmotor, uint32_t position);
uint16_t L6470_GetStatus(L6470_Handle_t *hmotor);
int32_t L6470_GetPosition(L6470_Handle_t *hmotor);
L6470_Status_t L6470_ResetPosition(L6470_Handle_t *hmotor, bool with_DMA);
uint8_t L6470_IsBusy(L6470_Handle_t *hmotor);
L6470_Status_t L6470_WaitWhileBusy(L6470_Handle_t *hmotor, uint32_t timeout_ms);

/*DMA function prototypes*/
HAL_StatusTypeDef L6470_SendCommand_DMA(L6470_Handle_t *hmotor, uint8_t command);
HAL_StatusTypeDef L6470_SendCommand_Data_DMA(L6470_Handle_t *hmotor, uint8_t command, uint32_t data);
HAL_StatusTypeDef L6470_GetParam_DMA(L6470_Handle_t *hmotor, uint8_t param, uint32_t *value);
HAL_StatusTypeDef L6470_SetParam_DMA(L6470_Handle_t *hmotor, uint8_t param, uint32_t value);

/* Helper Functions */
uint8_t L6470_SendCommand(L6470_Handle_t *hmotor, uint8_t command);
void L6470_SendData(L6470_Handle_t *hmotor, uint8_t *data, uint8_t length);
void L6470_ReceiveData(L6470_Handle_t *hmotor, uint8_t *data, uint8_t length);
uint32_t L6470_ConvertSpeed(uint32_t speed_steps_per_sec);
uint32_t L6470_ConvertAcceleration(uint32_t accel_steps_per_sec2);

#endif /* INC_L6470_DRIVER_H_ */
