/*
 * l6470_driver.c
 *
 *  Created on: Jun 21, 2025
 *      Author: digia
 */
#include "l6470_driver.h"

/* Private function prototypes */
static void L6470_CS_High(L6470_Handle_t *hmotor);
static void L6470_CS_Low(L6470_Handle_t *hmotor);
static uint8_t L6470_GetParamLength(uint32_t param);
static void L6470_Delay(uint32_t ms);
/**
 * @brief Initialize L6470 driver
 * @param hmotor: Pointer to L6470 handle structure
 * @param config: Pointer to configuration structure
 * @param config: DMA selector
 * @return L6470_Status_t
 */
L6470_Status_t L6470_Init(L6470_Handle_t *hmotor, L6470_Config_t *config, bool with_DMA)
{
    // Copy configuration
    hmotor->config = *config;

    // Set default parameters
    hmotor->step_mode = L6470_STEP_MODE_1_16; // MODE
    hmotor->max_speed = 1600; // max speed set to 1600 to  obtain in 1 s an half step
    hmotor->min_speed = 0;
    hmotor->acceleration = 100; //100 steps s^2
    hmotor->deceleration = 100; //100 steps s^2
    hmotor->config.transfer_complete = false;
    hmotor->config.transfer_error = false;

    //Clear DMA buffers
    for (int i = 0; i < DMA_BUFFER_SIZE; i++)
    {
    	hmotor->config.tx_buffer[i] = 0;
    	hmotor->config.rx_buffer[i] = 0;
    }

    // Initialize GPIO pins
    L6470_CS_High(hmotor);

    // Reset device
    if (L6470_Reset(hmotor) != L6470_OK)
    {
    	return L6470_ERROR;
    }

    // Wait for device to be ready
    L6470_Delay(100);

    L6470_SetParam(hmotor, L6470_STEP_MODE, hmotor->step_mode);

    if (with_DMA)
    {
      //Configure basic parameters with DMA
      L6470_SetParam_DMA(hmotor, L6470_MAX_SPEED, L6470_ConvertSpeed(hmotor->max_speed));
      L6470_SetParam_DMA(hmotor, L6470_MIN_SPEED, L6470_ConvertSpeed(hmotor->min_speed));
      L6470_SetParam_DMA(hmotor, L6470_ACC, L6470_ConvertAcceleration(hmotor->acceleration));
      L6470_SetParam_DMA(hmotor, L6470_DEC, L6470_ConvertAcceleration(hmotor->deceleration));
    }
    else
    {
    	// Configure basic parameters
			L6470_SetParam(hmotor, L6470_MAX_SPEED, L6470_ConvertSpeed(hmotor->max_speed));
			L6470_SetParam(hmotor, L6470_MIN_SPEED, L6470_ConvertSpeed(hmotor->min_speed));
			L6470_SetParam(hmotor, L6470_ACC, L6470_ConvertAcceleration(hmotor->acceleration));
			L6470_SetParam(hmotor, L6470_DEC, L6470_ConvertAcceleration(hmotor->deceleration));
    }

    return L6470_OK;
}

/**
 * @brief Reset L6470 device
 * @param hmotor: Pointer to L6470 handle structure
 * @return L6470_Status_t
 */
L6470_Status_t L6470_Reset(L6470_Handle_t *hmotor)
{
    // Hardware reset
    HAL_GPIO_WritePin(hmotor->config.reset_port, hmotor->config.reset_pin, GPIO_PIN_RESET);
    L6470_Delay(10);
    HAL_GPIO_WritePin(hmotor->config.reset_port, hmotor->config.reset_pin, GPIO_PIN_SET);
    L6470_Delay(10);

    // Software reset (the ResetDevice command resets the device to power-up conditions)
    L6470_SendCommand(hmotor, L6470_RESET_DEVICE);
    L6470_Delay(10);

    return L6470_OK;
}

/**
 * @brief Set parameter value
 * @param hmotor: Pointer to L6470 handle structure
 * @param param: Parameter to set
 * @param value: Value to set
 * @return L6470_Status_t
 */
L6470_Status_t L6470_SetParam(L6470_Handle_t *hmotor, uint8_t param, uint32_t value)
{
    uint8_t length = L6470_GetParamLength(param);
    uint8_t data[4];

    data[0] = L6470_SET_PARAM | param;

    //Store value in data buffer
    for (int i = 0; i < length; i++)
    {
        data[i + 1] = (value >> (8 * (length - 1 - i))) & 0xFF;
    }

    L6470_CS_Low(hmotor);
    L6470_SendData(hmotor, data, length + 1);
    L6470_CS_High(hmotor);

    return L6470_OK;
}

/**
 * @brief Get parameter value
 * @param hmotor: Pointer to L6470 handle structure
 * @param param: Parameter to get
 * @return Parameter value
 */
uint32_t L6470_GetParam(L6470_Handle_t *hmotor, uint8_t param)
{
    uint8_t length = L6470_GetParamLength(param);
    uint8_t cmd = L6470_GET_PARAM | param;
    uint8_t data[4] = {0};
    uint32_t value = 0;

    L6470_CS_Low(hmotor);
    L6470_SendData(hmotor, &cmd, 1);
    L6470_ReceiveData(hmotor, data, length);
    L6470_CS_High(hmotor);

    //Store value in data buffer
    for (int i = 0; i < length; i++)
    {
    	value = (value << 8) | data[i];
    }

    return value;
}

/**
 * @brief Move motor to absolute position
 * @param hmotor: Pointer to L6470 handle structure
 * @param position: Target position
 */
void L6470_GoTo(L6470_Handle_t *hmotor, uint32_t position)
{
    uint8_t data[4];

    data[0] = L6470_GOTO;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = position & 0xFF;

    L6470_CS_Low(hmotor);
    L6470_SendData(hmotor, data, 4);
    L6470_CS_High(hmotor);
}

/**
 * @brief Get motor status
 * @param hmotor: Pointer to L6470 handle structure
 * @return Status register value
 */
uint16_t L6470_GetStatus(L6470_Handle_t *hmotor)
{
    uint8_t data[2];
    uint8_t cmd = L6470_GET_STATUS;

    L6470_CS_Low(hmotor);
    L6470_SendData(hmotor, &cmd, 1);
    L6470_ReceiveData(hmotor, data, 2);
    L6470_CS_High(hmotor);

    return (data[0] << 8) | data[1];
}

/**
 * @brief Get current motor position
 * @param hmotor: Pointer to L6470 handle structure
 * @return Current position (signed)
 */
int32_t L6470_GetPosition(L6470_Handle_t *hmotor)
{
    uint32_t pos = L6470_GetParam(hmotor, L6470_ABS_POS);

    // Convert to signed value (22-bit 2's complement)
    if (pos & 0x200000)
    {
    	pos |= 0xFFC00000;
    }

    return (int32_t)pos;
}


/**
 * @brief Check if motor is busy
 * @param hmotor: Pointer to L6470 handle structure
 * @return 1 if busy, 0 if not busy
 */
uint8_t L6470_IsBusy(L6470_Handle_t *hmotor)
{
    return !HAL_GPIO_ReadPin(hmotor->config.busy_port, hmotor->config.busy_pin);
}

/**
 * @brief Wait while motor is busy
 * @param hmotor: Pointer to L6470 handle structure
 * @param timeout_ms: Timeout in milliseconds
 * @return L6470_Status_t
 */
L6470_Status_t L6470_WaitWhileBusy(L6470_Handle_t *hmotor, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while (L6470_IsBusy(hmotor)) {
        if ((HAL_GetTick() - start_time) > timeout_ms) {
            return L6470_TIMEOUT;
        }
        L6470_Delay(1);
    }

    return L6470_OK;
}

/**
 * @brief Send single command
 * @param hmotor: Pointer to L6470 handle structure
 * @param command: Command to send
 * @return Response byte
 */
uint8_t L6470_SendCommand(L6470_Handle_t *hmotor, uint8_t command)
{
    uint8_t response;

    L6470_CS_Low(hmotor);
    HAL_SPI_TransmitReceive(hmotor->config.hspi, &command, &response, 1, HAL_MAX_DELAY);
    L6470_CS_High(hmotor);

    return response;
}

/*************************************************
 * 							DMA FUNCTION
 *************************************************/

/**
 * @brief Reset motor position to zero
 * @param hmotor: Pointer to L6470 handle structure
 * @param with_DMA: DMA selector
 * @return L6470_Status_t
 */
L6470_Status_t L6470_ResetPosition(L6470_Handle_t *hmotor, bool with_DMA)
{
		if(with_DMA)
		{
			L6470_SendCommand_DMA(hmotor, L6470_RESET_POS);
		}
		else
		{
	    L6470_SendCommand(hmotor, L6470_RESET_POS);
		}
		return L6470_OK;
}


/**
 * @brief Send command through DMA
 * @param hmotor: Pointer to L6470 handle structure
 * @param command: Command to send
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef L6470_SendCommand_DMA(L6470_Handle_t *hmotor, uint8_t command)
{
	hmotor->config.transfer_complete = false;
	hmotor->config.transfer_error = false;

	// Prepare command to transmit
	hmotor->config.tx_buffer[0] = command;

	L6470_CS_Low(hmotor);

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hmotor->config.hspi,
																												 hmotor->config.tx_buffer,
																												 hmotor->config.rx_buffer,
																												 1);

	if (status != HAL_OK)
	{
		L6470_CS_High(hmotor);
		return status;
	}

	// Wait for transfer completion
	uint32_t timeout = HAL_GetTick() + 100;
	while (!hmotor->config.transfer_complete && !hmotor->config.transfer_error && HAL_GetTick() < timeout)
	{
		//Wait for callbacks
		L6470_Delay(1);
	}

	L6470_CS_High(hmotor);

	if (hmotor->config.transfer_error)
		return HAL_ERROR;

	if (!hmotor->config.transfer_complete)
		return HAL_TIMEOUT;

	return HAL_OK;
}

/**
 * @brief Send data through DMA
 * @param hmotor: Pointer to L6470 handle structure
 * @param command: Command to send
 * @param command: Data to send
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef L6470_SendCommand_Data_DMA(L6470_Handle_t *hmotor, uint8_t command, uint32_t data)
{
	hmotor->config.transfer_complete = false;
	hmotor->config.transfer_error = false;

	// Prepare data for transmission (MSB first)
	hmotor->config.tx_buffer[0] = command;
	hmotor->config.tx_buffer[1] = (data >> 16) & 0xFF;
	hmotor->config.tx_buffer[2] = (data >> 8) & 0xFF;
	hmotor->config.tx_buffer[3] = data & 0xFF;

	L6470_CS_Low(hmotor);

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hmotor->config.hspi,
																												 hmotor->config.tx_buffer,
																												 hmotor->config.rx_buffer,
																												 4);

	if (status != HAL_OK)
	{
		L6470_CS_High(hmotor);
		return status;
	}

	// Wait for transfer completion
	uint32_t timeout = HAL_GetTick() + 100;
	while (!hmotor->config.transfer_complete && !hmotor->config.transfer_error && HAL_GetTick() < timeout)
	{
		//Wait for callbacks
		L6470_Delay(1);
	}

	L6470_CS_High(hmotor);

	if (hmotor->config.transfer_error)
		return HAL_ERROR;

	if (!hmotor->config.transfer_complete)
		return HAL_TIMEOUT;

	return HAL_OK;
}

/**
 * @brief Set parameters through DMA
 * @param hmotor: Pointer to L6470 handle structure
 * @param param: parameter of configuration
 * @param value: value of parameter
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef L6470_SetParam_DMA(L6470_Handle_t *hmotor, uint8_t param, uint32_t value)
{
    return L6470_SendCommand_Data_DMA(hmotor, L6470_SET_PARAM | param, value);
}

/**
 * @brief Send command through DMA
 * @param hmotor: Pointer to L6470 handle structure
 * @param param: parameter for the command to send
 * @param value: value read from the rx buffer
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef L6470_GetParam_DMA(L6470_Handle_t *hmotor, uint8_t param, uint32_t *value)
{
	hmotor->config.transfer_complete = false;
	hmotor->config.transfer_error = false;

	hmotor->config.tx_buffer[0] = L6470_GET_PARAM | param;
	hmotor->config.tx_buffer[1] = 0x00;
	hmotor->config.tx_buffer[2] = 0x00;
	hmotor->config.tx_buffer[3] = 0x00;

	L6470_CS_Low(hmotor);

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(hmotor->config.hspi,
																												hmotor->config.tx_buffer,
																												hmotor->config.rx_buffer,
																												4);

	if (status != HAL_OK)
	{
		L6470_CS_High(hmotor);
	  return status;
	}

	// Wait for transfer completion and check for errors
	uint32_t timeout = HAL_GetTick() + 100;
	while (!hmotor->config.transfer_complete && !hmotor->config.transfer_error && HAL_GetTick() < timeout)
	{
		//Wait for callbacks
		L6470_Delay(1);
	}

	L6470_CS_High(hmotor);

	if (hmotor->config.transfer_error)
	  return HAL_ERROR;

	if (!hmotor->config.transfer_complete)
	  return HAL_TIMEOUT;

	// Extract received data
	*value = ((uint32_t)hmotor->config.rx_buffer[1] << 16) |
					 ((uint32_t)hmotor->config.rx_buffer[2] << 8) |
					 (uint32_t)hmotor->config.rx_buffer[3];

	return HAL_OK;
}



/*************************************************
 * 						END	DMA FUNCTION
 *************************************************/

/**
 * @brief Send data array
 * @param hmotor: Pointer to L6470 handle structure
 * @param data: Data array to send
 * @param length: Length of data array
 */
void L6470_SendData(L6470_Handle_t *hmotor, uint8_t *data, uint8_t length)
{
    HAL_SPI_Transmit(hmotor->config.hspi, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Receive data array
 * @param hmotor: Pointer to L6470 handle structure
 * @param data: Buffer to receive data
 * @param length: Length of data to receive
 */
void L6470_ReceiveData(L6470_Handle_t *hmotor, uint8_t *data, uint8_t length)
{
    HAL_SPI_Receive(hmotor->config.hspi, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Convert speed from steps/sec to L6470 format
 * @param speed_steps_per_sec: Speed in steps per second
 * @return Converted speed value
 */
uint32_t L6470_ConvertSpeed(uint32_t speed_steps_per_sec)
{
    return (uint32_t)((float)speed_steps_per_sec * 67.108864);
}

/**
 * @brief Convert acceleration from steps/sec² to L6470 format
 * @param accel_steps_per_sec2: Acceleration in steps per second squared
 * @return Converted acceleration value
 */
uint32_t L6470_ConvertAcceleration(uint32_t accel_steps_per_sec2)
{
    return (uint32_t)((float)accel_steps_per_sec2 * 0.068719);
}

/* Private Functions */

/**
 * @brief Set CS pin high
 * @param hmotor: Pointer to L6470 handle structure
 */
static void L6470_CS_High(L6470_Handle_t *hmotor)
{
    HAL_GPIO_WritePin(hmotor->config.cs_port, hmotor->config.cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Set CS pin low
 * @param hmotor: Pointer to L6470 handle structure
 */
static void L6470_CS_Low(L6470_Handle_t *hmotor)
{
    HAL_GPIO_WritePin(hmotor->config.cs_port, hmotor->config.cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Get parameter length in bytes
 * @param param: Parameter ID
 * @return Length in bytes
 */
static uint8_t L6470_GetParamLength(uint32_t param)
{
    switch (param) {
        case L6470_STATUS_BUSY:
        case L6470_STATUS_MOT_STATUS:
        case L6470_STATUS_NOTPERF_CMD:
        case L6470_STATUS_WRONG_CMD:
            return 2;
        default:
            return 1;
    }
}

/**
 * @brief Simple delay function
 * @param ms: Delay in milliseconds
 */
static void L6470_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

