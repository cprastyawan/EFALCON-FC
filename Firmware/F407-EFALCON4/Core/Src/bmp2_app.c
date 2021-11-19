/*
 * bmp2_defs.c
 *
 *  Created on: Jun 15, 2021
 *      Author: LENOVO
 */

#include "bmp2_app.h"
#include <string.h>

static uint8_t dev_addr;

/*!
 * I2C read function map to COINES platform
 */
BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return 0;
}

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
BMP2_INTF_RET_TYPE bmp2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr){
	HAL_StatusTypeDef status;
	ACTIVATE_SPI(BMP_CS_GPIO_Port, BMP_CS_Pin);
	//status = HAL_SPI_TransmitReceive(&hspi1, &reg_addr, reg_data, length, HAL_MAX_DELAY);
	status = HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return 1;
	status = HAL_SPI_Receive(&hspi1, reg_data, length, HAL_MAX_DELAY);
	DEACTIVATE_SPI(BMP_CS_GPIO_Port, BMP_CS_Pin);
	if(status != HAL_OK) return 1;
	else return BMP2_INTF_RET_SUCCESS;
}

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
BMP2_INTF_RET_TYPE bmp2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr){
	uint8_t transmit_data[1 + length];
	transmit_data[0] = reg_addr;
	memcpy(&transmit_data[1], reg_data, length);

	HAL_StatusTypeDef status;
	ACTIVATE_SPI(BMP_CS_GPIO_Port, BMP_CS_Pin);
	//status = HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	status = HAL_SPI_Transmit(&hspi1, (uint8_t*)transmit_data, length + 1, HAL_MAX_DELAY);
	DEACTIVATE_SPI(BMP_CS_GPIO_Port, BMP_CS_Pin);
	if(status == HAL_OK) return BMP2_INTF_RET_SUCCESS;
	else return 1;
}

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period - The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
void bmp2_delay_us(uint32_t period, void *intf_ptr){
	  uint32_t clk_cycle_start = DWT->CYCCNT;

	  /* Go to number of cycles for system */
	  period *= (HAL_RCC_GetHCLKFreq() / 1000000);

	  /* Delay till end */
	  while ((DWT->CYCCNT - clk_cycle_start) < period);
}


void bmp2_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP2_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BMP2_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;
            case BMP2_E_COM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;
            case BMP2_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error.", rslt);
                printf("Occurs when length of data to be written is zero\n");
                break;
            case BMP2_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_RANGE:
                printf("Error [%d] : Uncompensated temperature data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_PRESS_RANGE:
                printf("Error [%d] : Uncompensated pressure data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
                printf(
                    "Error [%d] : Uncompensated pressure data and uncompensated temperature data are not in valid range error.",
                    rslt);
                break;
            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp2_interface_selection(struct bmp2_dev *dev, uint8_t intf)
{
    int8_t rslt = BMP2_OK;

    if (dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMP2_I2C_INTF)
        {
            printf("I2C Interface\r\n");

            dev_addr = BMP2_I2C_ADDR_PRIM;
            dev->read = bmp2_i2c_read;
            dev->write = bmp2_i2c_write;
            dev->intf = BMP2_I2C_INTF;

        }
        /* Bus configuration : SPI */
        else if (intf == BMP2_SPI_INTF)
        {
            printf("SPI Interface\r\n");

            dev_addr = BMP_CS_Pin;
            dev->read = bmp2_spi_read;
            dev->write = bmp2_spi_write;
            dev->intf = BMP2_SPI_INTF;

        }

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bmp2_delay_us;

        HAL_Delay(100);
    }
    else
    {
        rslt = BMP2_E_NULL_PTR;
    }

    return rslt;
}
