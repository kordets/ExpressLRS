#include "Wire.h"
#include "internal.h"
#include "gpio.h" // i2c_setup
#include "utils.h"

constexpr uint8_t i2c_cnt = 0
#ifdef I2C1_BASE
    + 1
#endif
#ifdef I2C2_BASE
    + 1
#endif
#ifdef I2C3_BASE
    + 1
#endif
    ;
I2C_HandleTypeDef i2c_handles[i2c_cnt];

void* i2c_setup(uint32_t rate, uint8_t own_addr, uint8_t index)
{
    I2C_HandleTypeDef * handle = &i2c_handles[index];

    switch (index) {
#ifdef I2C1_BASE
        case 0:
            handle->Instance = I2C1;
            break;
#endif
#ifdef I2C2_BASE
        case 1:
            handle->Instance = I2C2;
            break;
#endif
#ifdef I2C3_BASE
        case 2:
            handle->Instance = I2C3;
            break;
#endif
        default:
            return NULL;
    }
#if defined(STM32F1xx) && defined(I2C_DUTYCYCLE_2)
    handle->Init.ClockSpeed = rate;
    handle->Init.DutyCycle = (100000 < rate) ? I2C_DUTYCYCLE_16_9 : I2C_DUTYCYCLE_2;
#elif defined(STM32L0xx)
    /* STM32L0xx runs with 32MHz clock! */
    handle->Init.Timing = rate == 100000 ? 0x10E0474A : 0x00B01626;
#elif defined(STM32L4xx)
    /* STM32L4xx runs with 80MHz clock! */
    handle->Init.Timing = rate == 100000 ? 0x60903132 : 0x10D11C2F;
#else
#error "No valid I2C config!"
#endif
    handle->Init.OwnAddress1 = own_addr;
    handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    handle->Init.OwnAddress2 = 0;
    handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(handle) != HAL_OK) {
        /* Initialization Error */
        shutdown("i2c init failed!");
        return NULL;
    }
    return handle;
}

/************************************************************************************************/

I2C::I2C()
{
}

void I2C::setSDA(uint32_t sda)
{
    sda_pin = sda;
}

void I2C::setSCL(uint32_t scl)
{
    scl_pin = scl;
}

void I2C::begin()
{
    uint8_t index = 2;
    if (scl_pin == GPIO('B', 6) && sda_pin == GPIO('B', 7))
        index = 0;
    else if (scl_pin == GPIO('B', 10) && sda_pin == GPIO('B', 11))
        index = 1;
    if (index < 2) {
        i2c = i2c_setup(100000, 0x33, index);
        // TODO: Setup gpio AFIO!
        //gpio_peripheral(scl_pin, GPIO_FUNCTION(7), 1);
        //gpio_peripheral(sda_pin, GPIO_FUNCTION(7), 0);
    }
}

void I2C::beginTransmission(uint8_t addr)
{
    address = (addr << 1);
}

void I2C::write(uint8_t data)
{
    if (!address || sizeof(buffer) <= buffer_cnt)
        return;
    buffer[buffer_cnt++] = data;
}

void I2C::write(uint8_t * data, uint8_t len)
{
    while(len--) {
        write(*data++);
    }
}

void I2C::requestFrom(uint8_t addr, uint8_t len)
{
    if (!addr || sizeof(buffer) < len)
        return;
    buffer_cnt = 0;
    rx_idx = 0;
    if (HAL_I2C_Master_Receive((I2C_HandleTypeDef*)i2c, (addr << 1), buffer, len, HAL_MAX_DELAY) == HAL_OK) {
        buffer_cnt = len;
    }
}

uint8_t I2C::available()
{
    return (buffer_cnt <= rx_idx) ? 0 : (buffer_cnt - rx_idx);
}

uint8_t I2C::read()
{
    if (buffer_cnt <= rx_idx)
        return 0;
    return buffer[rx_idx++];
}

void I2C::endTransmission()
{
    if (address)
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)i2c, address, buffer, buffer_cnt, HAL_MAX_DELAY);
    address = 0;
}

I2C Wire;
