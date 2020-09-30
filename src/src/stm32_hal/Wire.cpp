// I2C functions on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "Wire.h"
#include "internal.h"
#include "gpio.h" // i2c_setup
#include "utils.h"

I2C_HandleTypeDef i2c_handles[2];

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
        default:
            return NULL;
    }
    handle->Init.ClockSpeed = rate;
    handle->Init.DutyCycle = (100000 < rate) ? I2C_DUTYCYCLE_16_9 : I2C_DUTYCYCLE_2;
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
        index = 2;
    if (index < 2)
        i2c = i2c_setup(100000, 0x33, index);
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




#if 0

struct i2c_info
{
    I2C_TypeDef *i2c;
    uint8_t scl_pin, sda_pin;
};

const struct i2c_info i2c_bus[] = {
    {I2C1, GPIO('B', 6), GPIO('B', 7)},
    {I2C2, GPIO('B', 10), GPIO('B', 11)},
};


// Work around stm32 errata causing busy bit to be stuck
static void
i2c_busy_errata(uint8_t scl_pin, uint8_t sda_pin)
{
#if defined(STM32F1xx)
    gpio_peripheral(scl_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, -1);
    gpio_peripheral(scl_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, -1);
    gpio_peripheral(scl_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
    gpio_peripheral(sda_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 1);
#endif
}

I2C_TypeDef *
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    // Lookup requested i2c bus
    if (bus >= 2)
        return NULL;

    const struct i2c_info *ii = &i2c_bus[bus];
    I2C_TypeDef *i2c = ii->i2c;

    if (!is_enabled_pclock((uint32_t)i2c))
    {
        // Enable i2c clock and gpio
        enable_pclock((uint32_t)i2c);
        i2c_busy_errata(ii->scl_pin, ii->sda_pin);
        gpio_peripheral(ii->scl_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        gpio_peripheral(ii->sda_pin, GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN, 1);
        i2c->CR1 = I2C_CR1_SWRST;
        i2c->CR1 = 0;

        // Set 100Khz frequency and enable
        uint32_t pclk = get_pclock_frequency((uint32_t)i2c);
        i2c->CR2 = pclk / 1000000;
        i2c->CCR = pclk / 100000 / 2;
        i2c->TRISE = (pclk / 1000000) + 1;
        i2c->CR1 = I2C_CR1_PE;
    }

    return i2c;
}

static uint32_t
i2c_wait(I2C_TypeDef *i2c, uint32_t set, uint32_t clear, uint32_t timeout)
{
    for (;;)
    {
        uint32_t sr1 = i2c->SR1;
        if ((sr1 & set) == set && (sr1 & clear) == 0)
            return sr1;
        if (!timer_is_before(timer_read_time(), timeout))
            shutdown("i2c timeout");
    }
}

static void
i2c_start(I2C_TypeDef *i2c, uint8_t addr, uint32_t timeout)
{
    i2c->CR1 = I2C_CR1_START | I2C_CR1_PE;
    i2c_wait(i2c, I2C_SR1_SB, 0, timeout);
    i2c->DR = addr;
    i2c_wait(i2c, I2C_SR1_ADDR, 0, timeout);
    uint32_t sr2 = i2c->SR2;
    if (!(sr2 & I2C_SR2_MSL))
        shutdown("Failed to send i2c addr");
}

static void
i2c_send_byte(I2C_TypeDef *i2c, uint8_t b, uint32_t timeout)
{
    i2c->DR = b;
    i2c_wait(i2c, I2C_SR1_TXE, 0, timeout);
}

static void
i2c_stop(I2C_TypeDef *i2c, uint32_t timeout)
{
    i2c->CR1 = I2C_CR1_STOP | I2C_CR1_PE;
    i2c_wait(i2c, 0, I2C_SR1_TXE, timeout);
}

void i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)config.i2c;
    uint32_t timeout = timer_read_time() + microsecondsToClockCycles(5000);

    i2c_start(i2c, config.addr, timeout);
    while (write_len--)
        i2c_send_byte(i2c, *write++, timeout);
    i2c_stop(i2c, timeout);
}

void i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg, uint8_t read_len, uint8_t *read)
{
    //shutdown("i2c_read not supported on stm32");
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
        index = 0;
    if (index < 2)
        i2c = i2c_setup(index, 0, 0);
}

void I2C::beginTransmission(uint8_t addr)
{
    addr <<= 1;
    i2c_start((I2C_TypeDef *)i2c, addr, 0);
}

void I2C::write(uint8_t data)
{
    i2c_send_byte((I2C_TypeDef *)i2c, data, 0);
}

void I2C::endTransmission()
{
    i2c_stop((I2C_TypeDef *)i2c, 0);
}

I2C Wire;
#endif
