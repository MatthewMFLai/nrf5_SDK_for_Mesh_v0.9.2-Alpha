/* Copyright (c) Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <unity.h>
#include <cmock.h>

#include "serial_uart.h"

#include "fifo_mock.h"

NRF_UART_Type m_uart;
NRF_UART_Type* NRF_UART0 = &m_uart;
NRF_GPIO_Type m_gpio;
NRF_GPIO_Type* NRF_GPIO = &m_gpio;

void setUp(void)
{
}

void tearDown(void)
{

}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_init_params(void)
{
    nrf_mesh_serial_uart_params_t init_params;

    /* Happy path! */
    init_params.baudrate =  4321432;
    init_params.flow_control = true;
    init_params.pin_cts = 0x01;
    init_params.pin_rts = 0x02;
    init_params.pin_rx  = 0x03;
    init_params.pin_tx  = 0x04;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, serial_uart_init(&init_params));
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_cts, NRF_UART0->PSELCTS);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_rts, NRF_UART0->PSELRTS);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_rx,  NRF_UART0->PSELRXD);
    TEST_ASSERT_EQUAL_HEX32(init_params.pin_tx,  NRF_UART0->PSELTXD);
    TEST_ASSERT_EQUAL_HEX32(init_params.baudrate, NRF_UART0->BAUDRATE);
    TEST_ASSERT_EQUAL_HEX32(init_params.flow_control, !!(NRF_UART0->CONFIG & UART_CONFIG_HWFC_Msk));
    TEST_ASSERT_EQUAL_HEX32(0, NRF_UART0->CONFIG & UART_CONFIG_PARITY_Msk);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, serial_uart_init(&init_params));
}

void test_tx(void)
{

}

void test_rx(void)
{

}
