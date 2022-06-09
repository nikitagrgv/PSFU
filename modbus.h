#pragma once
#include "stm32f10x.h"
#include "meander.h" // for MEANDER_FREQ
#include "sifu.h"    // for UART_VALUE
#include "crc.h"

// address of this modbus device
const uint8_t DEVICE_ADDRESS = 0x01;

// states of modbus machine
typedef enum
{
    READY,     // modbus machine is ready to receive new packets
    RECEIVING, // modbus machine is receiving current packet
    PROCESSING // modbus machine has received the packet and is processing it
} ModbusState;

// modbus machine structure
struct
{
    ModbusState state; // state of modbus machine

    struct // received modbus packet
    {
        uint8_t data[32];
        uint8_t size;
    } in;

    struct // modbus packet that will be transmitted
    {
        uint8_t data[32];
        uint8_t size;
    } out;
} Modbus;

// add CRC to the packet
void AddCRC()
{
    uint16_t crc = CRC16(Modbus.out.data, Modbus.out.size);
    Modbus.out.data[Modbus.out.size++] = (crc & 0x00FF) >> 0; // low CRC byte
    Modbus.out.data[Modbus.out.size++] = (crc & 0xFF00) >> 8; // high CRC byte
}

// transmit response via USART1
void TransmitResponse()
{
    // enable driver mode by setting high level on A11
    GPIOA->BSRR = GPIO_BSRR_BS11;

    for (uint8_t i = 0; i < Modbus.out.size; i++)
    {
        // wait for USART ready
        while (!(USART1->SR & USART_SR_TXE))
            ;
        // transmit current byte
        USART1->DR = Modbus.out.data[i];
    }

    // wait for transmitting last byte
    for (volatile uint16_t i = 0; i < 450; i++)
        ;

    // enable receiver mode by setting low level on A11
    GPIOA->BSRR = GPIO_BSRR_BR11;
}

// form response with error code (without CRC)
void SendError(uint8_t error_code)
{
    Modbus.out.data[0] = DEVICE_ADDRESS;           // device address
    Modbus.out.data[1] = Modbus.in.data[1] + 0x80; // function code + error bit
    Modbus.out.data[2] = error_code;               // error code
    Modbus.out.size = 3;
    AddCRC();
    TransmitResponse();
}

// process modbus packet and form response
void ProcessPacket()
{
    // ignore empty packets
    if (Modbus.in.size <= 4)
    {
        return;
    }

    // ignore if packet is not for us
    if (Modbus.in.data[0] != DEVICE_ADDRESS)
    {
        return;
    }

    // ignore if CRC validation failed
    uint16_t crc_valid = CRC16(Modbus.in.data, Modbus.in.size - 2);
    if (((crc_valid & 0x00FF) >> 0) != Modbus.in.data[Modbus.in.size - 2] ||
        ((crc_valid & 0xFF00) >> 8) != Modbus.in.data[Modbus.in.size - 1])
    {
        return;
    }

    // request for reading input register (function code = 0x04)
    if (Modbus.in.data[1] == 0x04)
    {
        // get starting address from the request
        uint16_t starting_address;
        starting_address = Modbus.in.data[2] << 8 | // high byte
                           Modbus.in.data[3];       // low byte
        // get registers quantity from the request
        uint16_t registers_quantity;
        registers_quantity = Modbus.in.data[4] << 8 | // high byte
                             Modbus.in.data[5];       // low byte

        // only addresses 0x00, 0x01 are allowed, if other, send error code 2
        if (starting_address > 0x01)
        {
            SendError(0x02);
            return;
        }

        // there is only two registers, if other quantity, send error code 2
        // quantity must not be zero
        if (registers_quantity > 2 || registers_quantity == 0)
        {
            SendError(0x02);
            return;
        }

        // out of borders of registers
        if (starting_address + registers_quantity > 2)
        {
            SendError(0x02);
            return;
        }

        Modbus.out.data[0] = DEVICE_ADDRESS;         // device address
        Modbus.out.data[1] = Modbus.in.data[1];      // function code
        Modbus.out.data[2] = registers_quantity * 2; // bytes count = registers count * 2
        Modbus.out.size = 3;
        // write registers data
        for (int i = starting_address; i < starting_address + registers_quantity; i++)
        {
            // 0x00 - meander frequency
            if (i == 0)
            {
                Modbus.out.data[Modbus.out.size++] = MEANDER_FREQ >> 8; // high byte
                Modbus.out.data[Modbus.out.size++] = MEANDER_FREQ;      // low byte
            }
            // 0x01 - value from UART
            if (i == 1)
            {
                Modbus.out.data[Modbus.out.size++] = UART_VALUE >> 8; // high byte
                Modbus.out.data[Modbus.out.size++] = UART_VALUE;      // low byte
            }
        }
    }
    else
    {
        // other function codes are not supported, send error code 1
        SendError(0x01);
        return;
    }

    AddCRC();
    TransmitResponse();
}

// -----------------------------------------------------------------------------
// interrupts
// -----------------------------------------------------------------------------

// USART1 interrupt. For receiving Modbus packets. changes state of modbus
void USART1_IRQHandler(void)
{
    // reload the timer that performs the silence detection
    TIM3->CNT = 0;
    // read UART data
    uint8_t data = USART1->DR;

    // if modbus machine is ready, go to receive new packet
    if (Modbus.state == READY)
    {
        Modbus.state = RECEIVING;
        Modbus.in.size = 0;
    }
    // if modbus machine is processing packet, ignore new packet
    else if (Modbus.state == PROCESSING)
    {
        return;
    }

    // add new data to packet
    Modbus.in.data[Modbus.in.size] = data;
    Modbus.in.size++;
}

// TIM3 interrupt. For modbus packets separation. Triggers after 3.5+1 bytes pause
void TIM3_IRQHandler(void)
{
    // clear interrupt flag
    TIM3->SR = 0;

    // if packet was receiving, finish the packet and go to process it
    if (Modbus.state == RECEIVING)
    {
        Modbus.state = PROCESSING;
    }
}
