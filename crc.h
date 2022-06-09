#pragma once

#include <stdint.h>

#define BYTE uint8_t
#define BIT uint8_t

BYTE Reflect(BYTE byte)
{
    BYTE reflected = 0;
    for (int i = 0; i < 8; i++)
    {
        BIT bit = (byte >> i) & 1;
        reflected |= bit << (7 - i);
    }
    return reflected;
}

uint16_t CRC16(BYTE *data, int size)
{
    // temporary bytes
    BYTE tmp_byte[3] = {0, 0, 0};

    // add data to temporary bytes
    for(int i = 0; i < size && i < 3; i++)
    {
        tmp_byte[i] = Reflect(data[i]);
    }

    // apply init value
    tmp_byte[0] ^= 0xFF;
    tmp_byte[1] ^= 0xFF;

    // go through all bytes of data
    for (int byte_pos = 0; byte_pos < size; byte_pos++)
    {
        // go through bits of current byte, begin from MSB
        for (int bit_pos = 7; bit_pos >= 0; bit_pos--)
        {
            // get current bit
            BIT bit = (tmp_byte[0] >> bit_pos) & 1;

            // 0x18005 = 0b1 1000000000000101
            if (bit == 1)
            {
                // get global position of current bit
                int curr_bit_pos_glob = byte_pos * 8 + (7 - bit_pos);

                // get global positions of xor bits
                int xor_bit_pos_glob[3];
                xor_bit_pos_glob[0] = curr_bit_pos_glob + 1;
                xor_bit_pos_glob[1] = curr_bit_pos_glob + 14;
                xor_bit_pos_glob[2] = curr_bit_pos_glob + 16;

                // calculate in what bytes located xor bits
                int xor_byte_pos[3];
                xor_byte_pos[0] = xor_bit_pos_glob[0] / 8;
                xor_byte_pos[1] = xor_bit_pos_glob[1] / 8;
                xor_byte_pos[2] = xor_bit_pos_glob[2] / 8;

                // calculate local positions of xor bits
                int xor_bit_pos[3];
                xor_bit_pos[0] = 7 - (xor_bit_pos_glob[0] % 8);
                xor_bit_pos[1] = 7 - (xor_bit_pos_glob[1] % 8);
                xor_bit_pos[2] = 7 - (xor_bit_pos_glob[2] % 8);

                // do xor
                tmp_byte[xor_byte_pos[0] - byte_pos] ^= 1 << xor_bit_pos[0];
                tmp_byte[xor_byte_pos[1] - byte_pos] ^= 1 << xor_bit_pos[1];
                tmp_byte[xor_byte_pos[2] - byte_pos] ^= 1 << xor_bit_pos[2];
            }
        }
        // go to next byte
        tmp_byte[0] = tmp_byte[1];
        tmp_byte[1] = tmp_byte[2];
        if (byte_pos + 3 < size)
        {
            tmp_byte[2] = Reflect(data[byte_pos + 3]);
        }
        else
        {
            tmp_byte[2] = 0;
        }
    }

    // reflect remainder and get CRC
    uint16_t crc = 0;
    crc |= Reflect(tmp_byte[1]) << 8;
    crc |= Reflect(tmp_byte[0]);

    return crc;
}
