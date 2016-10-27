/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/


#include <SPI.h>
#include "DeviceNRF24L01.h"
#include "Utils.h"

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

#define NRF_CS_HI()     digitalWrite(PIN_NRF_CSN, HIGH);
#define NRF_CS_LO()     digitalWrite(PIN_NRF_CSN, LOW);

DeviceNRF24L01::DeviceNRF24L01()
{
    initialize();
}

void DeviceNRF24L01::initialize()
{
    setRFSwitch(TX_NRF24L01);

    mSPI->setBitOrder(MSBFIRST);
    mSPI->setDataMode(SPI_MODE0);
    mSPI->setClockDivider(SPI_CLOCK_DIV4);
    mRFsetup = 0x0F;
    xn297_scramble_enabled = 1;
    xn297_crc = 0;
}

u8 DeviceNRF24L01::writeReg(u8 reg, u8 data)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(W_REGISTER | (REGISTER_MASK & reg));
    mSPI->transfer(data);
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::writeRegMulti(u8 reg, const u8 *data, u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++) {
        mSPI->transfer(*data++);
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::writeRegMulti_P(u8 reg, const u8 *data, u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++) {
        mSPI->transfer(pgm_read_byte(data++));
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::writePayload(u8 *data, u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++) {
        mSPI->transfer(*data++);
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::writePayload_P(const u8 *data, u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++) {
        mSPI->transfer(pgm_read_byte(data++));
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::readReg(u8 reg)
{
    NRF_CS_LO();
    mSPI->transfer(R_REGISTER | (REGISTER_MASK & reg));
    u8 data = mSPI->transfer(0xFF);
    NRF_CS_HI();
    return data;
}

u8 DeviceNRF24L01::readRegMulti(u8 reg, u8 data[], u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(R_REGISTER | (REGISTER_MASK & reg));
    for(u8 i = 0; i < length; i++) {
        data[i] = mSPI->transfer(0xFF);
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::readPayload(u8 *data, u8 length)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(R_RX_PAYLOAD);
    for(u8 i = 0; i < length; i++) {
        data[i] = mSPI->transfer(0xFF);
    }
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::strobe(u8 state)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(state);
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::flushTx()
{
    return strobe(FLUSH_TX);
}

u8 DeviceNRF24L01::flushRx()
{
    return strobe(FLUSH_RX);
}

u8 DeviceNRF24L01::activate(u8 code)
{
    NRF_CS_LO();
    u8 res = mSPI->transfer(ACTIVATE);
    mSPI->transfer(code);
    NRF_CS_HI();
    return res;
}

u8 DeviceNRF24L01::setBitrate(u8 bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    mRFsetup = (mRFsetup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return writeReg(NRF24L01_06_RF_SETUP, mRFsetup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm.
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/
u8 DeviceNRF24L01::setRFPower(u8 power)
{
    u8 nrf_power = 0;

    switch(power) {
        case TXPOWER_100uW:
        case TXPOWER_300uW:
        case TXPOWER_1mW:
            nrf_power = 0;
            break;

        case TXPOWER_3mW:
        case TXPOWER_10mW:
            nrf_power = 1;
            break;

        case TXPOWER_30mW:
            nrf_power = 2;
            break;

        case TXPOWER_100mW:
        case TXPOWER_150mW:
            nrf_power = 3;
            break;
    };

    // Power is in range 0..3 for nRF24L01
    mRFsetup = (mRFsetup & 0xF9) | ((nrf_power & 0x03) << 1);
    return writeReg(NRF24L01_06_RF_SETUP, mRFsetup);
}

void DeviceNRF24L01::setRFModeImpl(enum RF_MODE mode)
{
    if (mode == RF_TX) {
        writeReg(NRF24L01_07_STATUS,  BV(NRF24L01_07_RX_DR)     // reset the flag(s)
                                    | BV(NRF24L01_07_TX_DS)
                                    | BV(NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG,  BV(NRF24L01_00_EN_CRC)    // switch to TX mode
                                    | BV(NRF24L01_00_CRCO)
                                    | BV(NRF24L01_00_PWR_UP));
        delayMicroseconds(150);
    } else if (mode == RF_RX) {
        writeReg(NRF24L01_07_STATUS, 0x70);                     // reset the flag(s)
        writeReg(NRF24L01_00_CONFIG, 0x0F);                     // switch to RX mode
        writeReg(NRF24L01_07_STATUS,  BV(NRF24L01_07_RX_DR)     // reset the flag(s)
                                    | BV(NRF24L01_07_TX_DS)
                                    | BV(NRF24L01_07_MAX_RT));
        writeReg(NRF24L01_00_CONFIG,  BV(NRF24L01_00_EN_CRC)    // switch to RX mode
                                    | BV(NRF24L01_00_CRCO)
                                    | BV(NRF24L01_00_PWR_UP)
                                    | BV(NRF24L01_00_PRIM_RX));
        delayMicroseconds(150);
    } else {
        writeReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC));   //PowerDown
    }
}

int DeviceNRF24L01::reset()
{
    flushTx();
    flushRx();
    u8 status1 = strobe(NOP);
    u8 status2 = readReg(0x07);
    setRFMode(RF_IDLE);
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}


// XN297 emulation layer
static const u8 xn297_scramble[] = {
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
    0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
    0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
    0x8e, 0xc5, 0x2f};

static const u16 xn297_crc_xorout_scrambled[] = {
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
    0x2138, 0x129F, 0xB3A0, 0x2988};

static const u16 xn297_crc_xorout[] = {
    0x0000, 0x3d5f, 0xa6f1, 0x3a23, 0xaa16, 0x1caf,
    0x62b2, 0xe0eb, 0x0821, 0xbe07, 0x5f1a, 0xaf15,
    0x4f0a, 0xad24, 0x5e48, 0xed34, 0x068c, 0xf2c9,
    0x1852, 0xdf36, 0x129d, 0xb17c, 0xd5f5, 0x70d7,
    0xb798, 0x5133, 0x67db, 0xd94e};

u8 DeviceNRF24L01::bit_reverse(u8 b_in)
{
    u8 b_out = 0;
    for (u8 i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

static const u16 polynomial = 0x1021;
u16 DeviceNRF24L01::crc16_update(u16 crc, u8 a)
{
    crc ^= a << 8;
    for (u8 i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ polynomial;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void DeviceNRF24L01::XN297_setTxAddr(const u8* addr, u8 len)
{
    if (len > 5)
        len = 5;
    if (len < 3)
        len = 3;

    u8 buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
    xn297_addr_len = len;
    if (xn297_addr_len < 4) {
        for (u8 i = 0; i < 4; ++i) {
            buf[i] = buf[i+1];
        }
    }
    writeReg(NRF24L01_03_SETUP_AW, len-2);
    writeRegMulti(NRF24L01_10_TX_ADDR, buf, 5);
    // Receive address is complicated. We need to use scrambled actual address as a receive address
    // but the TX code now assumes fixed 4-byte transmit address for preamble. We need to adjust it
    // first. Also, if the scrambled address begings with 1 nRF24 will look for preamble byte 0xAA
    // instead of 0x55 to ensure enough 0-1 transitions to tune the receiver. Still need to experiment
    // with receiving signals.
    memcpy(xn297_tx_addr, addr, len);
}


void DeviceNRF24L01::XN297_setRxAddr(const u8* addr, u8 len)
{
    if (len > 5)
        len = 5;

    if (len < 3)
        len = 3;

    u8 buf[] = { 0, 0, 0, 0, 0 };
    memcpy(buf, addr, len);
    memcpy(xn297_rx_addr, addr, len);
    for (u8 i = 0; i < xn297_addr_len; ++i) {
        buf[i] = xn297_rx_addr[i];
        if(xn297_scramble_enabled)
            buf[i] ^= xn297_scramble[xn297_addr_len-i-1];
    }
    writeReg(NRF24L01_03_SETUP_AW, len-2);
    writeRegMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
}


void DeviceNRF24L01::XN297_configure(u8 flags)
{
    xn297_crc = !!(flags & BV(NRF24L01_00_EN_CRC));
    flags &= ~(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    writeReg(NRF24L01_00_CONFIG, flags & 0xff);
}

void DeviceNRF24L01::XN297_setScrambledMode(const u8 mode)
{
    xn297_scramble_enabled = mode;
}

u8 DeviceNRF24L01::XN297_writePayload(u8* data, u8 len)
{
    u8 last = 0;

    if (xn297_addr_len < 4) {
        // If address length (which is defined by receive address length)
        // is less than 4 the TX address can't fit the preamble, so the last
        // byte goes here
        packet[last++] = 0x55;
    }
    for (u8 i = 0; i < xn297_addr_len; ++i) {
        packet[last] = xn297_tx_addr[xn297_addr_len-i-1];
        if(xn297_scramble_enabled)
            packet[last] ^= xn297_scramble[i];
        last++;
    }

    for (u8 i = 0; i < len; ++i) {
        // bit-reverse bytes in packet
        u8 b_out = bit_reverse(data[i]);
        packet[last] = b_out;
        if(xn297_scramble_enabled)
            packet[last] ^= xn297_scramble[xn297_addr_len+i];
        last++;
    }

    if (xn297_crc) {
        u8 offset = xn297_addr_len < 4 ? 1 : 0;
        u16 crc = 0xb5d2;   // initial crc
        for (u8 i = offset; i < last; ++i) {
            crc = crc16_update(crc, packet[i]);
        }
        if(xn297_scramble_enabled)
            crc ^= pgm_read_word(&xn297_crc_xorout_scrambled[xn297_addr_len - 3 + len]);
        else
            crc ^= pgm_read_word(&xn297_crc_xorout[xn297_addr_len - 3 + len]);
        packet[last++] = crc >> 8;
        packet[last++] = crc & 0xff;
    }
    return writePayload(packet, last);
}


u8 DeviceNRF24L01::XN297_readPayload(u8* data, u8 len)
{
    // TODO: if xn297_crc==1, check CRC before filling *data
    u8 res = readPayload(data, len);
    for(u8 i=0; i<len; i++) {
      data[i] = bit_reverse(data[i]);
      if(xn297_scramble_enabled)
        data[i] ^= bit_reverse(xn297_scramble[i+xn297_addr_len]);
    }
    return res;
}


// End of XN297 emulation
