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


#ifndef _DEVICE_COMMON_H_
#define _DEVICE_COMMON_H_
#include <SPI.h>
#include "Common.h"

enum TXPOWER {
    TXPOWER_100uW = 0,      // -35dBm
    TXPOWER_300uW,          // -30dBm
    TXPOWER_1mW,            // -24dBm
    TXPOWER_3mW,            // -18dBm
    TXPOWER_10mW,           // -13dBm
    TXPOWER_30mW,           // - 5dBm
    TXPOWER_100mW,          //   0dBm
    TXPOWER_150mW,          // + 4dBm
    TXPOWER_LAST,
};

enum RF_MODE {
    RF_IDLE = 0,
    RF_TX,
    RF_RX,
};

enum TX_CHIP {
    TX_NRF24L01 = 1,
    TX_A7105,
    TX_CYRF6936,
    TX_CC2500
};

#define PIN_CC_CSN      PB4
#define CC_CS_HI()      digitalWrite(PIN_CC_CSN, HIGH);
#define CC_CS_LO()      digitalWrite(PIN_CC_CSN, LOW);

#define PIN_CSN_7105    PB5
#define PIN_NRF_CSN     PB3
#define PIN_CYRF_CSN    PA15
#define PIN_CYRF_RESET  PA8

// RFX2401 TX/RX switch, high active
#define PIN_TXEN        PB0
#define PIN_RXEN        PB1

// 4052 RF switch
#define PIN_RF_PE1      PB7
#define PIN_RF_PE2      PB6


class DeviceCommon
{
public:

    DeviceCommon();
    ~DeviceCommon();

    void setRFMode(enum RF_MODE mode, u8 skipImpl=FALSE);
    void setRFSwitch(enum TX_CHIP chip);

    virtual void setRFModeImpl(enum RF_MODE mode) = 0;

protected:
    SPIClass    *mSPI;

private:
    void init(void);
};


#endif
