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
#include "RFProtocol.h"
#include "utils.h"

#define abs(x) (((x) > 0) ? (x) : -(x))

u32 RFProtocol::mNextTS = 0;
RFProtocol* RFProtocol::mChild = NULL;

void RFProtocol::initVars(void)
{
    memset(mBufControls, 0, sizeof(mBufControls));
    mBufControls[CH_THROTTLE] = CHAN_MIN_VALUE;
    mTXPower  = TXPOWER_10mW;
}

RFProtocol::RFProtocol(u32 id)
{
    mNextTS  = 0;
    mProtoID = id;
    initVars();

    Timer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
    Timer2.pause();
    Timer2.setCount(0);
    Timer2.setOverflow(65530);
    Timer2.setPrescaleFactor(CYCLES_PER_MICROSECOND);
    Timer2.refresh();
}

RFProtocol::~RFProtocol()
{
    timer_disable_irq(Timer2.c_dev(), TIMER_CH1);
    Timer2.pause();
    Timer2.detachInterrupt(TIMER_CH1);
    close();
    LOG("PROTOCOL CLOSED\n");
}

void RFProtocol::handleTimerIntr(void)
{
    if (mChild) {
        Timer2.pause();
        u16 now = Timer2.getCount();
        now = now + mChild->callState(now, mNextTS);
        Timer2.setCompare(TIMER_CH1, now);
        mNextTS = now;
        Timer2.resume();
    }
}

void RFProtocol::registerCallback(RFProtocol *protocol)
{
    mChild = protocol;
}

int RFProtocol::init(u8 bind)
{
    return 0;
}

int RFProtocol::close(void)
{
    return 0;
}

int RFProtocol::reset(void)
{
    return 0;
}

int RFProtocol::setRFPower(u8 power)
{
    mTXPower = (power | 0x80);
    return 0;
}

u8 RFProtocol::getRFPower(void)
{
    return (mTXPower & 0x7f);
}

bool RFProtocol::isRFPowerUpdated(void)
{
    return (mTXPower & 0x80);
}

void RFProtocol::clearRFPowerUpdated(void)
{
    mTXPower &= 0x7f;
}

int RFProtocol::getInfo(s8 id, u8 *data)
{
    u8 size = 0;

    switch (id) {
        case INFO_ID:
            size = 4;
            *((u32*)data) = (u32)getProtoID();
            break;

        case INFO_RF_POWER:
            size = 1;
            *data = mTXPower;
            break;
    }
    return size;
}

void RFProtocol::injectControl(u8 ch, s16 val)
{
    mBufControls[ch] = val;
}

void RFProtocol::injectControls(s16 *data, int size)
{
    for (int i = 0; i < size; i++)
        mBufControls[i] = *data++;

    for (int i = size; i < MAX_CHANNEL; i++)
        mBufControls[i] = CHAN_MIN_VALUE;
}

s16 RFProtocol::getControl(u8 ch)
{
    return mBufControls[ch];
}


// Channel value is converted to 8bit values full scale
u8 RFProtocol::getControl_8b(u8 ch)
{
	return (u8) (map(getControl(ch), CHAN_MIN_VALUE, CHAN_MAX_VALUE, 0, 255));
}

// Channel value is converted to 8bit values to provided values scale
u8 RFProtocol::getControl_8b(u8 ch, u8 min, u8 max)
{
	return (u8) (map(getControl(ch), CHAN_MIN_VALUE, CHAN_MAX_VALUE, min, max));
}

// Channel value is converted sign + magnitude 8bit values
u8 RFProtocol::getControl_s8b(u8 ch)
{
	u8 ret;

	ret = getControl_8b(ch);
	return (ch < 128 ? 127 - ret : ret);
}

u8 RFProtocol::isStickMoved(u8 init)
{
    const s32 STICK_MOVEMENT = 15;   // defines when the bind dialog should be interrupted (stick movement STICK_MOVEMENT %)
    static s16 ele_start, ail_start;

    s16 ele = mBufControls[CH_ELEVATOR];
    s16 ail = mBufControls[CH_AILERON];

    if(init) {
        ele_start = ele;
        ail_start = ail;
        return 0;
    }

    s16 ele_diff = abs(ele_start - ele);
    s16 ail_diff = abs(ail_start - ail);
    return ((ele_diff + ail_diff > 2 * STICK_MOVEMENT * CHAN_MAX_VALUE / 100));
}

void RFProtocol::startState(u16 period)
{
    Timer2.setCompare(TIMER_CH1, period);
    Timer2.refresh();
    Timer2.attachInterrupt(TIMER_CH1, RFProtocol::handleTimerIntr);
    mNextTS = period;
    Timer2.resume();
}

// E A T R : deviation channel order
static const PROGMEM u8 TBL_ORDERS[4] = {
    RFProtocol::CH_ELEVATOR, RFProtocol::CH_AILERON,
    RFProtocol::CH_THROTTLE, RFProtocol::CH_RUDDER };

s16 RFProtocol::getControlByOrder(u8 ch)
{
    if (ch < 4)
        ch = pgm_read_byte(TBL_ORDERS + ch);

    s16 val = mBufControls[ch];

    if (ch == RFProtocol::CH_AILERON || ch == RFProtocol::CH_RUDDER)
        val = -val;

    return val;
}
