/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#include <stdarg.h>
#include "common.h"
#include "utils.h"


static u32 rand_seed = 0xb2c54a2ful;
// Linear feedback shift register with 32-bit Xilinx polinomial x^32 + x^22 + x^2 + x + 1
static const u32 LFSR_FEEDBACK = 0x80200003ul;
static const u32 LFSR_INTAP = 32-1;

static void update_lfsr(uint32_t *lfsr, uint8_t b)
{
    for (int i = 0; i < 8; ++i) {
        *lfsr = (*lfsr >> 1) ^ ((-(*lfsr & 1u) & LFSR_FEEDBACK) ^ ~((uint32_t)(b & 1) << LFSR_INTAP));
        b >>= 1;
    }
}

u32 rand32_r(u32 *seed, u8 update)
{
    if (!seed)
        seed = (u32*)&rand_seed;
    update_lfsr(seed, update);
    return *seed;
}

u32 rand32()
{
    return rand32_r(0, 0);
}

#if __DEBUG__
#define MAX_BUF_SIZE 512

struct ringBuf {
    volatile u8 buffer[MAX_BUF_SIZE];
    volatile u16 head;
    volatile u16 tail;
};

struct ringBuf mTxRingBuf = { {0}, 0, 0 };

static void putChar(struct ringBuf *buf, u8 data)
{
    u16 head;

    head = buf->head;
    buf->buffer[head] = data;
    if (++head >= MAX_BUF_SIZE)
        head = 0;
    buf->head = head;
}

static void putStr(struct ringBuf *buf, char *str)
{
    u16 head;

    head = buf->head;

    while(*str) {
        buf->buffer[head] = *str++;
        if (++head >= MAX_BUF_SIZE)
            head = 0;
    }

    buf->head = head;
}

void DRAIN_LOG(void)
{
    struct ringBuf *buf = &mTxRingBuf;

    u16 tail = buf->tail;
    u16 head = buf->head;
    char ch;

    if (tail == head)
        return;

    while (tail != head) {
        ch = buf->buffer[tail];
         Serial.print(ch);
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
    }
    buf->tail = tail;
}

void LOG(char *fmt, ... )
{
    char buf[128]; // resulting string limited to 128 chars

    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);

    putStr(&mTxRingBuf, buf);
}

void DUMP(char *name, u8 *data, u16 cnt)
{
    u8  i;
    u8  b;
    u16 addr = 0;

    LOG("-- %s buf size : %d -- \n", name, cnt);
    while (cnt) {
        LOG("%08x - ", addr);

        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            LOG("%02x ", b);
        }

        LOG(" : ");
        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            if ((b > 0x1f) && (b < 0x7f))
                LOG("%c", b);
            else
                LOG(".");
        }
        LOG("\n");
        data += i;
        addr += i;
        cnt  -= i;
    }
}
#endif
