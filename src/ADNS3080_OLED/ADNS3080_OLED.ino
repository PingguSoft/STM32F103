#include <stdarg.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include "ADNS3080.h"

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);


static const int _cs_pin    = PA4;
static const int _reset_pin = 0;

void printf2(char *fmt, ... )
{
    char buf[100];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

void setup() 
{
    Serial.begin(115200);
    
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV64);   // Slow speed (72 / 64 = 1.125 MHz SPI_1 speed)

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.display();
  
    if (initSensor() == false )
        Serial.println("Failed to initialise ADNS3080");
}

void loop() 
{
    Serial.println("image data --------------");
    sendData();
    Serial.println("-------------------------");
}

static byte getPixel(int x, int y, byte *buf, int stride)
{
    return *(buf + (stride * y) + x);
}

static void setPixel(int x, int y, byte pixel, byte *buf, int stride)
{
    *(buf + (stride * y) + x) = pixel;
}

static int findNearestColor(byte color) 
{
	return (color >= 0x7E) ? 255 : 0;
}

static byte plus_truncate_uchar(byte a, int b) {
	if ((a & 0xff) + b < 0)
		return 0;
	else if ((a & 0xff) + b > 255)
		return (byte)255;
	else
		return (byte)(a + b);
}

static void dither(byte *img, int width, int height)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int  pixel  = getPixel(x, y, img, width);
			int  newPix = findNearestColor(pixel); 
            setPixel(x, y, newPix, img, width);

            int err = pixel - newPix;
			int nePix;

			if (x + 1 < width) {
				nePix  = getPixel(x + 1, y + 0, img, width);
				newPix = plus_truncate_uchar(nePix, (err * 7) >> 4);
				setPixel(x + 1, y + 0, newPix, img, width);
			}
			if (y + 1 < height) {
				if (x - 1 > 0) {
					nePix = getPixel(x - 1, y + 1, img, width);
                    newPix = plus_truncate_uchar(nePix, (err * 3) >> 4);
                    setPixel(x - 1, y + 1, newPix, img, width);
				}
				nePix = getPixel(x + 0, y + 1, img, width);
                newPix = plus_truncate_uchar(nePix, (err * 5) >> 4);
				setPixel(x + 0, y + 1, newPix, img, width);

				if (x + 1 < width) {
					nePix = getPixel(x + 1, y + 1, img, width);
                    newPix = plus_truncate_uchar(nePix, (err * 1) >> 4);
					setPixel(x + 1, y + 1, newPix, img, width);
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADNS3080 SPECIFIC FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reset()
{
    if(_reset_pin == 0)
        return;

    digitalWrite(_reset_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_reset_pin, LOW);
}

byte readReg(byte address)
{
    byte result = 0, junk = 0;


    digitalWrite(_cs_pin, LOW);
    junk = SPI.transfer(address);
    delayMicroseconds(50);
    result = SPI.transfer(0x00);
    digitalWrite(_cs_pin, HIGH);

    return result;
}

boolean initSensor(void)
{
    int     retry = 0;
    byte    pid;

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    reset();

    if(retry < 3) {
        pid = readReg(ADNS3080_PRODUCT_ID);
        printf2("PID : %02x\n", pid);
        if (pid  == 0x17)
            return true;
        retry++;
    }

    return false;
}

void writeReg(byte address, byte value)
{
    byte junk = 0;

    digitalWrite(_cs_pin, LOW);
    junk = SPI.transfer(address | 0x80 );
    delayMicroseconds(50);
    junk = SPI.transfer(value);
    digitalWrite(_cs_pin, HIGH);
}

static byte rxbuf[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y + 2];
void sendData(void)
{
    int     i, j;
    int     cnt;
    boolean bFoundFirst = false;
    byte    regValue;
    byte    pixelValue;
    byte    txCmd;

    writeReg(ADNS3080_FRAME_CAPTURE, 0x83);

    // wait 3 frame periods + 10 nanoseconds for frame to be captured
    // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510
    delayMicroseconds(1510);  

    digitalWrite(_cs_pin, LOW);
    txCmd = ADNS3080_PIXEL_BURST;
    SPI.dmaTransfer(txCmd, rxbuf, sizeof(rxbuf));
    digitalWrite(_cs_pin, HIGH);

    for (i = 0; i < sizeof(rxbuf); i++) {
        if (!bFoundFirst) {
            if (rxbuf[i] & 0x40) {
                bFoundFirst = true;
                cnt = 0;
                break;
            }
        }
/*
        if (bFoundFirst) {
            pixelValue = (rxbuf[i] << 2);
            Serial.print(pixelValue, DEC);
            cnt++;
            if((cnt % ADNS3080_PIXELS_X) != 0) {
                Serial.print(",");
            } else {
                Serial.println();
            }
        }
*/        
    }

    display.clearDisplay();
    byte *img = &rxbuf[i];
    dither(img, ADNS3080_PIXELS_X, ADNS3080_PIXELS_Y);

    int color;
    for (i = 0; i < ADNS3080_PIXELS_Y; i++) {
        for (j = 0; j < ADNS3080_PIXELS_X; j++) {
            if ( *(img + i * ADNS3080_PIXELS_X + j) == 255) {
                color = WHITE;
            }
            else {
                color = BLACK;
            }
            display.drawPixel((ADNS3080_PIXELS_X - j) * 2,     (ADNS3080_PIXELS_Y - i) * 2, color);
            display.drawPixel((ADNS3080_PIXELS_X - j) * 2 + 1, (ADNS3080_PIXELS_Y - i) * 2, color);
            display.drawPixel((ADNS3080_PIXELS_X - j) * 2,     (ADNS3080_PIXELS_Y - i) * 2 + 1, color);
            display.drawPixel((ADNS3080_PIXELS_X - j) * 2 + 1, (ADNS3080_PIXELS_Y - i) * 2 + 1, color);
        }
    }
    display.display();
    reset();
}

