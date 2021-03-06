/* Touchscreen library for XPT2046 Touch Controller Chip
 * Copyright (c) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "XPT2046_Touchscreen.h"

#define Z_THRESHOLD     400
#define MSEC_THRESHOLD  3
#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)

XPT2046_Touchscreen::XPT2046_Touchscreen(uint8_t cs)
{
	csPin = cs;
	msraw = 0x80000000;
	xraw = 0;
	yraw = 0;
	zraw = 0;
}

bool XPT2046_Touchscreen::begin()
{
	SPI.begin();
	pinMode(csPin, OUTPUT);
	digitalWrite(csPin, HIGH);
	return true;
}

TS_Point XPT2046_Touchscreen::getPoint()
{
	update();
	return TS_Point(xraw, yraw, zraw);
}

bool XPT2046_Touchscreen::touched()
{
	update();
	return (zraw >= Z_THRESHOLD);
}

void XPT2046_Touchscreen::readData(uint16_t *x, uint16_t *y, uint8_t *z)
{
	update();
	*x = xraw;
	*y = yraw;
	*z = zraw;
}

bool XPT2046_Touchscreen::bufferEmpty()
{
	return ((millis() - msraw) < MSEC_THRESHOLD);
}

/*
static int32_t distsq(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int16_t dx = x1 - x2;
	int16_t dy = y1 - y2;
	return (int32_t)dx * (int32_t)dx + (int32_t)dy * (int32_t)dy;
}*/

// 1 start bit, 12-bit data, but 16-bit read
#define SHR 3

#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)

const uint8_t READ_Z1 = 0b10110001;
const uint8_t READ_Z2 = 0b11000001;
const uint8_t READ_X = 0b11010001;
const uint8_t READ_Y = 0b10010001;
const uint8_t SAMPLE_COUNT = 10;

uint16_t XPT2046_Touchscreen::readUntilStable(uint8_t ctrl, uint8_t max_samples) const {
    SPI.transfer(ctrl);

    uint16_t prev = 0xffff, cur = 0xffff;
    uint8_t i = 0;
    do {
        prev = cur;
        cur = SPI.transfer16(ctrl) >> SHR;
    } while ((prev != cur) && (++i < max_samples));
    SPI.transfer16(0); //flush MISO queue
    return cur;
}

void XPT2046_Touchscreen::update()
{
	uint32_t now = millis();
	if (now - msraw < MSEC_THRESHOLD) return;
	msraw = now;
	SPI.beginTransaction(SPI_SETTING);
	digitalWrite(csPin, LOW);
#ifdef MG
    SPI.transfer(READ_Z1); //z1
    int16_t z1 = SPI.transfer16(READ_Z2) >> SHR; //z2
    int16_t z2 = SPI.transfer16(READ_X) >> SHR; //x
    xraw= SPI.transfer16(READ_Y) >> SHR; //y
    yraw= SPI.transfer16(0) >> SHR;    

    zraw = z1 + 4095 - z2;
#else 
    xraw = readUntilStable(READ_X, SAMPLE_COUNT);
    yraw = readUntilStable(READ_Y, SAMPLE_COUNT);

    uint16_t z1 = readUntilStable(READ_Z1, SAMPLE_COUNT);
    uint16_t z2 = readUntilStable(READ_Z2, SAMPLE_COUNT);
    zraw = z1 + 4095 - z2;
    //        Serial.printf("z1,z2, x, y = %d,%d,  %d, %d", z1, z2, xraw, yraw);
//        Serial.println();
#endif

    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    if (xraw >= 4095 || yraw >= 4095) {
        zraw = 0;
    }

    /*
    int16_t data[6];
    SPI.transfer(0xB1 /* Z1 );
	int16_t z1 = SPI.transfer16(0xC1 /* Z2 ) >> SHR;
	int16_t z2 = SPI.transfer16(0x91 /* X ) >> SHR;
	SPI.transfer16(0x91 /* X );  // dummy X measure, 1st is always noisy
	data[0] = SPI.transfer16(0xD1 /* Y ) >> SHR;
	data[1] = SPI.transfer16(0x91 /* X ) >> SHR; // make 3 x-y measurements
	data[2] = SPI.transfer16(0xD1 /* Y ) >> SHR;
	data[3] = SPI.transfer16(0x91 /* X ) >> SHR;
	data[4] = SPI.transfer16(0xD0 /* Y ) >> SHR;
	data[5] = SPI.transfer16(0) >> SHR;
	digitalWrite(csPin, HIGH);
	SPI.endTransaction();
	//int z = z1 + 4095 - z2;
    int z = (z2 - z1) / z1;
	if (z < 0) z = 0;
	zraw = z;
	// compute the distance between each pair of x-y measurements
	Serial.printf("p=%d,  %d,%d  %d,%d  %d,%d", zraw,
		data[0], data[1], data[2], data[3], data[4], data[5]);
	int32_t d12 = distsq(data[0], data[1], data[2], data[3]);
	int32_t d23 = distsq(data[2], data[3], data[4], data[5]);
	int32_t d13 = distsq(data[0], data[1], data[4], data[5]);
	Serial.printf("  %6d  %6d  %6d", d12, d23, d13);
	// choose the pair with the least distance and average them
	int16_t x=0, y=0;
	if (d12 < d23) { // do not use d23
		if (d13 < d12) {
			// use d13
			x = (data[0] + data[4]) >> 1;
			y = (data[1] + data[5]) >> 1;
		} else {
			// use d12
			x = (data[0] + data[2]) >> 1;
			y = (data[1] + data[3]) >> 1;
		}
	} else { // do not use d12
		if (d13 < d23) {
			// use d13
			x = (data[0] + data[4]) >> 1;
			y = (data[1] + data[5]) >> 1;
		} else {
			// use d23
			x = (data[2] + data[4]) >> 1;
			y = (data[3] + data[5]) >> 1;
		}
        Serial.printf("    %d,%d", x, y);
        Serial.println();
        if (z >= Z_THRESHOLD) {
        xraw = x;
        yraw = y;
        }
        }*/
}


