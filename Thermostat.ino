#include <Time.h> //http://www.arduino.cc/playground/Code/Time
#include <Wire.h> //http://arduino.cc/en/Reference/Wire // I2C
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include "Adafruit_ILI9341.h" // Hardware-specific library
#include <Adafruit_FT6206.h>

// The FT6206 cap touch uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();


// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK
#define TFT_DC 9
#define TFT_CS 10
#define SD_CS 4
// One-Wire sensor interface
#define TEMP_PIN A2

// Heater relay on Pin ADC3 is active low and needs to be set to high ASAP
// (Note that A6 and A7 cannot be set to OUTPUT mode)
#define HEATER_RELAY A3
#define FAN_RELAY A2
#define DISABLE_RELAY HIGH
#define ENABLE_RELAY LOW

#define SWING 2.0f

#define CMD_READTEMP 0x44
#define CMD_WRITESCRATCH 0x4E
#define CMD_READSCRATCH 0xBE

OneWire  onewire(TEMP_PIN);  // on pin A2 (needs a 4.7K pull up resistor !)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

float currentTemp = 0;

bool bFanButtonOn = false;
bool bHeaterButtonOn = true;

TS_Point dialCenter;

// Scratch RAM from Temperature sensor (9 bytes)
void readScratch(byte* dataOut) {
    onewire.reset();
    onewire.skip();             // broadcast to all devices on bus
    onewire.write(CMD_READSCRATCH);         // Read Scratchpad  

    for (int i = 0; i < 9; i++) {           // we need 9 bytes
        dataOut[i] = onewire.read();
    }
}

// 10-bit ADC temperature is fine (provide 9byte buffer)
void setSensorResolution(byte* data) {
    onewire.reset();
    onewire.skip();             // broadcast to all devices on bus
    onewire.write(CMD_WRITESCRATCH);

    // this changes ADC mode to 10-bit
    data[4] = 63;
    for (int i = 2; i < 5; i++) {           // we need bytes 2,3,4
        onewire.write(data[i]);
    }
}

// Take temperature scratchpad data and convert to
// fahrenheit. 
float parseTemperature(byte* data) {
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    //  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    //else if (cfg == 0x20) 
    raw = raw & ~3; // 10 bit res, 187.5 ms
                    //else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                    //// default is 12 bit resolution, 750 ms conversion time

    float celsius = (float)raw / 16.0;
    return celsius * 1.8 + 32.0;
}

void setupTempSensor(void) {
    byte data[9];
    readScratch(data);
    setSensorResolution(data);
}

// Returns fahrenheit- takes 200ms to read
float getTemperatureF() {
    byte data[9];

    onewire.reset();
    onewire.skip();             // broadcast to all devices on bus
    onewire.write(CMD_READTEMP);
    delay(200);     // 10-bit ADC duration

    readScratch(data);
    return parseTemperature(data);
}

// Connect Redbear BLE Mini to +5 and GND
// then connect Redbear TX to Arduino RX and vice versa
// Everything sent via Serial class goes to either USB serial or BLE module when standalone

// DS3232 breakout board pins (DIP->SO)
// 1=1, 4=2, 6=3=32kHz 8=4=VCC 10=5=INT, 12=6=RST
// 28=20=SCL 21=17=SDA 20=16=VBat 16=15=GND

void processNetwork()
{
    while (Serial.available()) {
        byte cmd = Serial.read();
        switch (cmd) {
        case 'V': {
            byte buffer[] = { 'V', 0x00, 0x00, 0x01 };
            Serial.write(buffer, sizeof(buffer));
            break;
        }
        case 'Z': {
            byte len = Serial.read();
            byte buf[len];
            for (int i = 0; i < len; i++)
                buf[i] = Serial.read();
            break;
        }
        }
    }
}

// Trigger relay to enable/disable heater
void enableHeater(bool enable)
{
    digitalWrite(HEATER_RELAY, enable ? ENABLE_RELAY : DISABLE_RELAY);
}

// Check if heater is currently on
bool isHeating()
{
    return digitalRead(HEATER_RELAY) == ENABLE_RELAY;
}

void enableFan(bool enable)
{
    digitalWrite(FAN_RELAY, enable ? ENABLE_RELAY : DISABLE_RELAY);
}

// Check if heater is currently on
bool isFanRequested()
{
    return digitalRead(FAN_RELAY) == ENABLE_RELAY;
}

void setupRelays()
{
    // Heater relay is active low and needs to be set to high ASAP !
    pinMode(HEATER_RELAY, OUTPUT);
    enableHeater(false);
    pinMode(FAN_RELAY, OUTPUT);
    enableFan(false);
}

void printError(const char* err)
{
    Serial.println(err);
}

void drawDial(int x, int y)
{
    fsfDrawDial(x - 25, y - 25);
    dialCenter.x = x; dialCenter.y = y;
}

void setupGfx()
{
    tft.begin();

    Serial.print(F("SD card?"));
    if (!SD.begin(SD_CS)) {
        Serial.println("ERR");
    }
    Serial.println("OK");

    fsfDraw("thermobg.fsf", 0, 0);

    // Dial is 50x50 big
    drawDial(100 + 25, 27 + 25);

    if (!ctp.begin(40)) { // pass in 'sensitivity' coefficient
        Serial.println(F("Touch Error"));
    }
}

void setup()
{
    // Heater relay is active low and needs to be set to high ASAP !
    setupRelays();

    // Serial is either via USB or via BLE in standalone mode
    Serial.begin(57600);

    setupGfx();

    // for clock & temperature
    setupTempSensor();
}

//TODO: Read this from database or current override temperature
float getDesiredTemperatureF()
{
    return 85.0f;
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(':');
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

void printClock(void)
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(month());
    Serial.print(' ');
    Serial.print(year());
    Serial.println();
}

void processOutput()
{
    Serial.print(currentTemp);
    Serial.println(" F  ");
    printClock();
}

// check if heater needs to be turned on / off
void processHeating()
{
    // error case
    if (currentTemp <= 0) {
        enableHeater(false);
        enableFan(false);
        printError("Invalid temperature reading");
        return;
    }

    const float desiredTemp = getDesiredTemperatureF();

    // Keep heating while less than desiredTemp but only start heating
    // when swing threshold has been reached

    if (!bHeaterButtonOn) {
        // stop heating
        enableHeater(false);
    }
    else {
        if (isHeating()) {
            if (currentTemp >= desiredTemp) {
                // stop heating
                enableHeater(false);
            }
        }
        else {
            // should we start heating?
            if ((desiredTemp - currentTemp) > SWING) {
                enableHeater(true);
            }
        }
    }
}

// user pressed button
void onToggleFan()
{
    bFanButtonOn = !bFanButtonOn;
    enableFan(bFanButtonOn); // trigger relay
    fsfDraw(bFanButtonOn ? "fanon.fsf" : "fanoff.fsf",
        151, 218);
    //fanoff.fsf TL is 151,218
}

// user pressed button
void onToggleHeater()
{
    bHeaterButtonOn = !bHeaterButtonOn;
    fsfDraw(bHeaterButtonOn ? "heaton.fsf" : "heatoff.fsf",
        62, 215);
    //heatoff.fsf top left corner is 62,215
}

void processTouchScreen()
{
    static bool touchReleased = true;
    // Wait for a touch
    if (!ctp.touched()) {
        touchReleased = true;
        return;
    }

    // TODO: Add Debouncing!!!
    TS_Point p = ctp.getPoint();
    // flip it around to match the screen.
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);

    // Toggle buttons
    if (p.y > 209) {
        // touch was handled- do not allow any more clicks until touch released
        if (!touchReleased)
            return;
        if (p.x < 160) {
            onToggleHeater();
        }
        else {
            onToggleFan();
        }
        touchReleased = false; // touch was handled- do not allow any more clicks until touch released
        return;
    }

    // Otherwise we are on circle.
    // cx=120, cy=110, radius= 90px
    //tft.fillCircle(p.x, p.y, 10, ILI9341_CYAN);

    // dial is 50x50
    drawDial(p.x, p.y);
    touchReleased = false;
}

void loop()
{
    if (!ctp.touched()) {
        currentTemp = getTemperatureF();
        processHeating();
    }
    else {
        processNetwork();
        processOutput();
        processTouchScreen();
    }
}

void fsfDrawDial(uint8_t x, uint16_t y)
{
    static File bmpFile;
    static bool initialized = false;
    // buffer for fast file reading
    uint16_t scanline[50];

    static uint16_t  w, h;
    uint16_t rgb565;
    uint32_t startTime = millis();
    if (initialized) {
        bmpFile.seek(4);
    }
    else {
        if ((bmpFile = SD.open("dial.fsf")) == NULL) {
            Serial.println(F("ERR-Dial"));
            return;
        }
        w = read16(bmpFile);
        h = read16(bmpFile);
        initialized = true;
    }

    // Set TFT address window to clipped image bounds
    tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

    for (uint16_t row = 0; row < h; row++) { // For each scanline...
        bmpFile.read(scanline, w << 1);
        tft.pushColors(scanline, w);
    } // end scanline
      //    Serial.print(F("Loaded in "));
      //    Serial.print(millis() - startTime);
      //    Serial.println(" ms");
      //bmpFile.close();  
}

void fsfDraw(const char* filename, uint8_t x, uint16_t y)
{
    File bmpFile;
    // buffer for fast file reading
    uint16_t scanline[48];

    uint16_t  w, h;
    uint16_t rgb565;
    uint32_t startTime = millis();

    if ((bmpFile = SD.open(filename)) == NULL) {
        Serial.print(F("ERR:"));
        Serial.println(filename);
        return;
    }
    w = read16(bmpFile);
    h = read16(bmpFile);

    // Set TFT address window to clipped image bounds
    tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

    for (uint16_t row = 0; row < h; row++) { // For each scanline...
        uint8_t col = 0;
        // break up full scanline into smaller chunks
        while (col < w) {
            const uint8_t toRead = min(w - col, sizeof(scanline) / sizeof(uint16_t));
            bmpFile.read(scanline, toRead << 1);
            tft.pushColors(scanline, toRead);
            col += toRead;
        }
    } // end scanline
      //    Serial.print(F("Loaded in "));
      //    Serial.print(millis() - startTime);
      //    Serial.println(" ms");
    bmpFile.close();
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File& f)
{
    uint16_t result;
    ((uint8_t*)&result)[0] = f.read(); // LSB
    ((uint8_t*)&result)[1] = f.read(); // MSB
    return result;
}


/*
// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20
void bmpDraw(const char* filename, uint8_t x, uint16_t y)
{
File bmpFile;
int bmpWidth, bmpHeight; // W+H in pixels
uint8_t bmpDepth; // Bit depth (currently must be 24)
uint32_t bmpImageoffset; // Start of image data in file
uint32_t rowSize; // Not always = bmpWidth; may have padding
uint8_t sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
uint8_t buffidx = sizeof(sdbuffer); // Current position in sdbuffer
boolean goodBmp = false; // Set to true on valid header parse
boolean flip = true; // BMP is stored bottom-to-top
int w, h, row, col;
uint8_t r, g, b;
uint32_t pos = 0, startTime = millis();

if ((x >= tft.width()) || (y >= tft.height()))
return;

//    Serial.println();
//    Serial.print(F("Loading image '"));
//    Serial.print(filename);
//    Serial.println('\'');

// Open requested file on SD card
if ((bmpFile = SD.open(filename)) == NULL) {
Serial.print(F("File not found "));
Serial.println(filename);
return;
}

// Parse BMP header
if (read16(bmpFile) == 0x4D42) { // BMP signature
//        Serial.print(F("File size: "));
//        Serial.println(
(void)read32(bmpFile);
(void)read32(bmpFile); // Read & ignore creator bytes
bmpImageoffset = read32(bmpFile); // Start of image data
//        Serial.print(F("Image Offset: "));
//        Serial.println(bmpImageoffset, DEC);
// Read DIB header
//        Serial.print(F("Header size: "));
//        Serial.println(
(void)read32(bmpFile);
bmpWidth = read32(bmpFile);
bmpHeight = read32(bmpFile);
if (read16(bmpFile) == 1) { // # planes -- must be '1'
bmpDepth = read16(bmpFile); // bits per pixel
//            Serial.print(F("Bit Depth: "));
//            Serial.println(bmpDepth);
if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

goodBmp = true; // Supported BMP format -- proceed!
//                Serial.print(F("Image size: "));
//                Serial.print(bmpWidth);
//                Serial.print('x');
//                Serial.println(bmpHeight);

// BMP rows are padded (if needed) to 4-byte boundary
rowSize = (bmpWidth * 3 + 3) & ~3;

// If bmpHeight is negative, image is in top-down order.
// This is not canon but has been observed in the wild.
if (bmpHeight < 0) {
bmpHeight = -bmpHeight;
flip = false;
}

// Crop area to be loaded
w = bmpWidth;
h = bmpHeight;
if ((x + w - 1) >= tft.width())
w = tft.width() - x;
if ((y + h - 1) >= tft.height())
h = tft.height() - y;

// Set TFT address window to clipped image bounds
tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

for (row = 0; row < h; row++) { // For each scanline...

// Seek to start of scan line.  It might seem labor-
// intensive to be doing this on every line, but this
// method covers a lot of gritty details like cropping
// and scanline padding.  Also, the seek only takes
// place if the file position actually needs to change
// (avoids a lot of cluster math in SD library).
if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
else // Bitmap is stored top-to-bottom
pos = bmpImageoffset + row * rowSize;
if (bmpFile.position() != pos) { // Need seek?
bmpFile.seek(pos);
buffidx = sizeof(sdbuffer); // Force buffer reload
}

for (col = 0; col < w; col++) { // For each pixel...
// Time to read more pixel data?
if (buffidx >= sizeof(sdbuffer)) { // Indeed
bmpFile.read(sdbuffer, sizeof(sdbuffer));
buffidx = 0; // Set index to beginning
}

// Convert pixel from BMP to TFT format, push to display
b = sdbuffer[buffidx++];
g = sdbuffer[buffidx++];
r = sdbuffer[buffidx++];
tft.pushColor(tft.color565(r, g, b));
} // end pixel
} // end scanline
Serial.print(F("Loaded in "));
Serial.print(millis() - startTime);
Serial.println(" ms");
} // end goodBmp
}
}

bmpFile.close();
if (!goodBmp)
Serial.println(F("BMP format not recognized."));
}
*/
