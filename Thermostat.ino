//http://www.arduino.cc/playground/Code/Time
#include "Time/TimeLib.h"
#include <Wire.h> //http://arduino.cc/en/Reference/Wire // I2C
#include <SPI.h>
#include <OneWire.h>
#include <XPT2046_Touchscreen.h>
#include "ILI9341/ILI9341.h"
#include "FS.h" // Flash SPIFF file system (3MB)

using namespace std;
// TFT
// SCK= 14, MISO=12, MOSI=13, SCS= 15
#define TFT_DC 2
#define TFT_CS 15

// One-Wire sensor interface
#define TEMP_PIN 5

#define TOUCH_CS 16

// Heater relay on Pin 4 is active low and needs to be set to high ASAP
#define HEATER_RELAY 4
#define FAN_RELAY 0
#define DISABLE_RELAY HIGH
#define ENABLE_RELAY LOW

#define SWING 2.0f

#define CMD_READTEMP 0x44
#define CMD_WRITESCRATCH 0x4E
#define CMD_READSCRATCH 0xBE

XPT2046_Touchscreen touch(TOUCH_CS);

OneWire  onewire(TEMP_PIN);  // Needs a 4.7K pull up resistor !
ILI9341 tft = ILI9341(TFT_CS, TFT_DC);

// currentTemp is latest reading from sensor
// overrideTemp is either 0 or something selected via dial on screen
uint8_t currentTemp = 0;
uint8_t overrideTemp = 0;

bool bFanButtonOn = false;
bool bHeaterButtonOn = true;

char digitFile[] = "/blX.fsf";
char dialFile[] = "/dX.fsf";
char backgroundTile[] = "/bX.fsf";

void startTempReading() {
  onewire.reset();
  onewire.skip();             // broadcast to all devices on bus
  onewire.write(CMD_READTEMP);
}

// Take temperature scratchpad data and convert to
// fahrenheit.
uint8_t parseTemperature() {
  byte data[9];
  onewire.reset();
  onewire.skip();             // broadcast to all devices on bus
  onewire.write(CMD_READSCRATCH);         // Read Scratchpad

  for (int i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = onewire.read();
  }


  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  //byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  //  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  //else if (cfg == 0x20)
  //raw = raw & ~3; // 10 bit res, 187.5 ms
  //else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  const float celsius = (float)raw / 16.0;
  return static_cast<int>(celsius * 1.8 + 32.0);
}

void setupTempSensor(void) {
  startTempReading();
  while (!onewire.read())
    ; // wait until it's done

  currentTemp = parseTemperature();

  // Start next cycle for async processing
  startTempReading();
}

// Returns fahrenheit gets updated every 700ms or so
uint8_t getTemperatureF() {
  //Serial.println("Wait for temp reading");
  // Still busy converting?
  if (!onewire.read())
    return currentTemp;
  //Serial.println("New temp found");
  uint8_t f = parseTemperature();

  // start next cycle
  startTempReading();
  return f;
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
          /*            byte len = Serial.read();
                      byte buf[len];
                      for (int i = 0; i < len; i++)
                          buf[i] = Serial.read();*/
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

// defined later on..
void fsfDrawSub(const char* filename, uint8_t x, uint16_t y, uint8_t subx, uint8_t suby, uint8_t subw, uint8_t subh);

// Index is one of the [0..20] valid dial positions
void drawDial(uint8_t index)
{
  static uint8_t previousIndex = 0;
  if (index == previousIndex) // didn't move at all
    return;

  // TODO: Get rid of rounding errors and calculate these coordinates instead
  const uint8_t dialPosXy[21 * 2] = {74, 156, 65, 144, 58, 130, 55, 115, 56, 100, 60, 85, 67, 72, 78, 61, 90, 52, 105, 47, 120, 45, 135, 47, 150, 52, 162, 61, 173, 72, 180, 85, 184, 100, 185, 115, 182, 130, 175, 144, 166, 156};

  // a) First we draw background where the old dial used to be
  // find the right frame
  backgroundTile[2] = 'a' + previousIndex;
  // look up xy coordinates
  uint8_t x = dialPosXy[(previousIndex << 1)] - 25;
  uint8_t y = dialPosXy[(previousIndex << 1) + 1] - 25;  
  fsfDrawDial(backgroundTile, x, y);

  // b) Now draw the dial at new position
  // find the right frame
  dialFile[2] = 'a' + index;
  // look up xy coordinates
  x = dialPosXy[(index << 1)] - 25;
  y = dialPosXy[(index << 1) + 1] - 25;
  fsfDrawDial(dialFile, x, y);

  previousIndex = index;
}

void drawCurrentTemp()
{
  const uint8_t iTemp = currentTemp;
  const uint8_t tens = iTemp / 10;
  const uint8_t ones = iTemp % 10;

  // update filename to match digit
  digitFile[1] = 'b'; digitFile[2] = 'l';
  digitFile[3] = '0' + tens;
  fsfDraw(digitFile, 84, 276 );

  digitFile[1] = 'b'; digitFile[2] = 'r';
  digitFile[3] = '0' + ones;
  fsfDraw(digitFile, 113, 276 );
}

void drawDesiredTemp()
{
  const uint8_t iTemp = getDesiredTemperatureF();
  const uint8_t tens = iTemp / 10;
  const uint8_t ones = iTemp % 10;

  // TODO: Generate 't' image files!

  // update filename to match digit
  digitFile[1] = 'b'; digitFile[2] = 'l';
  digitFile[3] = '0' + tens;
  fsfDraw(digitFile, 84, 90 );

  digitFile[1] = 'b'; digitFile[2] = 'r';
  digitFile[3] = '0' + ones;
  fsfDraw(digitFile, 113, 90 );
}

void setupGfx()
{
  tft.begin();

  Serial.print("FS ready?");
  if (!SPIFFS.begin()) {
    Serial.println("ERR");
  }
  Serial.println("OK");
  fsfDraw("/thermobg.fsf", 0, 0);

  // render dial in center
  TS_Point p; p.x = 120; p.y = 60;
  onDialDrag(p);

  drawDesiredTemp();
  drawCurrentTemp();

  if (!touch.begin()) {
    Serial.println("Touch ERR");
  }
}

void setup()
{
  // Heater relay is active low and needs to be set to high ASAP !
  setupRelays();

  // Serial for debugging purposes only
  Serial.begin(115200);

  // for clock & temperature
  setupTempSensor();

  setupGfx();
}

//TODO: Read this from database or current override temperature
uint8_t getDesiredTemperatureF()
{
  if (overrideTemp > 0)
    return overrideTemp;

  // Read from EPROM
  return 85;
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
  time_t t = now();

  // digital clock display of the time
  Serial.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
  Serial.print(',');
  Serial.print(day(t));
  Serial.print(',');
  Serial.print(month(t));
  Serial.print(',');
  Serial.print(year(t));
  Serial.println();
}

time_t last = millis();

void diagnosticOutput()
{
  //if ((now()-last)<2000)
  //return; // don't spam the console
  // TODO: WHY IS THIS NOT BEING CALLED?
  //Serial.print(currentTemp);
  //Serial.print(',');
  
  //printClock();
  last = now();
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

  const uint8_t desiredTemp = getDesiredTemperatureF();

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
  fsfDraw(bFanButtonOn ? "/fanon.fsf" : "/fanoff.fsf",
          151, 218);
  //fanoff.fsf TL is 151,218
}

// user pressed button
void onToggleHeater()
{
  bHeaterButtonOn = !bHeaterButtonOn;
  fsfDraw(bHeaterButtonOn ? "/heaton.fsf" : "/heatoff.fsf",
          62, 215);

  // Disabling heater also resets temperature override
  if (!bHeaterButtonOn) {
    overrideTemp = 0;
    drawDesiredTemp();
  }
  //heatoff.fsf top left corner is 62,215
}

// Has user lifted finger or are we still dragging?
bool touchReleased = true;

void processTouchScreen()
{  
  TS_Point p = touch.getPoint();
  // Recently released ?
  if (!p.isValid()) 
    return;
  
  // flip it around to match the screen.
  p.x = map(p.x, 3733, 260, 0, 240);
  p.y = map(p.y, 288, 3752, 0, 320);

  Serial.print(p.x); Serial.print("  "); Serial.println(p.y);
  
  // Toggle buttons
  if (p.y > 209) {
    // touch was handled- do not allow any more clicks until touch released
    if (!touchReleased)
      return;
    if (p.x < 150) {
      onToggleHeater();
    }
    else if (p.x > 160) {
      onToggleFan();
    }
    touchReleased = false; // touch was handled- do not allow any more clicks until touch released
    return;
  }
  // Otherwise we are on circle.

  // still dragging..
  touchReleased = false;
  onDialDrag(p);
}

void onDialDrag(TS_Point &p)
{
  // cx=120, cy=110, radius= 90px or 65px to center

  float angle = atan2(110 - p.y, p.x - 120);
  // map from [-Pi, Pi] to [0..2*PI]
  if (angle < 0)
    angle += M_PI * 2;

  float percent = angle + M_PI_4; // 1/4 PI
  if (percent > M_PI * 2)
    percent -= M_PI * 2;
  percent /= M_PI_2 * 3;
  percent = 1.0 - percent;

  if (percent < 0)
    return; // invalid - we are only using a section of the whole circle

  p.x = 65 * cos(angle) + 120; p.y = -65 * sin(angle) + 110;

  // value in [0..20] we use this to pick an image
  uint8_t newTemp = static_cast<uint8_t>(0.5f + percent * 20.0f);

  drawDial(newTemp);

  // map to [65..85] degrees
  newTemp += 65;
  if (newTemp != overrideTemp) {
    overrideTemp = newTemp;
    drawDesiredTemp();
  }
}

void loop()
{
  //Serial.println("ProcessNetwork");
  processNetwork();

  bool touched = touch.touched();

  // If there is no touch do regular temperature processing
  if (!touched) {
    touchReleased = true;

    // read temperature and update LCD if needed
    uint8_t newTemp = getTemperatureF();
    if (newTemp != currentTemp) {
      currentTemp = newTemp;
      //Serial.println("drawCurrentTemp");
      drawCurrentTemp();
    }

    //Serial.println("processHeating");
    processHeating();
    //Serial.println("diagnostic");
    diagnosticOutput();
  }
  else {
    // interactive mode - don't bother reading temperature
    processTouchScreen();
  }
}

void fsfDrawDial(const char* filename, uint8_t x, uint16_t y)
{
  File bmpFile = SPIFFS.open(filename, "r");
  // buffer for fast file reading
  uint16_t scanline[50];

  uint32_t startTime = millis();

  if (!bmpFile) {
    Serial.println("ERR-Dial");
    return;
  }

  // skip width/height field
  bmpFile.read((uint8_t*)(scanline), 4);

  // Set TFT address window to clipped image bounds
  tft.setAddrWindow(x, y, x + 50 - 1, y + 50 - 1);

  for (uint16_t row = 0; row < 50; row++) { // For each scanline...
    bmpFile.read((uint8_t*)(scanline), 50 << 1);
    tft.pushColors(scanline, 50);
  } // end scanline
  Serial.print(F("Dialtime "));
  Serial.print(millis() - startTime);
  Serial.println(" ms");
  bmpFile.close();
}

void fsfDraw(const char* filename, uint8_t x, uint16_t y)
{
  File bmpFile = SPIFFS.open(filename, "r");
  // buffer for fast file reading
  uint16_t scanline[50];

  uint16_t  w, h;
  //    uint32_t startTime = millis();

  if (!bmpFile) {
    Serial.print("ERR:");
    Serial.println(filename);
    return;
  }
  w = read16(bmpFile);
  h = read16(bmpFile);

  // Set TFT address window to clipped image bounds
  tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

  for (uint16_t row = 0; row < h; row++) { // For each scanline...
    uint16_t col = 0;
    // break up full scanline into smaller chunks
    while (col < w) {
      const uint8_t toRead = min( unsigned(w - col), sizeof(scanline) / sizeof(uint16_t));
      bmpFile.read((uint8_t*)(scanline), toRead << 1);
      tft.pushColors(scanline, toRead);
      col += toRead;
    }
  } // end scanline
  //    Serial.print(F("Loaded in "));
  //    Serial.print(millis() - startTime);
  //    Serial.println(" ms");
  bmpFile.close();
}

// draw to position x,y but only use source rectangle subx/suby/subw/subh
void fsfDrawSub(const char* filename, uint8_t x, uint16_t y,
                uint8_t subx, uint8_t suby, uint8_t subw, uint8_t subh)
{
  // buffer for fast file reading
  uint16_t scanline[48];

  uint16_t  w;
  uint32_t startTime = millis();

  File bmpFile = SPIFFS.open(filename, "r");

  if (!bmpFile) {
    Serial.print("ERR:");
    Serial.println(filename);
    return;
  }
  w = read16(bmpFile);
  read16(bmpFile); // seek past height value

  // Set TFT address window to clipped image bounds
  tft.setAddrWindow(x, y,
                    x + subw - 1, y + subh - 1);

  // For each scanline...
  for (uint16_t row = 0; row < subh; row++)
  {
    // w/h field plus sub region top left offset
    const uint32_t words = (uint32_t)2 + (suby + row) * w + subx;
    bmpFile.seek(  words << 1, SeekCur );

    // break up full scanline into smaller chunks
    uint8_t col = 0;
    while (col < subw) {
      const uint8_t toRead = min(unsigned(subw - col), sizeof(scanline) / sizeof(uint16_t));
      bmpFile.read((uint8_t*)(scanline), toRead << 1);
      tft.pushColors(scanline, toRead);
      col += toRead;
    }
  } // end scanline
  Serial.print(F("Subtime: "));
  Serial.print(millis() - startTime);
  Serial.println(" ms");
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
