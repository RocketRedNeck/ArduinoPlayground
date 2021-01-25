/***************************************************
  This is our Bitmap drawing example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_ILI9341.h" // Hardware-specific library
#include <SPI.h>
#include <SD.h>

// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK.

#define TFT_DC 9
#define TFT_CS 10
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#define SD_CS 4

int bmpWidth, bmpHeight;   // W+H in pixels, yeah, we're cheating ... mod later to use structure and reference to return information after draw

int16_t xScreenSize = 0; // Will be computed at startup
int16_t yScreenSize = 0;

bool landscape = false;

#define ICON_NAME "Pikachu.bmp"

void setup(void) {
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(3); // 0 = default; portrait mode
                      // 1 = landscape mode; image 90 degrees clockwise
                      // 2 = portrait mode; image 180 degrees
                      // 3 = landscape mode; image 90 degrees counter clockwise
  
  xScreenSize = tft.width();
  yScreenSize = tft.height();
  
  landscape = xScreenSize > yScreenSize;
  
  tft.fillScreen(ILI9341_BLACK);
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
  }
  else
  {
    Serial.println("OK!");
  }
}

void loop() 
{
  drawSpriteCostume(ICON_NAME, 10, 10, 1, 9, 22, 31);
  //delay(1000);
  drawSpriteCostume(ICON_NAME, 10, 10, 26, 9, 45, 31);
  //delay(1000);
  drawSpriteCostume(ICON_NAME, 10, 10, 50, 9, 70, 31);
  //delay(1000);
  drawSpriteCostume(ICON_NAME, 10, 10, 26, 9, 45, 31);
  //delay(1000);
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 32
uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)

void drawSpriteCostume(char *filename, uint16_t x, uint16_t y, uint16_t xCorner1, uint16_t yCorner1, uint16_t xCorner2, uint16_t yCorner2) 
{
//  Serial.print("X: "); Serial.println(x);
//  Serial.print("Y: "); Serial.println(y);
//  Serial.print("X Corner 1: "); Serial.println(xCorner1);
//  Serial.print("X Corner 2: "); Serial.println(xCorner2);
//  Serial.print("Y Corner 1: "); Serial.println(yCorner1);
//  Serial.print("Y Corner 2: "); Serial.println(yCorner2);

  File     bmpFile;

  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  uint16_t w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  uint16_t tftw = tft.width();
  uint16_t tfth = tft.height();

  if((x >= tftw) || (y >= tfth)) return;

  //Serial.println();
  //Serial.print(F("Loading image '"));
  //Serial.print(filename);
  //Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    //Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    //Serial.print(F("File size: ")); 
    //Serial.println(
      read32(bmpFile);
    //);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    //Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    //Serial.print(F("Header size: ")); 
    //Serial.println(
      read32(bmpFile);
    //);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      //Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
//        Serial.print(F("Image size: "));
//        Serial.print(bmpWidth);
//        Serial.print('x');
//        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
//        if((x+w-1) >= tftw)  w = tftw  - x;
//        if((y+h-1) >= tfth) h = tfth - y;

        // Set TFT address window to clipped image bounds
        int x2 = x + xCorner2 - xCorner1;
        int y2 = y + yCorner2 - yCorner1;
        tft.setAddrWindow(x, y, x2, y2);
//        Serial.print("Address window created from X ");
//        Serial.print(x);
//        Serial.print(" - ");
//        Serial.print(x2);
//        Serial.print(" and Y ");
//        Serial.print(y);
//        Serial.print(" - ");
//        Serial.println(y2);

        for (row = yCorner1; row <= yCorner2; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;

          // Set starting file position to the correct column
          pos += (xCorner1 * 3);
          
          // Need seek?
          if(bmpFile.position() != pos) 
          { 
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }          

          // For each pixel...
          for (col = xCorner1; col <= xCorner2; col++) 
          {
            // Time to read more pixel data?
            //bmpFile.seek(col);
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            //if ((row >= yCorner1 && row <= yCorner2) && (col >= xCorner1 && col <= xCorner2)
            tft.pushColor(tft.color565(r,g,b));
            //Serial.print("Ploting pixel from (");
            //Serial.print(col);
            //Serial.print(",");
            //Serial.print(row);
            //Serial.println(")");
          } // end pixel
        } // end scanline
        //Serial.print(F("Loaded in "));
        //Serial.print(millis() - startTime);
        //Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
