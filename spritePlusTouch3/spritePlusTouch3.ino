// ****************************************************************************************************
//
// SpritePlusTouch
//
// An example of controlling the capacitive touch screen for basic icon display and animation
//
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2016 - RocketRedNeck - RocketRedNeck.net
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
//
// Derived from examples provided by Limor Fried/Ladyada of Adafruit Industries for
// use of the Adafruit ILI9341 Breakout and Shield
// ----> http://www.adafruit.com/products/1651
//
// ****************************************************************************************************

#include <SPI.h>       // SPI library needed for ILI9341 - TFT display
#include <SD.h>        // SD Card interfaces
#include <Wire.h>      // this is needed for FT6206 - capcitive touch screen interfaces

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ILI9341.h> // Hardware-specific library for display
#include <Adafruit_FT6206.h>  // Cap touch screen

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();

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

TS_Point gp(10,10,0);
int16_t xPos = 10;
int16_t yPos = 10;
int16_t lastX = 0;
int16_t lastY = 0;
#define STEP_SIZE 2
int16_t xDirection = STEP_SIZE;
int16_t yDirection = STEP_SIZE;
int16_t lastDirX = xDirection;
int16_t lastDirY = yDirection;

bool landscape = false;
bool jump = false;

#define ICON_NAME "Pikachu.bmp"
#define SCALE 2

#if (SCALE == 1)
   #define VCLEAR(x,y,w,h,c) tft.drawFastVLine(x, y, h, c)
   #define HCLEAR(x,y,w,h,c) tft.drawFastHLine(x, y, w, c)
#else
   #define VCLEAR(x,y,w,h,c) tft.fillRect(x, y, w, h, c);
   #define HCLEAR(x,y,w,h,c) tft.fillRect(x, y, w, h, c);
#endif

// Pixel coordinates for columns and rows in Pikachu.bmp
// ******* NOTE *****
// ******************
// May need to re-work bmp file to ensure that there is a clear anchor point
// to allow transitions between any two icons of the same type (up-down as well as in same row)
// Right now, only the sequence in the row may line up right
uint16_t g_cols[] = 
{
   0,
  23,
  48,
  72,
  95,
 119,
 145,
 169,
 191,
 216,
 239,
 265,
 287 // end
};

uint16_t g_rows[] = 
{
    8,
   37,
   72,
  101,
  133,
  165,
  197,
  229,
  255
};

#define CR(x,y,s) g_cols[x], g_rows[y], g_cols[x+1], g_rows[y+1], s, s

#define DIM(x) (sizeof(x)/sizeof(x[0]))

uint16_t colors[] =
{
  ILI9341_BLACK,
  ILI9341_NAVY,
  ILI9341_DARKGREEN,
  ILI9341_DARKCYAN,
  ILI9341_MAROON,
  ILI9341_PURPLE,
  ILI9341_OLIVE,
  ILI9341_LIGHTGREY,
  ILI9341_DARKGREY,
  ILI9341_BLUE,
  ILI9341_GREEN,
  ILI9341_CYAN,
  ILI9341_RED,
  ILI9341_MAGENTA,
  ILI9341_YELLOW,
  ILI9341_WHITE,
  ILI9341_ORANGE,
  ILI9341_GREENYELLOW,
  ILI9341_PINK
};
uint16_t colorIndex = 0;

#define FILL_COLOR colors[colorIndex]

#define PAGE_PORTRAIT 0
#define PAGE_LANDSCAPE_CLOCKWISE 1
#define PAGE_PORTRAIT_INVERTED 2
#define PAGE_LANDSCAPE_COUNTER_CLOCKWISE 3

#define PAGE PAGE_LANDSCAPE_COUNTER_CLOCKWISE

void setup(void) 
{
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(PAGE); 
  
  xScreenSize = tft.width();
  yScreenSize = tft.height();
  
  landscape = xScreenSize > yScreenSize;
  
  // Set sensitivity of touch screen
  if (! ctp.begin(40)) 
  {  
    Serial.println("Couldn't start FT6206 touchscreen controller");
    for (;;);
  }
  
  tft.fillScreen(FILL_COLOR);
  
  if (!SD.begin(SD_CS)) 
  {
    Serial.println("failed!");
  }
  else
  {
    Serial.println("OK!");
  }
}

void colorTouch(void)
{
  if (ctp.touched()) 
  {
    // Retrieve a point  
    gp = ctp.getPoint();

  // Print out raw data from screen touch controller
  //Serial.print("X = "); Serial.print(gp.x);
  //Serial.print("\tY = "); Serial.print(gp.y);
  //Serial.print(" -> ");

  // flip it around to match the screen.
  // The display (0,0) is on the opposite corner of the touch sensing (0,0)
  gp.x = map(gp.x, 0, 240, 240, 0);
  gp.y = map(gp.y, 0, 320, 320, 0);
 
  if (landscape)
  {
    int16_t temp = gp.y;
    gp.y = gp.x;
    gp.x = temp;
    
    if (PAGE == PAGE_LANDSCAPE_COUNTER_CLOCKWISE)
    {
      gp.x = xScreenSize - gp.x;
    } 
  }
   // Print out the remapped (rotated) coordinates
  //Serial.print("("); Serial.print(gp.x);
  //Serial.print(", "); Serial.print(gp.y);
  //Serial.println(")");
  
    
   
    ++colorIndex;
    if (colorIndex >= DIM(colors))
    {
      colorIndex = 0;
    }
    tft.fillCircle(gp.x, gp.y, 10, FILL_COLOR);
    
    tft.fillScreen(FILL_COLOR);
    
    xPos = gp.x;
    yPos = gp.y;
  }
}

void move(uint16_t width, uint16_t height)
{
  lastX = xPos;
  lastY = yPos;
  
  lastDirX = xDirection;
  lastDirY = yDirection;
    
  Serial.print("X = "); Serial.print(xPos);
  Serial.print("\tY = "); Serial.print(yPos);
  Serial.print(" -> ");  
  xPos += xDirection;
  yPos += yDirection;
  
  uint16_t h = height * SCALE;
  uint16_t w = width * SCALE;

  Serial.print("("); Serial.print(xPos + h);
  Serial.print(", "); Serial.print(yPos + w);
  Serial.println(")");

  if (xPos >= (xScreenSize - (landscape?h:w)))
  {
    xDirection *= -1;
  }
  else if (xPos <= 0)
  {
    xDirection *= -1;
  }
  if (yPos >= (yScreenSize - (landscape?w:h)))
  {
    yDirection *= -1;
  }
  else if (yPos <= 0)
  {
    yDirection *= -1;
  }
  
  if (lastDirX > 0)
  {
    // x, y, w, h
    VCLEAR(lastX , lastY, lastDirX, h, FILL_COLOR);
  }
  else if (lastDirX < 0)
  {
    VCLEAR(lastX + w + lastDirX, lastY, -lastDirX, h, FILL_COLOR);
  }
  
  if (lastDirY > 0)
  {
    HCLEAR(lastX, lastY, w, lastDirY, FILL_COLOR);
  }
  else if (lastDirY < 0)
  {
    HCLEAR(lastX, lastY + h + lastDirY, w, -lastDirY, FILL_COLOR);
  }
}

void loop() 
{
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(0,(xDirection>0?5:7),SCALE));
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(1,(xDirection>0?5:7),SCALE));
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(2,(xDirection>0?5:7),SCALE));
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(1,(xDirection>0?5:7),SCALE));
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 24
uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)

void drawSpriteCostume(char *filename, uint16_t x, uint16_t y, uint16_t xCorner1, uint16_t yCorner1, uint16_t xCorner2, uint16_t yCorner2, int xx, int yy) 
{
  move(xCorner2-xCorner1, yCorner2-yCorner1);

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

        for (row = yCorner1; row <= yCorner2; row++)
        { // For each scanline...

          uint16_t rr = row - yCorner1;
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

          for (col = xCorner1; col <= xCorner2; col++) 
          {      
            uint16_t cc = col - xCorner1;
            
            if (buffidx >= sizeof(sdbuffer)) 
            { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            
            uint16_t color565 = tft.color565(r,g,b);
            if ((b == 0) &&
                (g == 152) && 
                (r == 40))
            {
              color565 = FILL_COLOR;
            }
            tft.fillRect(x + (cc * xx), y + (rr * yy), xx, yy, color565);
          }
          
        } // end scanline
        //Serial.print(F("Loaded in "));
        //Serial.print(millis() - startTime);
        //Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
  
  colorTouch();

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
