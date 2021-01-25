#include <Adafruit_GrayOLED.h>
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>

// ****************************************************************************************************
//
// Pikagotchi
//
// Tamagotchi-like game using Pokemon theme
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
// avr-objdump -t project.elf to get elf
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

uint16_t bmpWidth, bmpHeight;   // W+H in pixels, yeah, we're cheating ... mod later to use structure and reference to return information after draw

int16_t xScreenSize = 0; // Will be computed at startup
int16_t yScreenSize = 0;

int16_t xWorkSize = 0; // Will be computed at startup
int16_t yWorkSize = 0;

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

// Choose either tft or Serial for output
#define TEXT_SIZE 2
#define OUT tft

#define CLEARLN true
#define KEEPLN false

#if (OUT == tft)
   #define LINE(c, y) if(c){uint16_t ty = yWorkSize + 2 + (y<<3)*TEXT_SIZE; TCLEAR(0, ty, xWorkSize, TEXT_SIZE << 3, ILI9341_BLACK); tft.setCursor(0, ty);}
#else
   #define LINE(c, y)
#endif

#define PRINT(c,y,...) LINE(c,y);OUT.print(__VA_ARGS__)
#define PRINTLN(c,y,...) LINE(c,y);OUT.println(__VA_ARGS__)

#define ERRPRINT(...) tft.setTextColor(ILI9341_RED); PRINT(__VA_ARGS__);tft.setTextColor(ILI9341_YELLOW)
#define ERRPRINTLN(...) tft.setTextColor(ILI9341_RED); PRINTLN(__VA_ARGS__);tft.setTextColor(ILI9341_YELLOW)

#define COMMAND_BAR_SIZE 55
#define TEXT_BAR_SIZE (40*TEXT_SIZE)



bool landscape = false;

#define ICON_NAME "Pikachu.bmp"
#define SCALE 2


#define VCLEAR(x,y,w,h,c) vhclear(x,y,w,h,c)
#define HCLEAR(x,y,w,h,c) vhclear(x,y,w,h,c)
#define TCLEAR(x,y,w,h,c) tft.fillRect(x,y,w,h,c)

void vhclear(int16_t x, int16_t y, int16_t w, int16_t h, int16_t c)
{
  if ((x + w) >= xWorkSize)
  {
    w = xWorkSize - x - 1;
  }
  if ((y + h) >= yWorkSize)
  {
    h = yWorkSize - y - 1;
  }
  if ((w > 0) && (h > 0))
  {
    tft.fillRect(x,y,w,h,c);
  }
}



// Pixel coordinates for columns and rows in Pikachu.bmp
// ******* NOTE *****
// ******************
// May need to re-work bmp file to ensure that there is a clear anchor point
// to allow transitions between any two icons of the same type (up-down as well as in same row)
// Right now, only the sequence in the row may line up right
const uint16_t g_cols[] = 
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

const uint16_t g_rows[] = 
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

const uint16_t colors[] =
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

  xWorkSize = xScreenSize - COMMAND_BAR_SIZE;
  yWorkSize = yScreenSize - TEXT_BAR_SIZE;  
  
  // Set sensitivity of touch screen
  if (! ctp.begin(40)) 
  {  
    Serial.println(F("Couldn't start FT6206 touchscreen controller"));
    for (;;);
  }
  
  tft.fillScreen(FILL_COLOR);
  tft.drawFastVLine(xWorkSize, 0, yScreenSize, ILI9341_WHITE);
  tft.drawFastHLine(0, yWorkSize, xWorkSize, ILI9341_WHITE);
  
  tft.setTextColor(ILI9341_YELLOW); 
  tft.setTextSize(TEXT_SIZE);
  
  PRINTLN(CLEARLN,0,F("Pikagotchi!"));
  PRINTLN(CLEARLN,1,F("----------------------"));
  
  
  if (!SD.begin(SD_CS)) 
  {
    ERRPRINTLN(CLEARLN,0,F("SD failed!"));
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
    if ((gp.x < xWorkSize) &&
        (gp.y < yWorkSize))
    {
     
      ++colorIndex;
      if (colorIndex >= DIM(colors))
      {
        colorIndex = 0;
      }
      //tft.fillCircle(gp.x, gp.y, 1, FILL_COLOR);
      
      tft.fillRect(0,0,xWorkSize-1,yWorkSize-1,FILL_COLOR);
      
      xPos = gp.x;
      yPos = gp.y;
    }
    else
    {
    }
  }
}

void move(uint16_t width, uint16_t height)
{
  lastX = xPos;
  lastY = yPos;
  
  lastDirX = xDirection;
  lastDirY = yDirection;
      
  int16_t h = (height * SCALE) + SCALE;
  int16_t w = (width * SCALE) + SCALE;

  if ((xPos + w) >= xWorkSize)
  {
    xDirection = -SCALE;
  }
  else if (xPos <= 0)
  {
    xDirection = SCALE;
  }
  if ((yPos + h) >= yWorkSize)
  {
    yDirection = -SCALE;
  }
  else if (yPos <= 0)
  {
    yDirection = SCALE;
  }
  
  xPos += xDirection;
  yPos += yDirection;
  
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

uint32_t counter = 0;
void loop() 
{
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(0,(xDirection>0?5:7),SCALE));
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(1,(xDirection>0?5:7),SCALE));

  PRINT(CLEARLN,2,F("Steps = "));
  PRINTLN(KEEPLN,2,counter+=1);

  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(2,(xDirection>0?5:7),SCALE));
  drawSpriteCostume(ICON_NAME, xPos, yPos, CR(1,(xDirection>0?5:7),SCALE));
  
  PRINT(CLEARLN,2,F("Steps = "));
  ERRPRINTLN(KEEPLN,2,counter+=1);
  
  PRINT(CLEARLN,3,F("Time = "));
  PRINTLN(KEEPLN,3, double(millis())*1.0e-3);
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
            
            int16_t xxx = x + (cc * xx);
            int16_t yyy = y + (rr * yy);
            
            if (((xxx + xx) < xWorkSize) &&
                ((yyy + yy) < yWorkSize))
            {
               tft.fillRect(x + (cc * xx), y + (rr * yy), xx, yy, color565);
            }
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
