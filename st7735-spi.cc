//simple ST7735 spi lcd library.

/*
 * Author: eggman <skouhei@gmail.com>
 *
 * Credits to Adafruit.
 * Based on Adafruit ST7735 library, see original license in license.txt file.
 */

#include  <inttypes.h>

#include "gpio_api.h"
#include "spi_api.h"

#include "st7735-spi.h"

/*** platform depend start ***/

#define SPI0_MOSI  PC_2
#define SPI0_MISO  PC_3
#define SPI0_SCLK  PC_1
#define SPI0_CS    PC_0

void ST7735SPI::platformInit() {

    gpio_init(&gpio_dc,(PinName) _rs);
    gpio_dir(&gpio_dc,PIN_OUTPUT);
    gpio_mode(&gpio_dc, PullUp);

    gpio_init(&gpio_rst,(PinName) _rst);
    gpio_dir(&gpio_rst,PIN_OUTPUT);
    gpio_mode(&gpio_dc, PullUp);

    spi_init(&spi_master, SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
    spi_format(&spi_master, 8, 0 , 0);
    spi_frequency(&spi_master, 20*1000*1000);
}


inline void ST7735SPI::spiwrite(uint8_t c) {
    spi_master_write(&spi_master, (int) c);
}


inline void ST7735SPI::dcHIGH() {
    gpio_write(&gpio_dc, 1);
}


inline void ST7735SPI::dcLOW() {
    gpio_write(&gpio_dc, 0);
}


inline void ST7735SPI::rstHIGH() {
    gpio_write(&gpio_rst, 1);
}


inline void ST7735SPI::rstLOW() {
    gpio_write(&gpio_rst, 0);
}


void ST7735SPI::mdelay(int ms) {
    HalDelayUs(ms*1000);
}

/*** platform depend end   ***/


#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

void ST7735SPI::writecommand(uint8_t c) {
    dcLOW();
    spiwrite(c);
}


void ST7735SPI::writedata(uint8_t d) {
    dcHIGH();
    spiwrite(d);
}


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
Bcmd[] = {                    // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
    50,                       //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
    255,                      //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
    0x05,                     //     16-bit color
    10,                       //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
    0x00,                     //     fastest refresh
    0x06,                     //     6 lines front porch
    0x03,                     //     3 lines back porch
    10,                       //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
    0x08,                     //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
    0x15,                     //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
    0x02,                     //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
    0x0,                      //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
    0x02,                     //     GVDD = 4.7V
    0x70,                     //     1.0uA
    10,                       //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
    0x05,                     //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
    0x01,                     //     Opamp current small
    0x02,                     //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
    0x3C,                     //     VCOMH = 4V
    0x38,                     //     VCOML = -1.1V
    10,                       //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
    0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
    0x09, 0x16, 0x09, 0x20,   //     (seriously though, not sure what
    0x21, 0x1B, 0x13, 0x19,   //      these config values represent)
    0x17, 0x15, 0x1E, 0x2B,
    0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
    0x0B, 0x14, 0x08, 0x1E,   //     (ditto)
    0x22, 0x1D, 0x18, 0x1E,
    0x1B, 0x1A, 0x24, 0x2B,
    0x06, 0x06, 0x02, 0x0F,
    10,                       //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
    0x00, 0x02,               //     XSTART = 2
    0x00, 0x81,               //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
    0x00, 0x02,               //     XSTART = 1
    0x00, 0x81,               //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
    10,                       //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
    255 };                    //     255 = 500 ms delay

static const uint8_t PROGMEM
Rcmd1[] = {                   // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
    150,                      //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
    255,                      //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
    0x01, 0x2C, 0x2D,         //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
    0x01, 0x2C, 0x2D,         //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
    0x01, 0x2C, 0x2D,         //     Dot inversion mode
    0x01, 0x2C, 0x2D,         //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
    0x07,                     //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
    0xA2,
    0x02,                     //     -4.6V
    0x84,                     //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
    0xC5,                     //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
    0x0A,                     //     Opamp current small
    0x00,                     //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
    0x8A,                     //     BCLK/2, Opamp current small & Medium low
    0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
    0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
    0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
    0xC8,                     //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
    0x05 };                   //     16-bit color

static const uint8_t PROGMEM
Rcmd2green[] = {              // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
    0x00, 0x02,               //     XSTART = 0
    0x00, 0x7F+0x02,          //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
    0x00, 0x01,               //     XSTART = 0
    0x00, 0x9F+0x01 };        //     XEND = 159

static const uint8_t PROGMEM
Rcmd2red[] = {                // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
    0x00, 0x00,               //     XSTART = 0
    0x00, 0x7F,               //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
    0x00, 0x00,               //     XSTART = 0
    0x00, 0x9F };             //     XEND = 159

static const uint8_t PROGMEM
Rcmd2green144[] = {           // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
    0x00, 0x00,               //     XSTART = 0
    0x00, 0x7F,               //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
    0x00, 0x00,               //     XSTART = 0
    0x00, 0x7F };             //     XEND = 127

static const uint8_t PROGMEM
Rcmd3[] = {                   // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
    0x02, 0x1c, 0x07, 0x12,
    0x37, 0x32, 0x29, 0x2d,
    0x29, 0x25, 0x2B, 0x39,
    0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
    0x03, 0x1d, 0x07, 0x06,
    0x2E, 0x2C, 0x29, 0x2D,
    0x2E, 0x2E, 0x37, 0x3F,
    0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
    10,                       //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
    100 };                    //     100 ms delay



// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7735SPI::commandList(const uint8_t *addr) {

    uint8_t  numCommands, numArgs;
    uint16_t ms;

    numCommands = pgm_read_byte(addr++);       // Number of commands to follow
    while(numCommands--) {                     // For each command...
        writecommand(pgm_read_byte(addr++));   //   Read, issue command
        numArgs  = pgm_read_byte(addr++);      //   Number of args to follow
        ms       = numArgs & DELAY;            //   If hibit set, delay follows args
        numArgs &= ~DELAY;                     //   Mask out delay bit
        while(numArgs--) {                     //   For each argument...
            writedata(pgm_read_byte(addr++));  //     Read, issue argument
        }

        if(ms) {
            ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
            if(ms == 255) ms = 500;     // If 255, delay for 500 ms
            mdelay(ms);
        }
    }
}


// Initialization code common to both 'B' and 'R' type displays
void ST7735SPI::commonInit(const uint8_t *cmdList) {
    colstart  = rowstart = 0; // May be overridden in init func

    platformInit();

    dcHIGH();

    rstHIGH();
    mdelay(500);
    rstLOW();
    mdelay(500);
    rstHIGH();
    mdelay(500);

    if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void ST7735SPI::initB(void) {
    commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void ST7735SPI::initR(uint8_t options) {
    commonInit(Rcmd1);
    if(options == INITR_GREENTAB) {
        commandList(Rcmd2green);
        colstart = 0;
        rowstart = 0;
    } else if(options == INITR_144GREENTAB) {
        _height = ST7735_TFTHEIGHT_144;
        commandList(Rcmd2green144);
        colstart = 2;
        rowstart = 3;
    } else {
        // colstart, rowstart left at default '0' values
        commandList(Rcmd2red);
    }
    commandList(Rcmd3);

    // if black, change MADCTL color filter
    if (options == INITR_BLACKTAB) {
        writecommand(ST7735_MADCTL);
        writedata(0xC0);
    }

    tabcolor = options;
}


ST7735SPI::ST7735SPI(int8_t cs, int8_t rs, int8_t rst) {
    _cs   = cs;
    _rs   = rs;
    _rst  = rst;
    _sid  = _sclk = 0;

    _width = ST7735_TFTWIDTH;
    _height = ST7735_TFTHEIGHT_18;

    cursor_x = 0;
    cursor_y = 0;
    rotation = 0;
}


void ST7735SPI::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    writecommand(ST7735_CASET); // Column addr set
    dcHIGH();
    spiwrite(0x00);
    spiwrite(x0+colstart);     // XSTART 
    spiwrite(0x00);
    spiwrite(x1+colstart);     // XEND

    writecommand(ST7735_RASET); // Row addr set
    dcHIGH();
    spiwrite(0x00);
    spiwrite(y0+rowstart);     // YSTART
    spiwrite(0x00);
    spiwrite(y1+rowstart);     // YEND

    writecommand(ST7735_RAMWR); // write to RAM
}


void ST7735SPI::drawPixel(int16_t x, int16_t y, uint16_t color) {

    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x,y,x+1,y+1);

    dcHIGH();
    spiwrite(color >> 8);
    spiwrite(color);

}


void ST7735SPI::fillScreen(uint16_t color) {
    fillRect(0, 0,  _width, _height, color);
}


void ST7735SPI::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {

    // rudimentary clipping (drawChar w/big text requires this)
    if((x >= _width) || (y >= _height)) return;
    if((x + w - 1) >= _width)  w = _width  - x;
    if((y + h - 1) >= _height) h = _height - y;

    setAddrWindow(x, y, x+w-1, y+h-1);

    uint8_t hi = color >> 8, lo = color;

    dcHIGH();
    for(y=h; y>0; y--) {
        for(x=w; x>0; x--) {
            spiwrite(hi);
            spiwrite(lo);
        }
    }
}


