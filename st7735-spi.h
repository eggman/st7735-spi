//simple ST7735 spi lcd library.

/*
 * Author: eggman  <skouhei@gmail.com>
 *
 * Credits to Adafruit.
 * Based on Adafruit ST7735 library, see original license in license.txt file.
 */

// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB   0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1

#define ST7735_TFTWIDTH  128
// for 1.44" display
#define ST7735_TFTHEIGHT_144 128
// for 1.8" display
#define ST7735_TFTHEIGHT_18  160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF


class ST7735SPI {
    public:
        ST7735SPI(int8_t CS, int8_t RS, int8_t RST = -1);

        void initB(void),                             // for ST7735B displays
             initR(uint8_t options = INITR_GREENTAB), // for ST7735R
             fillScreen(uint16_t color),
             drawPixel(int16_t x, int16_t y, uint16_t color),
             fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    private:
        uint8_t  tabcolor;

        void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1),
             spiwrite(uint8_t d),
             writecommand(uint8_t c),
             writedata(uint8_t d),
             commandList(const uint8_t *addr),
             commonInit(const uint8_t *cmdList),
             mdelay(int ms),
             platformInit(),
             dcHIGH(),
             dcLOW(),
             rstHIGH(),
             rstLOW();

        uint32_t _cs, _rs, _sid, _sclk,
                 datapinmask, clkpinmask, cspinmask, rspinmask,
                 colstart, rowstart; // some displays need this changed

        int32_t  _rst;  // Must use signed type since a -1 sentinel is assigned.k

        int16_t  _width, _height, // Display w/h as modified by current rotation
                 cursor_x, cursor_y; 

        uint8_t  rotation;

/*** platform depend start ***/
        gpio_t gpio_dc;
        gpio_t gpio_rst;
        spi_t spi_master;
/*** platform depend end   ***/
};


