//simple ST7735 spi lcd library.
//sample pgogram for Realtek Ameba.

extern "C" {
#include "rtl8195a.h"
#include "PinNames.h"

#include "gpio_api.h"
#include "spi_api.h"
#include "analogin_api.h"
extern u32 HalDelayUs(u32 us);

}
#include "st7735-spi.h"

#define TFT_CS  PD_7
#define TFT_DC  PB_5
#define TFT_RST PE_5

short random(short value)
{
  analogin_t   adc;
  unsigned short adcdat;

  analogin_init(&adc, AD_2);
  adcdat = analogin_read_u16(&adc);
  return (short) (adcdat % value);
}


int main(void)
{
    int x,y,side;
    int color_hi, color_lo;

    ST7735SPI tft = ST7735SPI(TFT_CS,  TFT_DC, TFT_RST);
    tft.initR(INITR_GREENTAB);
    tft.fillScreen(ST7735_BLACK);
    tft.fillRect(16,16, 64, 64, 3255);

    while (1) {
        x = random(64);
        y = random(80);
        side = random(32);
        color_hi = random(255);
        color_lo = random(255);

        tft.fillRect(x, y, x+side, y+side, ( (color_hi<<8) + color_lo));
    }
}

