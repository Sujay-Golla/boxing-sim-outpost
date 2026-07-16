/**
 * TFT_eSPI_User_Setup.h
 * ---------------------
 * Copy this file to:
 *   <Arduino sketchbook>/libraries/TFT_eSPI/User_Setup.h
 *
 * Or merge these #defines into your existing User_Setup.h.
 * Select the driver that matches your 2.8" module (ILI9341 is most common).
 */

#define USER_SETUP_ID 320_RP2040_BOXING

#define ILI9341_DRIVER

// RP2040 SPI0 — matches config.h pin map
#define TFT_MISO 20
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS   21
#define TFT_DC   22
#define TFT_RST  26

// No touch screen on this build
// #define TOUCH_CS  -1

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT

// SPI frequency — 27 MHz is reliable for short harnesses on RP2040
#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000

// Color order: try TFT_RGB first; swap to TFT_BGR if red/blue are flipped
#define TFT_RGB_ORDER TFT_RGB
