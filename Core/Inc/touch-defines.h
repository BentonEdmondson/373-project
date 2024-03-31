#ifndef INC_TOUCH_DEFINES_H_
#define INC_TOUCH_DEFINES_H_


#define HX8357D 0xD ///< Our internal const for D type
#define HX8357B 0xB ///< Our internal const for B type

#define HX8357_TFTWIDTH 320  ///< 320 pixels wide
#define HX8357_TFTHEIGHT 480 ///< 480 pixels tall

#define HX8357_NOP 0x00     ///< No op
#define HX8357_SWRESET 0x01 ///< software reset
#define HX8357_RDDID 0x04   ///< Read ID
#define HX8357_RDDST 0x09   ///< (unknown)

#define HX8357_RDPOWMODE 0x0A ///< Read power mode Read power mode
#define HX8357_RDMADCTL 0x0B  ///< Read MADCTL
#define HX8357_RDCOLMOD 0x0C  ///< Column entry mode
#define HX8357_RDDIM 0x0D     ///< Read display image mode
#define HX8357_RDDSDR 0x0F    ///< Read dosplay signal mode

#define HX8357_SLPIN 0x10  ///< Enter sleep mode
#define HX8357_SLPOUT 0x11 ///< Exit sleep mode
#define HX8357B_PTLON 0x12 ///< Partial mode on
#define HX8357B_NORON 0x13 ///< Normal mode

#define HX8357_INVOFF 0x20  ///< Turn off invert
#define HX8357_INVON 0x21   ///< Turn on invert
#define HX8357_DISPOFF 0x28 ///< Display on
#define HX8357_DISPON 0x29  ///< Display off

#define HX8357_CASET 0x2A ///< Column addr set
#define HX8357_PASET 0x2B ///< Page addr set
#define HX8357_RAMWR 0x2C ///< Write VRAM
#define HX8357_RAMRD 0x2E ///< Read VRAm

#define HX8357B_PTLAR 0x30   ///< (unknown)
#define HX8357_TEON 0x35     ///< Tear enable on
#define HX8357_TEARLINE 0x44 ///< (unknown)
#define HX8357_MADCTL 0x36   ///< Memory access control
#define HX8357_COLMOD 0x3A   ///< Color mode

#define HX8357_SETOSC 0xB0      ///< Set oscillator
#define HX8357_SETPWR1 0xB1     ///< Set power control
#define HX8357B_SETDISPLAY 0xB2 ///< Set display mode
#define HX8357_SETRGB 0xB3      ///< Set RGB interface
#define HX8357D_SETCOM 0xB6     ///< Set VCOM voltage

#define HX8357B_SETDISPMODE 0xB4 ///< Set display mode
#define HX8357D_SETCYC 0xB4      ///< Set display cycle reg
#define HX8357B_SETOTP 0xB7      ///< Set OTP memory
#define HX8357D_SETC 0xB9        ///< Enable extension command

#define HX8357B_SET_PANEL_DRIVING 0xC0 ///< Set panel drive mode
#define HX8357D_SETSTBA 0xC0           ///< Set source option
#define HX8357B_SETDGC 0xC1            ///< Set DGC settings
#define HX8357B_SETID 0xC3             ///< Set ID
#define HX8357B_SETDDB 0xC4            ///< Set DDB
#define HX8357B_SETDISPLAYFRAME 0xC5   ///< Set display frame
#define HX8357B_GAMMASET 0xC8          ///< Set Gamma correction
#define HX8357B_SETCABC 0xC9           ///< Set CABC
#define HX8357_SETPANEL 0xCC           ///< Set Panel

#define HX8357B_SETPOWER 0xD0     ///< Set power control
#define HX8357B_SETVCOM 0xD1      ///< Set VCOM
#define HX8357B_SETPWRNORMAL 0xD2 ///< Set power normal

#define HX8357B_RDID1 0xDA ///< Read ID #1
#define HX8357B_RDID2 0xDB ///< Read ID #2
#define HX8357B_RDID3 0xDC ///< Read ID #3
#define HX8357B_RDID4 0xDD ///< Read ID #4

#define HX8357D_SETGAMMA 0xE0 ///< Set Gamma

#define HX8357B_SETGAMMA 0xC8        ///< Set Gamma
#define HX8357B_SETPANELRELATED 0xE9 ///< Set panel related

// Plan is to move this to GFX header (with different prefix), though
// defines will be kept here for existing code that might be referencing
// them. Some additional ones are in the ILI9341 lib -- add all in GFX!
// Color definitions
#define HX8357_BLACK 0x0000   ///< BLACK color for drawing graphics
#define HX8357_BLUE 0x001F    ///< BLUE color for drawing graphics
#define HX8357_RED 0xF800     ///< RED color for drawing graphics
#define HX8357_GREEN 0x07E0   ///< GREEN color for drawing graphics
#define HX8357_CYAN 0x07FF    ///< CYAN color for drawing graphics
#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
#define HX8357_YELLOW 0xFFE0  ///< YELLOW color for drawing graphics
#define HX8357_WHITE 0xFFFF   ///< WHITE color for drawing graphics
#endif /* INC_TOUCH_DEFINES_H_ */
