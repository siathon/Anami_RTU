#include "TFT_ILI9163C_BASE.h"
#include "mbed.h"

/**
 * TFT_ILI9163C library
 * 
 * @author Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa
 * https://github.com/sumotoy/TFT_ILI9163C
 *
 * @author modified by masuda, Masuda Naika
 */

//Serial pc(SERIAL_TX, SERIAL_RX);

//constructors
TFT_ILI9163C_BASE::TFT_ILI9163C_BASE(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc, PinName reset) 
    : Adafruit_GFX(_TFTWIDTH,_TFTHEIGHT) , SPI(mosi,miso,sclk,NC), _cs(cs, 1), _dc(dc, 0) {
        
        _resetPinName = reset;
        init(cs, dc);
}

TFT_ILI9163C_BASE::TFT_ILI9163C_BASE(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc) 
    : Adafruit_GFX(_TFTWIDTH,_TFTHEIGHT) , SPI(mosi,miso,sclk,NC), _cs(cs, 1), _dc(dc, 0) {
        
        _resetPinName = NC;
        init(cs, dc);
}


void TFT_ILI9163C_BASE::init(PinName cs, PinName dc){
}

inline void TFT_ILI9163C_BASE::selectSlave() {
    _cs = 0;
}

inline void TFT_ILI9163C_BASE::deselectSlave() {
    _cs = 1;
}

inline void TFT_ILI9163C_BASE::setCommandMode() {
    _dc = 0;
}

inline void TFT_ILI9163C_BASE::setDataMode() {
    _dc = 1;
}

void TFT_ILI9163C_BASE::writecommand(uint8_t c){
    
    setCommandMode();
    selectSlave();
    
    SPI::write(c);

    deselectSlave();
}

void TFT_ILI9163C_BASE::writedata(uint8_t c){
    
    setDataMode();
    selectSlave();
    
    SPI::write(c);
    
    deselectSlave();
} 

void TFT_ILI9163C_BASE::writedata16(uint16_t d){
    
    setDataMode();
    selectSlave();
    
    SPI::write(d >> 8);
    SPI::write(d & 0xff);
    
    deselectSlave();
}

void TFT_ILI9163C_BASE::writedata32(uint16_t d1, uint16_t d2){
    
    setDataMode();
    selectSlave();
    
    SPI::write(d1 >> 8);
    SPI::write(d1 & 0xff);
    SPI::write(d2 >> 8);
    SPI::write(d2 & 0xff);

    deselectSlave();
}

void TFT_ILI9163C_BASE::writedata16burst(uint16_t d, int32_t len) {
    
    len = len < 0 ? -len : len;
    
    if (len > 0) {
    
        setDataMode();
        selectSlave();
        
        while (len--) {
            SPI::write(d >> 8);
            SPI::write(d & 0xff);
        }
    
        deselectSlave();
    }
}


void TFT_ILI9163C_BASE::setBitrate(uint32_t n){
    SPI::frequency(n);
}

void TFT_ILI9163C_BASE::begin(void) {
    
    SPI::format(8,0);           // 8 bit spi mode 0
    SPI::frequency(15000000L);   // 5MHz
    
    if (_resetPinName != NC) {
        DigitalOut _reset(_resetPinName);
        _reset = 1;
        wait_ms(1);
        _reset = 0;
        wait_ms(2);
        _reset = 1;
        wait_ms(120);
    }

/*
7) MY:  1(bottom to top), 0(top to bottom)  Row Address Order
6) MX:  1(R to L),        0(L to R)         Column Address Order
5) MV:  1(Exchanged),     0(normal)         Row/Column exchange
4) ML:  1(bottom to top), 0(top to bottom)  Vertical Refresh Order
3) RGB: 1(BGR),            0(RGB)               Color Space
2) MH:  1(R to L),        0(L to R)         Horizontal Refresh Order
1)
0)

     MY, MX, MV, ML,RGB, MH, D1, D0
     0 | 0 | 0 | 0 | 1 | 0 | 0 | 0  //normal
     1 | 0 | 0 | 0 | 1 | 0 | 0 | 0  //Y-Mirror
     0 | 1 | 0 | 0 | 1 | 0 | 0 | 0  //X-Mirror
     1 | 1 | 0 | 0 | 1 | 0 | 0 | 0  //X-Y-Mirror
     0 | 0 | 1 | 0 | 1 | 0 | 0 | 0  //X-Y Exchange
     1 | 0 | 1 | 0 | 1 | 0 | 0 | 0  //X-Y Exchange, Y-Mirror
     0 | 1 | 1 | 0 | 1 | 0 | 0 | 0  //XY exchange
     1 | 1 | 1 | 0 | 1 | 0 | 0 | 0
*/
    _Mactrl_Data = 0;   // 0b00000000;
    _colorspaceData = __COLORSPC;//start with default data;
    
    chipInit();
}


void TFT_ILI9163C_BASE::chipInit() {
    writecommand(CMD_SWRESET);//software reset
    wait_ms(120);
    writecommand(CMD_SLPOUT);//exit sleep
    wait_ms(5);
    writecommand(CMD_PIXFMT);//Set Color Format 16bit   
    writedata(0x05);
    wait_ms(5);
    writecommand(CMD_GAMMASET);//default gamma curve 3
    writedata(0x04);//0x04
    wait_ms(1);
    writecommand(CMD_GAMRSEL);//Enable Gamma adj    
    writedata(0x01); 
    wait_ms(1);
    writecommand(CMD_NORML);
    
    writecommand(CMD_DFUNCTR);
    writedata(0xff);    // writedata(0b11111111);//
    writedata(0x06);    // writedata(0b00000110);//

    writecommand(CMD_PGAMMAC);//Positive Gamma Correction Setting
    #if defined(__GAMMASET1)
        writedata(0x36);//p1
        writedata(0x29);//p2
        writedata(0x12);//p3
        writedata(0x22);//p4
        writedata(0x1C);//p5
        writedata(0x15);//p6
        writedata(0x42);//p7
        writedata(0xB7);//p8
        writedata(0x2F);//p9
        writedata(0x13);//p10
        writedata(0x12);//p11
        writedata(0x0A);//p12
        writedata(0x11);//p13
        writedata(0x0B);//p14
        writedata(0x06);//p15
    #else
        writedata(0x3F);//p1
        writedata(0x25);//p2
        writedata(0x1C);//p3
        writedata(0x1E);//p4
        writedata(0x20);//p5
        writedata(0x12);//p6
        writedata(0x2A);//p7
        writedata(0x90);//p8
        writedata(0x24);//p9
        writedata(0x11);//p10
        writedata(0x00);//p11
        writedata(0x00);//p12
        writedata(0x00);//p13
        writedata(0x00);//p14
        writedata(0x00);//p15
    #endif

    writecommand(CMD_NGAMMAC);//Negative Gamma Correction Setting
    #if defined(__GAMMASET1)
        writedata(0x09);//p1
        writedata(0x16);//p2
        writedata(0x2D);//p3
        writedata(0x0D);//p4
        writedata(0x13);//p5
        writedata(0x15);//p6
        writedata(0x40);//p7
        writedata(0x48);//p8
        writedata(0x53);//p9
        writedata(0x0C);//p10
        writedata(0x1D);//p11
        writedata(0x25);//p12
        writedata(0x2E);//p13
        writedata(0x34);//p14
        writedata(0x39);//p15
    #else
        writedata(0x20);//p1
        writedata(0x20);//p2
        writedata(0x20);//p3
        writedata(0x20);//p4
        writedata(0x05);//p5
        writedata(0x15);//p6
        writedata(0x00);//p7
        writedata(0xA7);//p8
        writedata(0x3D);//p9
        writedata(0x18);//p10
        writedata(0x25);//p11
        writedata(0x2A);//p12
        writedata(0x2B);//p13
        writedata(0x2B);//p14
        writedata(0x3A);//p15
    #endif

    writecommand(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
    writedata(0x08);//0x0C//0x08
    writedata(0x02);//0x14//0x08
    wait_ms(1);
    writecommand(CMD_DINVCTR);//display inversion 
    writedata(0x07);
    wait_ms(1);
    writecommand(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD   
    writedata(0x0A);//4.30 - 0x0A
    writedata(0x02);//0x05
    wait_ms(1);
    writecommand(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL   
    writedata(0x02);
    wait_ms(1);
    writecommand(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML   
    writedata(0x50);//0x50
    writedata(99);//0x5b
    wait_ms(1);
    writecommand(CMD_VCOMOFFS);
    writedata(0);//0x40
    wait_ms(1);
  
    colorSpace(_colorspaceData);
    setRotation(0);
    defineScrollArea(0, 0); // top, bottom
    wait_ms(1);

    fillScreen(BLACK);
    writecommand(CMD_DISPON);//display ON 
}

/*
Colorspace selection:
0: RGB
1: GBR
*/
void TFT_ILI9163C_BASE::colorSpace(uint8_t cspace) {
    if (cspace < 1){
        _Mactrl_Data &= ~(1 << 3);  // bitClear(_Mactrl_Data,3);
    } else {
        _Mactrl_Data |= 1 << 3;     // bitSet(_Mactrl_Data,3);
    }
}


void TFT_ILI9163C_BASE::clearScreen(uint16_t color) {
    scroll(0);
    homeAddress();
    writedata16burst(color, _GRAMSIZE);
}

void TFT_ILI9163C_BASE::homeAddress() {
    setAddrWindow(0x00,0x00,_GRAMWIDTH-1,_GRAMHEIGH-1);
}


void TFT_ILI9163C_BASE::setCursor(int16_t x, int16_t y) {
    if (boundaryCheck(x,y)) return;
    setAddrWindow(0x00,0x00,x,y);
    cursor_x = x;
    cursor_y = y;
}


void TFT_ILI9163C_BASE::pushColor(uint16_t color) {
     writedata16(color);
}


void TFT_ILI9163C_BASE::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (boundaryCheck(x,y)) return;
    if ((x < 0) || (y < 0)) return;
    setAddrWindow(x,y,x+1,y+1);
    writedata16(color);
}


void TFT_ILI9163C_BASE::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    // Rudimentary clipping
    if (boundaryCheck(x,y)) return;
    if (((y + h) - 1) >= _height) h = _height-y;
    
    setAddrWindow(x,y,x,(y+h)-1);
    writedata16burst(color, h);
}

inline bool TFT_ILI9163C_BASE::boundaryCheck(int16_t x,int16_t y){
    if ((x >= _width) || (y >= _height)) return true;
    return false;
}

void TFT_ILI9163C_BASE::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Rudimentary clipping
    if (boundaryCheck(x,y)) return;
    if (((x+w) - 1) >= _width)  w = _width-x;
    
    setAddrWindow(x,y,(x+w)-1,y);
    writedata16burst(color, w);
}

void TFT_ILI9163C_BASE::fillScreen(uint16_t color) {
    clearScreen(color);
}

// fill a rectangle
void TFT_ILI9163C_BASE::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    
    if (boundaryCheck(x,y)) return;
    if (((x + w) - 1) >= _width)  w = _width  - x;
    if (((y + h) - 1) >= _height) h = _height - y;
    
    setAddrWindow(x,y,(x+w)-1,(y+h)-1);
    writedata16burst(color, w * h);
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color

uint16_t TFT_ILI9163C_BASE::Color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void TFT_ILI9163C_BASE::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    
    writecommand(CMD_CLMADRS); // Column
    
    if (rotation == 1) {
        writedata32(x0 + __OFFSET, x1 + __OFFSET);
    } else {
        writedata32(x0, x1);
    }

    writecommand(CMD_PGEADRS); // Page
    if (rotation == 0){
        writedata32(y0 + __OFFSET, y1 + __OFFSET);
    } else {
        writedata32(y0, y1);
    }

    writecommand(CMD_RAMWR); //Into RAM
}


void TFT_ILI9163C_BASE::setRotation(uint8_t m) {
    rotation = m &3; // can't be higher than 3
    switch (rotation) {
    case 0:
        _Mactrl_Data = 0x08;    // 0b00001000;
        _width  = _TFTWIDTH;
        _height = _TFTHEIGHT;//-__OFFSET;
        break;
    case 1:
        _Mactrl_Data = 0x68;    // 0b01101000;
        _width  = _TFTHEIGHT;//-__OFFSET;
        _height = _TFTWIDTH;
        break;
    case 2:
        _Mactrl_Data = 0xC8;    // 0b11001000;
        _width  = _TFTWIDTH;
        _height = _TFTHEIGHT;//-__OFFSET;
        break;
    case 3:
        _Mactrl_Data = 0xA8;    // 0b10101000;
        _width  = _TFTWIDTH;
        _height = _TFTHEIGHT;//-__OFFSET;
        break;
    }
    colorSpace(_colorspaceData);
    writecommand(CMD_MADCTL);
    writedata(_Mactrl_Data);
}


void TFT_ILI9163C_BASE::invertDisplay(bool i) {
    writecommand(i ? CMD_DINVON : CMD_DINVOF);
}

void TFT_ILI9163C_BASE::display(bool onOff) {
    if (onOff){
        writecommand(CMD_DISPON);
    } else {
        writecommand(CMD_DISPOFF);
    }
}

void TFT_ILI9163C_BASE::sleepMode(bool mode) {
    if (mode){
        if (sleep == 1) return;//already sleeping
        sleep = 1;
        writecommand(CMD_SLPIN);
        wait_ms(5);//needed
    } else {
        if (sleep == 0) return; //Already awake
        sleep = 0;
        writecommand(CMD_SLPOUT);
        wait_ms(120);//needed
    }
}


void TFT_ILI9163C_BASE::defineScrollArea(uint16_t tfa, uint16_t bfa){
    tfa += __OFFSET ;
    int16_t vsa = _GRAMHEIGH - tfa - bfa;
    if (vsa >= 0) {
        writecommand(CMD_VSCLLDEF);
        writedata16(tfa);
        writedata16(vsa);
        writedata16(bfa);
    }
}

void TFT_ILI9163C_BASE::scroll(uint16_t ssa) {
    if (ssa <= _TFTHEIGHT) {
        writecommand(CMD_VSSTADRS);
        writedata16(ssa + __OFFSET);
    }
}
