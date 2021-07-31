#include "TFT_ILI9163C.h"
#if defined(__MBED_GENERIC__)

/**
 * TFT_ILI9163C library for mbed generic
 * 
 * @author Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa
 * https://github.com/sumotoy/TFT_ILI9163C
 *
 * @author modified by masuda, Masuda Naika
 */

//Serial pc(SERIAL_TX, SERIAL_RX);

//constructors
TFT_ILI9163C::TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc, PinName reset) 
    : TFT_ILI9163C_BASE(mosi, miso, sclk, cs, dc, reset) {
        
        _resetPinName = reset;
}

TFT_ILI9163C::TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc) 
    : TFT_ILI9163C_BASE(mosi, miso, sclk, cs, dc) {
        
        _resetPinName = NC;
}
#endif
