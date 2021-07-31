#include "mbed.h"
#include "TFT_ILI9163C_BASE.h"

class TFT_ILI9163C : public TFT_ILI9163C_BASE {

 public:

    TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc, PinName reset);
    TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc);
    
};