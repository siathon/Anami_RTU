#include "mbed.h"
#include "TFT_ILI9163C_BASE.h"

// from http://www.keil.com/forum/21105/bitband-macro-to-get-bit-number-from-bit-mask/
#define MASK_TO_BITNUM(x)   (x&0x1?0:x&0x2?1:x&0x4?2:x&0x8?3:\
    x&0x10?4:x&0x20?5:x&0x40?6:x&0x80?7:\
    x&0x100?8:x&0x200?9:x&0x400?10:x&0x800?11:\
    x&0x1000?12:x&0x2000?13:x&0x4000?14:x&0x8000?15:\
    x&0x10000?16:x&0x20000?17:x&0x40000?18:x&0x80000?19:\
    x&0x100000?20:x&0x200000?21:x&0x400000?22:x&0x800000?23:\
    x&0x1000000?24:x&0x2000000?25:x&0x4000000?26:x&0x8000000?27:\
    x&0x10000000?28:x&0x20000000?29:x&0x40000000?30:x&0x80000000?31:32)
    
#define BITBAND_PERIPH(addr, bit) \
    (volatile uint32_t*)(PERIPH_BB_BASE+((uint32_t)addr-PERIPH_BASE)*32+(bit*4))


class TFT_ILI9163C : public TFT_ILI9163C_BASE {

 public:

    TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc, PinName reset);
    TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc);
    
 private:

    virtual void    init(PinName cs, PinName dc);
    virtual void    selectSlave();
    virtual void    deselectSlave();
    virtual void    setCommandMode();
    virtual void    setDataMode();
    virtual void    writecommand(uint8_t c);
    virtual void    writedata(uint8_t d);
    virtual void    writedata16(uint16_t d);
    virtual void    writedata32(uint16_t d1, uint16_t d2);
    virtual void    writedata16burst(uint16_t d, int32_t len);
    
    GPIO_TypeDef *cs_port_reg;
    volatile uint32_t cs_reg_mask;
    GPIO_TypeDef *dc_port_reg;
    volatile uint32_t dc_reg_mask;
    
    // peripheral bit-band addresses
//  volatile uint32_t *bb_cs_port;
//  volatile uint32_t *bb_dc_port;
    volatile uint32_t *bb_spi_txe;
    volatile uint32_t *bb_spi_bsy;
    volatile uint32_t *bb_spi_spe;
    volatile uint32_t *bb_spi_dff;
    
    void    waitSpiFree();
    void    waitBufferFree();
    void    set8bitMode();
    void    set16bitMode();
};