#include "TFT_ILI9163C.h"
#if defined(__F302R8_SOFT__)

/**
 * TFT_ILI9163C library for ST Nucleo F302R8 SOFT
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
        init(cs, dc);
}

TFT_ILI9163C::TFT_ILI9163C(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dc) 
    : TFT_ILI9163C_BASE(mosi, miso, sclk, cs, dc) {
        
        _resetPinName = NC;
        init(cs, dc);
}

void TFT_ILI9163C::init(PinName cs, PinName dc){
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
        
    uint32_t cs_port_index = (uint32_t) cs >> 4;
    uint32_t dc_port_index = (uint32_t) dc >> 4;
    
     //set cs/dc port addresses and masks
    cs_port_reg = (GPIO_TypeDef *) (GPIOA_BASE + (cs_port_index << 10));
    cs_reg_mask = 1 << ((uint32_t) cs & 0xf);
    dc_port_reg = (GPIO_TypeDef *) (GPIOA_BASE + (dc_port_index << 10));
    dc_reg_mask = 1 << ((uint32_t) dc & 0xf);
    
    // set bit band addresses
//  GPIO_TypeDef *cs_port_reg = (GPIO_TypeDef *) (GPIOA_BASE + (cs_port_index << 10));
//  GPIO_TypeDef *dc_port_reg = (GPIO_TypeDef *) (GPIOA_BASE + (dc_port_index << 10));
//  uint8_t cs_port_bit = (uint32_t) cs & 0xf;
//  uint8_t dc_port_bit = (uint32_t) dc & 0xf;
//  bb_cs_port = BITBAND_PERIPH(&cs_port_reg->ODR, cs_port_bit);
//  bb_dc_port = BITBAND_PERIPH(&dc_port_reg->ODR, dc_port_bit);

    bb_spi_txe = BITBAND_PERIPH(&spi_ptr->SR, MASK_TO_BITNUM(SPI_SR_TXE));
    bb_spi_bsy = BITBAND_PERIPH(&spi_ptr->SR, MASK_TO_BITNUM(SPI_SR_BSY));
    bb_spi_spe = BITBAND_PERIPH(&spi_ptr->CR1, MASK_TO_BITNUM(SPI_CR1_SPE));
}

inline void TFT_ILI9163C::selectSlave() {
    cs_port_reg->BSRRH = cs_reg_mask;   // Use BSRR register
}

inline void TFT_ILI9163C::deselectSlave() {
    cs_port_reg->BSRRL = cs_reg_mask;
}

inline void TFT_ILI9163C::setCommandMode() {
    dc_port_reg->BSRRH = dc_reg_mask;
}

inline void TFT_ILI9163C::setDataMode() {
    dc_port_reg->BSRRL = dc_reg_mask;
}

inline void TFT_ILI9163C::waitSpiFree() {
    while (*bb_spi_txe == 0);
    while (*bb_spi_bsy != 0);
}

inline void TFT_ILI9163C::waitBufferFree() {
    while (*bb_spi_txe == 0);
}


inline void TFT_ILI9163C::set8bitMode() {

    *bb_spi_spe = 0;
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;

    uint16_t tmp = spi_ptr->CR2;
    tmp &= ~SPI_CR2_DS;
    tmp |= SPI_DATASIZE_8BIT;
    spi_ptr->CR2 = tmp;

    *bb_spi_spe = 1;
}

inline void TFT_ILI9163C::set16bitMode() {

    *bb_spi_spe = 0;
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    
    uint16_t tmp = spi_ptr->CR2;
    tmp &= ~SPI_CR2_DS;
    tmp |= SPI_DATASIZE_16BIT;
    spi_ptr->CR2 = tmp;

    *bb_spi_spe = 1;
}

void TFT_ILI9163C::writecommand(uint8_t c){
    
    set8bitMode();
    setCommandMode();
    selectSlave();
    
    // Strange behavior :-(
    // Data packing RM0365 P.816
    // When the data frame size fits into one byte (less than or equal to 8 bits), data packing is
    // used automatically when any read or write 16-bit access is performed on the SPIx_DR register.
    
    // Force 8-bit access to the data register
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    uint8_t *p_spi_dr = (uint8_t*) &spi_ptr->DR;
    *p_spi_dr = (uint8_t) c;
    
    waitSpiFree();
    deselectSlave();
}


void TFT_ILI9163C::writedata(uint8_t c){
    
    set8bitMode();
    setDataMode();
    selectSlave();
    
    // Force 8-bit access to the data register
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    uint8_t *p_spi_dr = (uint8_t*) &spi_ptr->DR;
    *p_spi_dr = (uint8_t) c;
    
    waitSpiFree();
    deselectSlave();
} 


void TFT_ILI9163C::writedata16(uint16_t d){
    
    set16bitMode();
    setDataMode();
    selectSlave();
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    spi_ptr->DR = d;
    
    waitSpiFree();
    deselectSlave();
}


void TFT_ILI9163C::writedata32(uint16_t d1, uint16_t d2){
    
    set16bitMode();
    setDataMode();
    selectSlave();
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    spi_ptr->DR = d1;
    waitBufferFree();
    spi_ptr->DR = d2;
    
    waitSpiFree();
    deselectSlave();
}

// use software loop, fast enough :-)
void TFT_ILI9163C::writedata16burst(uint16_t d, int32_t len) {
    
    len = len < 0 ? -len : len;
    
    if (len > 0) {
        set16bitMode();
        setDataMode();
        selectSlave();
        
        SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
        while (len--) {
            waitBufferFree();
            spi_ptr->DR = d;
        }
        
        waitSpiFree();
        deselectSlave();
    }
}

#endif
