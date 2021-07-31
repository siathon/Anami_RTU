#include "TFT_ILI9163C.h"
#if defined(__F411RE_DMA_IT__)

/**
 * TFT_ILI9163C library for ST Nucleo F411RE DMA Interrupt
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


void DmaIrqHandler1() {
    tftPtr[0]->dmaIrqHandler();
}

void DmaIrqHandler2() {
    tftPtr[1]->dmaIrqHandler();
}

void DmaIrqHandler3() {
    tftPtr[2]->dmaIrqHandler();
}

void DmaIrqHandler4() {
    tftPtr[3]->dmaIrqHandler();
}

void DmaIrqHandler5() {
    tftPtr[4]->dmaIrqHandler();
}

void TFT_ILI9163C::dmaIrqHandler() {
//    if(__HAL_DMA_GET_FLAG(&hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma)) != RESET) {
//        if(__HAL_DMA_GET_IT_SOURCE(&hdma, DMA_IT_TC) != RESET) {
    if ((*dma_state_reg & dma_tc_flag_mask) != RESET) {
        if ((hdma.Instance->CR & DMA_IT_TC) != RESET) {
            // disable SPI-DMA
            *bb_spi_txdmaen = 0;
            // disable DMA
            *bb_dma_sxcr_en = 0;
            while (*bb_dma_sxcr_en);
            // Disable the transfer complete interrupt
//            __HAL_DMA_DISABLE_IT(&hdma, DMA_IT_TC);
//            hdma.Instance->CR &= ~(DMA_IT_TC);
            *bb_dma_tcie = 0;
            
            waitSpiFree();
            deselectSlave();
        }
    }
}

inline void TFT_ILI9163C::waitCsFree() {
    while ((cs_port_reg->IDR & cs_reg_mask) == 0);
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
    bb_spi_dff = BITBAND_PERIPH(&spi_ptr->CR1, MASK_TO_BITNUM(SPI_CR1_DFF));
    bb_spi_txdmaen = BITBAND_PERIPH(&spi_ptr->CR2, MASK_TO_BITNUM(SPI_CR2_TXDMAEN));

    // init DMA
    hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_DISABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma.Init.Mode = DMA_NORMAL;
    hdma.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
    if(_spi.spi == SPI_1){
        hdma.Instance = DMA2_Stream3;       // DMA2_Stream2
        hdma.Init.Channel = DMA_CHANNEL_3;  // DMA_CHANNEL_2
        dma_tc_flag_mask = DMA_LISR_TCIF3;
        dma_state_reg = &DMA2->LISR;
        dma_tc_clear_mask = DMA_LIFCR_CTCIF3;
        dma_state_clear_reg = &DMA2->LIFCR;
        NVIC_SetPriority(DMA2_Stream3_IRQn, 0);
        NVIC_SetVector(DMA2_Stream3_IRQn, (uint32_t) &DmaIrqHandler1);
        NVIC_EnableIRQ(DMA2_Stream3_IRQn);
        __DMA2_CLK_ENABLE();
        tftPtr[0] = this;
    } else if(_spi.spi == SPI_2){
        hdma.Instance = DMA1_Stream4;
        hdma.Init.Channel = DMA_CHANNEL_0;
        dma_tc_flag_mask = DMA_HISR_TCIF4;
        dma_state_reg = &DMA1->HISR;
        dma_tc_clear_mask = DMA_HIFCR_CTCIF4;
        dma_state_clear_reg = &DMA1->HIFCR;
        NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
        NVIC_SetVector(DMA1_Stream4_IRQn, (uint32_t) &DmaIrqHandler2);
        NVIC_EnableIRQ(DMA1_Stream4_IRQn);
        __DMA1_CLK_ENABLE();
        tftPtr[1] = this;
    } else if(_spi.spi == SPI_3){
        hdma.Instance = DMA1_Stream5;       // DMA1_Stream7
        hdma.Init.Channel = DMA_CHANNEL_0;  // DMA_CHANNEL0
        dma_tc_flag_mask = DMA_HISR_TCIF5;
        dma_state_reg = &DMA1->HISR;
        dma_tc_clear_mask = DMA_HIFCR_CTCIF5;
        dma_state_clear_reg = &DMA1->HIFCR;
        NVIC_SetPriority(DMA1_Stream5_IRQn, 0);
        NVIC_SetVector(DMA1_Stream5_IRQn, (uint32_t) &DmaIrqHandler3);
        NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        __DMA1_CLK_ENABLE();
        tftPtr[2] = this;
    } else if(_spi.spi == SPI_4){
        hdma.Instance = DMA2_Stream1;       // DMA2_Stream4
        hdma.Init.Channel = DMA_CHANNEL_4;  // DMA_CHANNEL_5
        dma_tc_flag_mask = DMA_LISR_TCIF1;
        dma_state_reg = &DMA2->LISR;
        dma_tc_clear_mask = DMA_LIFCR_CTCIF1;
        dma_state_clear_reg = &DMA1->LIFCR;
        NVIC_SetPriority(DMA2_Stream1_IRQn, 0);
        NVIC_SetVector(DMA2_Stream1_IRQn, (uint32_t) &DmaIrqHandler4);
        NVIC_EnableIRQ(DMA2_Stream1_IRQn);
        __DMA2_CLK_ENABLE();
        tftPtr[3] = this;
    } else if(_spi.spi == SPI_5){
        hdma.Instance = DMA2_Stream4;       // DMA2_Stream5, DMA2_Stream6
        hdma.Init.Channel = DMA_CHANNEL_2;  // DMA_CHANNEL5, DMA_CHANNEL7
        dma_tc_flag_mask = DMA_HISR_TCIF4;
        dma_state_reg = &DMA2->HISR;
        dma_tc_clear_mask = DMA_HIFCR_CTCIF4;
        dma_state_clear_reg = &DMA2->HIFCR;
        NVIC_SetPriority(DMA2_Stream4_IRQn, 0);
        NVIC_SetVector(DMA2_Stream4_IRQn, (uint32_t) &DmaIrqHandler5);
        NVIC_EnableIRQ(DMA2_Stream4_IRQn);
        __DMA2_CLK_ENABLE();
        tftPtr[4] = this;
    }
    
    HAL_DMA_Init(&hdma);

    // set SPI DR
    hdma.Instance->PAR = (uint32_t) &spi_ptr->DR;
    // set SPI MAR
    hdma.Instance->M0AR = (uint32_t) &dmaBuff;

    // set bit band addresses
    bb_dma_sxcr_en = BITBAND_PERIPH(&hdma.Instance->CR, MASK_TO_BITNUM(DMA_SxCR_EN));
    bb_dma_tcie = BITBAND_PERIPH(&hdma.Instance->CR, MASK_TO_BITNUM(DMA_SxCR_TCIE));
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
    *bb_spi_dff = 0;
    *bb_spi_spe = 1;
}

inline void TFT_ILI9163C::set16bitMode() {
    *bb_spi_spe = 0;
    *bb_spi_dff = 1;
    *bb_spi_spe = 1;
}

void TFT_ILI9163C::writecommand(uint8_t c){
    
    waitCsFree();
    
    set8bitMode();
    setCommandMode();
    selectSlave();
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    spi_ptr->DR = c;
    
    waitSpiFree();
    deselectSlave();
}

void TFT_ILI9163C::writedata(uint8_t c){
    
    waitCsFree();
    
    set8bitMode();
    setDataMode();
    selectSlave();
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    spi_ptr->DR = c;
    
    waitSpiFree();
    deselectSlave();
} 

void TFT_ILI9163C::writedata16(uint16_t d){
    
    waitCsFree();
    
    set16bitMode();
    setDataMode();
    selectSlave();
    
    SPI_TypeDef *spi_ptr = (SPI_TypeDef*) _spi.spi;
    spi_ptr->DR = d;
    
    waitSpiFree();
    deselectSlave();
}


void TFT_ILI9163C::writedata32(uint16_t d1, uint16_t d2){
    
    waitCsFree();
    
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

// use DMA, Interrupt
void TFT_ILI9163C::writedata16burst(uint16_t d, int32_t len) {
    
    len = len < 0 ? -len : len;
    
    if (len > 0) {
        waitCsFree();
        set16bitMode();
        setDataMode();
        selectSlave();
    
        // clear DMA flags
//        __HAL_DMA_CLEAR_FLAG(&hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma));
        *dma_state_clear_reg = dma_tc_clear_mask;
        
        dmaBuff = d;
        hdma.Instance->NDTR = len;
        
        /* Enable the transfer complete interrupt */
//        __HAL_DMA_ENABLE_IT(&hdma, DMA_IT_TC);
//        hdma.Instance->CR |= DMA_IT_TC;
        *bb_dma_tcie = 1;
        
        // enable DMA
        *bb_dma_sxcr_en = 1;
        // enable DMA request from SPI
        *bb_spi_txdmaen = 1;
    }
}

#endif