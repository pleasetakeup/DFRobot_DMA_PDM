#ifdef ARDUINO_SAM_ZERO
#include "DFRobot_DMA_PDM.h"
#include "wiring_private.h"
int DFRobot_DMA::_beginCount = 0;
// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}
// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}
DFRobot_DMA::DFRobot_DMA():_channelMask(0){
  memset(_descriptors, 0x00, sizeof(_descriptors));
  memset(_descriptorsWriteBack, 0x00, sizeof(_descriptorsWriteBack));
}

void DFRobot_DMA::begin(){
  if(_beginCount == 0){
      PM->AHBMASK.bit.DMAC_ = 1;
      PM->APBBMASK.bit.DMAC_ = 1;
      DMAC->CTRL.bit.CRCENABLE = 0;
      DMAC->CTRL.bit.DMAENABLE=0;
      DMAC->CTRL.bit.SWRST = 1;
      
      // configure the descriptor addresses
      DMAC->BASEADDR.bit.BASEADDR = (uint32_t)_descriptors;
      DMAC->WRBADDR.bit.WRBADDR = (uint32_t)_descriptorsWriteBack;
      
      // enable with all levels
      DMAC->CTRL.bit.LVLEN0 = 1;
      DMAC->CTRL.bit.LVLEN1 = 1;
      DMAC->CTRL.bit.LVLEN2 = 1;
      DMAC->CTRL.bit.LVLEN3 = 1;
      DMAC->CTRL.bit.DMAENABLE = 1;
  }
  _beginCount++;
}

uint8_t DFRobot_DMA::allocateChannel(){
  uint8_t channel = 0xFF;
  for(uint8_t i = 0; i < DF_DMA_NUM_CHANNELS; i++){
	  if((_channelMask &(1 << i)) == 0){
		  _channelMask |= (1 << i);
		  memset((void*)&_descriptors[i], 0x00, sizeof(_descriptors[i]));//Clear all data in the channel to 0
		  DMAC->CHID.bit.ID = i;//Record channel ID
          DMAC->CHCTRLA.bit.ENABLE = 0;
          DMAC->CHCTRLA.bit.SWRST = 1;//Channel reset
		  channel = i;
		  break;
	  }
  }
  return channel;
}

void DFRobot_DMA::setSrcAddr(uint8_t channel, void *src, uint16_t size, bool inc){
  _descriptors[channel].SRCADDR.bit.SRCADDR = (uint32_t)src;
  if(inc){
      _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
      _descriptors[channel].BTCTRL.bit.SRCINC = 1;
	  _descriptors[channel].SRCADDR.bit.SRCADDR += size;
  }else{
      _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
      _descriptors[channel].BTCTRL.bit.SRCINC = 0;
  }
  
}

void DFRobot_DMA::setDstAddr(uint8_t channel, void *dst, uint16_t size, bool inc){
  _descriptors[channel].DSTADDR.bit.DSTADDR = (uint32_t)dst;
  if(inc){
      _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
      _descriptors[channel].BTCTRL.bit.DSTINC = 1;
	  _descriptors[channel].DSTADDR.bit.DSTADDR += size;
  }else{
      _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
      _descriptors[channel].BTCTRL.bit.DSTINC = 0;
  }
  
}

void DFRobot_DMA::setRoundTransMode(bool flag){
  
}
void DFRobot_DMA::setIncMode(uint8_t channel, uint8_t mode){
  switch(mode){
	  case DF_DMA_SRCINC:
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
		   _descriptors[channel].BTCTRL.bit.SRCINC = 1;
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
		   _descriptors[channel].BTCTRL.bit.DSTINC = 0;
		   break;
	  case DF_DMA_DSTINC:
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
		   _descriptors[channel].BTCTRL.bit.SRCINC = 0;
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
		   _descriptors[channel].BTCTRL.bit.DSTINC = 1;
		   break;
	  case DF_DMA_BOTHINC:
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
		   _descriptors[channel].BTCTRL.bit.SRCINC = 1;
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
		   _descriptors[channel].BTCTRL.bit.DSTINC = 1;
		   break;
	  case DF_DMA_BOTHPAUSE:
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
		   _descriptors[channel].BTCTRL.bit.SRCINC = 0;
	       _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
		   _descriptors[channel].BTCTRL.bit.DSTINC = 0;
		   break;
  }
}

void DFRobot_DMA::setDataTransWidthAndSize(uint8_t channel, uint16_t size, uint8_t width){
  switch (width) {
    case 1:
    default:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
      break;

    case 2:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_HWORD_Val;
      break;

    case 4:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
      break;
  }
  _descriptors[channel].BTCNT.bit.BTCNT = size / width; 
}

void DFRobot_DMA::setPorityLevel(uint8_t channel, uint8_t level){
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLB.bit.LVL = level;
}

void DFRobot_DMA::setTriggerSource(uint8_t channel, int source)
{
  DMAC->CHID.bit.ID = channel;
  _descriptors[channel].DESCADDR.bit.DESCADDR = 0;
  _descriptors[channel].BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val;
  _descriptors[channel].BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
  DMAC->CHCTRLB.bit.TRIGSRC = source;

  if (DMAC->CHCTRLB.bit.TRIGSRC) {
    DMAC->CHCTRLB.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val;
  } else {
    DMAC->CHCTRLB.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BLOCK_Val;
  }
}

void DFRobot_DMA::start(uint8_t channel){
  while(_descriptorsWriteBack[channel].BTCTRL.bit.VALID); 
  _descriptors[channel].BTCTRL.bit.VALID = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}

void DFRobot_DMA::end(){
  DMAC->CTRL.bit.DMAENABLE = 0;
  PM->APBBMASK.bit.DMAC_ = 0;
  PM->AHBMASK.bit.DMAC_ = 0;
}

void DFRobot_DMA_SPI::begin(){
  DFRobot_DMA::begin();
  _channel = allocateChannel();//Distribution channel
  if(_channel == DMA_CHANNEL_NONE)
	  return;
  setPorityLevel(_channel, 0);//Set channel priority to highest
  setTriggerSource(_channel, 0x0A);
  setDataTransWidthAndSize(_channel, 0, 2);
  setDstAddr(_channel, (uint32_t *)0x42001828);
  setIncMode(_channel, DF_DMA_SRCINC);//Set incremental mode
  SPI.begin();
  sercom4.disableSPI();
  while(SERCOM4->SPI.SYNCBUSY.bit.ENABLE);
  SERCOM4->SPI.BAUD.reg = 0; 
  sercom4.enableSPI();
}
bool DFRobot_DMA_SPI::checkFlag(){
   return _descriptorsWriteBack[0].BTCTRL.bit.VALID;
}
void DFRobot_DMA_SPI::transfer(void *src, uint16_t size){
  while (_descriptorsWriteBack[0].BTCTRL.bit.VALID); 
  _descriptors[_channel].SRCADDR.bit.SRCADDR = (uint32_t)src+(uint32_t)size;
  _descriptors[0].BTCNT.bit.BTCNT = size;
  _descriptors[0].BTCTRL.bit.VALID = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}
DFRobot_DMA_SPI DMASPI;

void DFRobot_DMA_ADC::begin(uint32_t pin){
  DFRobot_DMA::begin();
  _channel = allocateChannel();//Distribution channel
  if(_channel == DMA_CHANNEL_NONE)
	  return;
  setPorityLevel(_channel, 0);//Set channel priority to highest
  setTriggerSource(_channel, 0x27);
  setDataTransWidthAndSize(_channel, 0, 2);
  
  //setDstAddr(_channel, (uint32_t *)0x4200401A);
  setSrcAddr(_channel, (uint32_t *)0x4200401A);
  setIncMode(_channel, DF_DMA_DSTINC);//Set incremental mode

  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_12BIT | 0x04;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 0x3f;                        // Set max Sampling Time Length

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)

  if (pin < A0) {
    pin += A0;
  }
  pinPeripheral(pin, PIO_ANALOG);
  
  // Disable DAC, if analogWrite() was used previously to enable the DAC
  if ((g_APinDescription[pin].ulADCChannelNumber == ADC_Channel0) || (g_APinDescription[pin].ulADCChannelNumber == DAC_Channel0)) {
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
    //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
    syncDAC();
  }

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  // Waiting for the 1st conversion to complete
  //while (ADC->INTFLAG.bit.RESRDY == 0);

  // Clear the Data Ready flag
  //ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  //syncADC();
  //ADC->SWTRIG.bit.START = 1;
}
uint32_t DFRobot_DMA_ADC::read(){

  uint32_t valueRead = 0;
  valueRead = ADC->RESULT.reg;
  return valueRead;
}
void DFRobot_DMA_ADC::start(void *dst,uint16_t size){
  //while (_descriptorsWriteBack[0].BTCTRL.bit.VALID); 
  _descriptors[_channel].DSTADDR.bit.DSTADDR = (uint32_t)dst + (uint32_t)size*2;
  _descriptors[0].BTCNT.bit.BTCNT = size;
  _descriptors[0].BTCTRL.bit.VALID = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}
bool DFRobot_DMA_ADC::checkFlag(){
	delay(1);
   ADC->RESULT.reg;
   return _descriptorsWriteBack[0].BTCTRL.bit.VALID;
}
DFRobot_DMA_ADC DMAADC;



bool DFRobot_DMA_IIS::begin(uint8_t clkPin,uint8_t dataPin)
{

  DFRobot_DMA::begin();
  _channel = allocateChannel();//Distribution channel
  if(_channel == DMA_CHANNEL_NONE)
	  return false;
  setPorityLevel(_channel, 0);//Set channel priority to highest
  setTriggerSource(_channel, 0x2A);
  setDataTransWidthAndSize(_channel, 0, 4);

  
  //setDstAddr(_channel, (uint32_t *)0x4200401A);
  setSrcAddr(_channel, (uint32_t *)0x42005034);
  setIncMode(_channel, DF_DMA_DSTINC);//Set incremental mode
  
  _clk = clkPin;
  _data = dataPin;
    _i2sclock = I2S_CLOCK_UNIT_0;
    _clk_pin = PIN_PA10G_I2S_SCK0;
    _clk_mux = MUX_PA10G_I2S_SCK0;
    _i2sserializer = I2S_SERIALIZER_1;
    _data_pin = PIN_PA08G_I2S_SD1;
    _data_mux = MUX_PA08G_I2S_SD1;
    PM->APBCMASK.reg |= PM_APBCMASK_I2S;
    _gclk = 3;
  /* Status check */
    uint32_t ctrla = I2S->CTRLA.reg;
    if (ctrla & I2S_CTRLA_ENABLE) {
      if (ctrla & (I2S_CTRLA_SEREN1 | I2S_CTRLA_SEREN0 | I2S_CTRLA_CKEN1 |
                 I2S_CTRLA_CKEN0)) {
        // return STATUS_BUSY;
      return false;
      } else {
      // return STATUS_ERR_DENIED;
       return false;
    }
  }

  return true;

}
void DFRobot_DMA_IIS::start(void *dst,uint16_t size)
{
  //while (_descriptorsWriteBack[0].BTCTRL.bit.VALID); 
  _descriptors[_channel].DSTADDR.bit.DSTADDR = (uint32_t)dst + (uint32_t)size*4;
  _descriptors[0].BTCNT.bit.BTCNT = size;
  _descriptors[0].BTCTRL.bit.VALID = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}
bool DFRobot_DMA_IIS::checkFlag()
{
  // SerialUSB.println("flag");
  delay(1);
  I2S->DATA[_i2sserializer].reg;
   return _descriptorsWriteBack[0].BTCTRL.bit.VALID;
}
uint32_t DFRobot_DMA_IIS::read()
{
  {
#if defined(ARDUINO_SAM_ZERO)
    uint32_t sync_bit, ready_bit;
    uint32_t data;
    ready_bit = I2S_INTFLAG_RXRDY0 << _i2sserializer;
    while (!(I2S->INTFLAG.reg & ready_bit)) {
      /* Wait until ready to transmit */
    }

    sync_bit = I2S_SYNCBUSY_DATA0 << _i2sserializer;
    while (I2S->SYNCBUSY.reg & sync_bit) {
      /* Wait sync */
    }
    /* Read data */
    data = I2S->DATA[_i2sserializer].reg;
    I2S->INTFLAG.reg = ready_bit;
    return data;
#endif
  }
}
bool DFRobot_DMA_IIS::configure(uint32_t sampleRateHz, boolean stereo)
{
Serial.println("1");
  while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE)
    ; // Sync wait
  I2S->CTRLA.reg &= ~I2S_SYNCBUSY_ENABLE;
Serial.println("2");
  {
#if defined(ARDUINO_SAM_ZERO)
    /* Cache new register configurations to minimize sync requirements. */
    uint32_t new_genctrl_config = (_gclk << GCLK_GENCTRL_ID_Pos);
    uint32_t new_gendiv_config = (_gclk << GCLK_GENDIV_ID_Pos);

    /* Select the requested source clock for the generator */
    // Set the clock generator to use the 48mhz main CPU clock and divide it
    // down to the SCK frequency.
    new_genctrl_config |= GCLK_SOURCE_DFLL48M << GCLK_GENCTRL_SRC_Pos;
    uint32_t division_factor =
        F_CPU / (sampleRateHz * 16); // 16 clocks for 16 stereo bits

    /* Set division factor */
    if (division_factor > 1) {
      /* Check if division is a power of two */
      if (((division_factor & (division_factor - 1)) == 0)) {
        /* Determine the index of the highest bit set to get the
         * division factor that must be loaded into the division
         * register */

        uint32_t div2_count = 0;

        uint32_t mask;
        for (mask = (1UL << 1); mask < division_factor; mask <<= 1) {
          div2_count++;
        }

        /* Set binary divider power of 2 division factor */
        new_gendiv_config |= div2_count << GCLK_GENDIV_DIV_Pos;
        new_genctrl_config |= GCLK_GENCTRL_DIVSEL;
      } else {
        /* Set integer division factor */

        new_gendiv_config |= (division_factor) << GCLK_GENDIV_DIV_Pos;

        /* Enable non-binary division with increased duty cycle accuracy */
        new_genctrl_config |= GCLK_GENCTRL_IDC;
      }
    }

    noInterrupts(); // cpu_irq_enter_critical();

    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);                                      // Wait for synchronization
  Serial.println("3");
    *((uint8_t *)&GCLK->GENDIV.reg) = _gclk; /* Select the correct generator */

    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization
    Serial.println("4");
    GCLK->GENDIV.reg = new_gendiv_config; /* Write the new generator configuration */
    Serial.println(new_gendiv_config);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization
    GCLK->GENCTRL.reg = new_genctrl_config | (GCLK->GENCTRL.reg & GCLK_GENCTRL_GENEN);
    Serial.println(new_genctrl_config);
    // Replace "system_gclk_gen_enable(_gclk);" with:

    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization
    *((uint8_t *)&GCLK->GENCTRL.reg) = _gclk; /* Select the requested generator */

    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);                                      // Wait for synchronization
    GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN; /* Enable generator */

	//SerialUSB.println(GCLK_GENCTRL_GENEN);


    interrupts(); // cpu_irq_leave_critical();
#endif
  }

  /******************************* Configure I2S clock *************/
  {
#if defined(ARDUINO_SAM_ZERO)
    /* Status check */
    /* Busy ? */
    if (I2S->SYNCBUSY.reg & (I2S_SYNCBUSY_CKEN0 << _i2sclock)) {
      return false; // return STATUS_BUSY;
    }
    /* Already enabled ? */
    if (I2S->CTRLA.reg & (I2S_CTRLA_CKEN0 << _i2sclock)) {

      return false; // return STATUS_ERR_DENIED;
    }

    /***************************** Initialize Clock Unit *************/
    uint32_t clkctrl =
        // I2S_CLKCTRL_MCKOUTINV | // mck out not inverted
        // I2S_CLKCTRL_SCKOUTINV | // sck out not inverted
        // I2S_CLKCTRL_FSOUTINV |  // fs not inverted
        // I2S_CLKCTRL_MCKEN |    // Disable MCK output
        // I2S_CLKCTRL_MCKSEL |   // Disable MCK output
        // I2S_CLKCTRL_SCKSEL |   // SCK source is GCLK
        // I2S_CLKCTRL_FSINV |    // do not invert frame sync
        // I2S_CLKCTRL_FSSEL |    // Configure FS generation from SCK clock.
        // I2S_CLKCTRL_BITDELAY |  // No bit delay (PDM)
        0;

    clkctrl |= I2S_CLKCTRL_MCKOUTDIV(0);
    clkctrl |= I2S_CLKCTRL_MCKDIV(0);
    clkctrl |= I2S_CLKCTRL_NBSLOTS(1); // STEREO is '1' (subtract one from #)
    clkctrl |= I2S_CLKCTRL_FSWIDTH(
        I2S_FRAME_SYNC_WIDTH_SLOT); // Frame Sync (FS) Pulse is 1 Slot width
    if (stereo) {
      clkctrl |= I2S_CLKCTRL_SLOTSIZE(I2S_SLOT_SIZE_16_BIT);
    } else {
      clkctrl |= I2S_CLKCTRL_SLOTSIZE(I2S_SLOT_SIZE_32_BIT);
    }
   SerialUSB.println(clkctrl);
    /* Write clock unit configurations */
    I2S->CLKCTRL[_i2sclock].reg = clkctrl;

    /* Select general clock source */
    const uint8_t i2s_gclk_ids[2] = {I2S_GCLK_ID_0, I2S_GCLK_ID_1};

    /* Cache the new config to reduce sync requirements */
    uint32_t new_clkctrl_config =
        (i2s_gclk_ids[_i2sclock] << GCLK_CLKCTRL_ID_Pos);

    /* Select the desired generic clock generator */
    new_clkctrl_config |= _gclk << GCLK_CLKCTRL_GEN_Pos;

    /* Disable generic clock channel */
    noInterrupts();

    /* Select the requested generator channel */
    *((uint8_t *)&GCLK->CLKCTRL.reg) = i2s_gclk_ids[_i2sclock];

    /* Switch to known-working source so that the channel can be disabled */
    uint32_t prev_gen_id = GCLK->CLKCTRL.bit.GEN;
    GCLK->CLKCTRL.bit.GEN = 0;

    /* Disable the generic clock */
    GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
    while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN)
      ; /* Wait for clock to become disabled */

    /* Restore previous configured clock generator */
    GCLK->CLKCTRL.bit.GEN = prev_gen_id;

    /* Write the new configuration */
    GCLK->CLKCTRL.reg = new_clkctrl_config;

    // enable it
    *((uint8_t *)&GCLK->CLKCTRL.reg) =
        i2s_gclk_ids[_i2sclock]; /* Select the requested generator channel */
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN; /* Enable the generic clock */

    interrupts();

    /* Initialize pins */
    pinPeripheral(_clk, (EPioType)_clk_mux);
#endif
  }

  /***************************** Configure I2S serializer *************/
  {
#if defined(ARDUINO_SAM_ZERO)

    /* Status check */
    /* Busy ? */
    while (I2S->SYNCBUSY.reg &
           ((I2S_SYNCBUSY_SEREN0 | I2S_SYNCBUSY_DATA0) << _i2sserializer)) {
      // return STATUS_BUSY;
      return false;
    }

    /* Already enabled ? */
    if (I2S->CTRLA.reg & (I2S_CTRLA_CKEN0 << _i2sserializer)) {
      // return STATUS_ERR_DENIED;
      return false;
    }

    /* Initialize Serializer */
    uint32_t serctrl =
        // I2S_SERCTRL_RXLOOP |    // Dont use loopback mode
        // I2S_SERCTRL_DMA    |    // Single DMA channel for all I2S channels
        // I2S_SERCTRL_MONO   |    // Dont use MONO mode
        // I2S_SERCTRL_SLOTDIS7 |  // Dont have any slot disabling
        // I2S_SERCTRL_SLOTDIS6 |
        // I2S_SERCTRL_SLOTDIS5 |
        // I2S_SERCTRL_SLOTDIS4 |
        // I2S_SERCTRL_SLOTDIS3 |
        // I2S_SERCTRL_SLOTDIS2 |
        // I2S_SERCTRL_SLOTDIS1 |
        // I2S_SERCTRL_SLOTDIS0 |
        I2S_SERCTRL_BITREV | // Do not transfer LSB first (MSB first!)
        // I2S_SERCTRL_WORDADJ  |  // Data NOT left in word
        I2S_SERCTRL_SLOTADJ | // Data is left in slot
        // I2S_SERCTRL_TXSAME   |  // Pad 0 on underrun
        0;

    // Configure clock unit to use with serializer, and set serializer as an
    // output.
    if (_i2sclock < 2) {
      serctrl |= (_i2sclock ? I2S_SERCTRL_CLKSEL : 0);
    } else {
      return false; // return STATUS_ERR_INVALID_ARG;
    }
    if (stereo) {
      serctrl |= I2S_SERCTRL_SERMODE(
          I2S_SERIALIZER_PDM2); // Serializer is used to receive PDM data on
                                // each clock edge
    } else {
      serctrl |= I2S_SERCTRL_SERMODE(I2S_SERIALIZER_RECEIVE); // act like I2S
    }

    // Configure serializer data size.
    serctrl |= I2S_SERCTRL_DATASIZE(
        I2S_DATA_SIZE_32BIT); // anything other than 32 bits is ridiculous to
                              // manage, force this to be 32

    serctrl |= I2S_SERCTRL_TXDEFAULT(
                   I2S_LINE_DEFAULT_0) | /** Output default value is 0 */
               I2S_SERCTRL_EXTEND(
                   I2S_DATA_PADDING_0); /** Padding 0 in case of under-run */

    /* Write Serializer configuration */
    I2S->SERCTRL[_i2sserializer].reg = serctrl;

    /* Initialize pins */
    // Enable SD pin.  See Adafruit_ZeroI2S.h for default pin value.
    pinPeripheral(_data, (EPioType)_data_mux);
#endif
  }

  /***************************** Enable everything configured above
   * *************/

#if defined(ARDUINO_SAM_ZERO)
  // Replace "i2s_enable(&_i2s_instance);" with:
  while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE)
    ; // Sync wait
  I2S->CTRLA.reg |= I2S_SYNCBUSY_ENABLE;

  // Replace "i2s_clock_unit_enable(&_i2s_instance, _i2sclock);" with:
  uint32_t cken_bit = I2S_CTRLA_CKEN0 << _i2sclock;
  while (I2S->SYNCBUSY.reg & cken_bit)
    ; // Sync wait
  I2S->CTRLA.reg |= cken_bit;

  // Replace "i2s_serializer_enable(&_i2s_instance, _i2sserializer);" with:
  uint32_t seren_bit = I2S_CTRLA_SEREN0 << _i2sserializer;
  while (I2S->SYNCBUSY.reg & seren_bit)
    ; // Sync wait
  I2S->CTRLA.reg |= seren_bit;
#endif
  return true;
}

DFRobot_DMA_IIS DMAIIS;
#endif