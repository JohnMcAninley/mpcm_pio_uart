#include <functional>

#include <stdio.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "drivers/uart.hpp"
#include "uart_tx.pio.h"
#include "uart_rx.pio.h"


void rxInterruptHandler(PIO pio, uint8_t sm_rx, irq_handler_t handler)
{
  uint32_t word = pio_sm_get(pio, sm_rx);
  uint16_t frame = word >> 16;
  
  uint8_t parity = (frame >> 15) & 1;
  uint8_t data = (frame >> 7) & 0xFF;

  uint8_t parityBit = 1;
  for (int i = 0; i < 8; i++)
  {
      parityBit ^= ((data >> i) & 0x01);
  }
  
  if (parity == parityBit)// && this->userRxInterruptHandler)
  {
    //this->userRxInterruptHandler();
    handler();
  }
}

// Note that the UART receiver defaults to ignoring data frames.
UART1::UART1(uint32_t baud, uint8_t txPin, uint8_t rxPin) :
  isValid(false), numFrameErrors(0), numOverRunErrors(0), numParityErrors(0)
{
  this->ignoreDataFrames();

  this->pio = pio0;
  this->sm_rx = 0;
  uint offset = pio_add_program(this->pio, &uart_rx_program);
  uart_rx_program_init(this->pio, this->sm_rx, offset, rxPin, baud);

  this->sm_tx = 1;
  offset = pio_add_program(this->pio, &uart_tx_program);
  uart_tx_program_init(this->pio, this->sm_tx, offset, txPin, baud);

  //std::bind(rxInterruptHandler, this->pio, this->sm_rx, &on_uart_rx);
  //irq_set_exclusive_handler(PIO0_IRQ_0, this->rxInterruptHandler);
}

// Disable USART
UART1::~UART1()
{
  disableRxInterrupt();

  // Cleanup pio
  pio_sm_set_enabled(this->pio, this->sm_tx, false);
  //pio_remove_program(this->pio, &uart_tx_program, offset);
  pio_sm_unclaim(this->pio, this->sm_tx);

  pio_sm_set_enabled(this->pio, this->sm_rx, false);
  //pio_remove_program(this->pio, &uart_rx_program, offset);
  pio_sm_unclaim(this->pio, this->sm_tx);
}


bool UART1::rxBufferEmpty()
{
  if (this->_dataAvailable) return false;
  // TODO parity not checked yet, data not necessarily valid 
  /*
  while (!pio_sm_is_rx_fifo_empty(this->pio, this->sm_rx))
  {
    this->_receive();
  }*/
  return pio_sm_is_rx_fifo_empty(this->pio, this->sm_rx);
}

bool UART1::txBufferEmpty()
{
  return pio_sm_is_tx_fifo_empty(this->pio, this->sm_tx);
}

void UART1::listenDataFrames()
{
  this->_listenDataFrames = true;
}

void UART1::ignoreDataFrames()
{
  this->_listenDataFrames = false;
}

void UART1::enableRxInterrupt()
{
  irq_set_enabled(this->pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, true);
  this->interruptsEnabled = true;
}

void UART1::disableRxInterrupt()
{
  irq_set_enabled(this->pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, false);
  this->interruptsEnabled = false;
}

bool UART1::isRxInterruptOn()
{ 
  //return irq_is_enabled(this->pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0);
  return this->interruptsEnabled;
}

void UART1::tx(uint16_t data)
{
  while (!pio_sm_is_tx_fifo_empty(this->pio, this->sm_tx));

  uint8_t parityBit = 1;
  for (int i = 0; i < 9; i++)
  {
    parityBit ^= ((data >> i) & 0x01);
  }
  data |= (parityBit << 9);
  uint32_t word = data;

  pio_sm_put(this->pio, this->sm_tx, word);

  while (!pio_sm_is_tx_fifo_empty(this->pio, this->sm_tx));
}

uint16_t UART1::rx() 
{
  // TODO check any words in fifo
  if (this->_dataAvailable)
  {
      this->_dataAvailable = false;
      return this->_data;
  }
  else
  {
    this->receive();
    this->_dataAvailable = false;
    return this->_data;
  }
}

bool UART1::waitForRx(uint8_t timeout_ms) {

  uint32_t startTime = to_ms_since_boot(get_absolute_time());

  if (this->_dataAvailable) return true;

  while(this->rxBufferEmpty() && to_ms_since_boot(get_absolute_time()) - startTime < timeout_ms);

  // Did we get here because of the buffer filling or because of the timer timing out?
  return !this->rxBufferEmpty();
}

void UART1::registerRxHandler(irq_handler_t handler)
{
  // TODO
}

bool UART1::rxIrqHandler()
{
  // TODO
  // Check for Frame Error, OverRun Error, and Parity Error (pg. 198)
  // Note: this must be done before reading the data buffer.
  const auto isError = false;

  // TODO parity not checked
  uint32_t word = pio_sm_get(this->pio, this->sm_rx);
  uint16_t frame = word >> 16;

  //printf("%x\n", frame);

  uint8_t parity = (frame >> 15) & 1;
  uint8_t isAddress = (frame >> 14) & 1;
  uint8_t data = (frame >> 6) & 0xFF;
  frame >>= 6;
  frame &= 0x1FF;

  uint8_t parityBit = isAddress ^ 1;
  for (int i = 0; i < 9; i++)
  {
      parityBit ^= ((data >> i) & 0x01);
  }

  //printf("ISR: %x: %x, %x, %x", frame, isAddress, data, parity);

  if(isError > 0) {
    this->isValid = false;
    // Increment error counters for diagnostics/debugging
    numFrameErrors   += 0;
    numParityErrors  += 0;
    numOverRunErrors += 0;
  } else {
    this->isValid = true;
  }

  //if (parityBit != parity) printf("...parity error\n");
  //else if (!isAddress && !this->_listenDataFrames) printf("...data ignored\n");

  if (parityBit == parity && (isAddress || this->_listenDataFrames))
  {
    //printf("...received\n");
    this->_dataAvailable = true;
    this->_data = frame;

    if (this->interruptsEnabled) return true;
  }

  return false;
}

void UART1::receive()
{
  uint32_t w = pio_sm_get(this->pio, this->sm_rx);
  uint16_t frame = w >> 16;
  
  uint8_t parity = (frame >> 15) & 1;
  uint8_t isAddress = (frame >> 14) & 1;
  uint8_t data = (frame >> 6) & 0xFF;
  frame >>= 6;
  frame &= 0x1FF;

  //printf("%x, %x\n", (uint8_t)data, parity);

  uint8_t parityBit = isAddress ^ 1;
  for (int i = 0; i < 9; i++)
  {
      parityBit ^= ((data >> i) & 0x01);
  }
  
  //printf("%x: %x, %x, %x", frame, isAddress, data, parity);

  if (parityBit != parity)
  {
      //printf(", frame error");
  }
  //printf("\n");

  if (parityBit == parity && (isAddress || _listenDataFrames))
  {
      _dataAvailable = true;
      _data = frame;
  }
}
