#ifndef __UART_HPP__
#define __UART_HPP__


#include "stdint.h"

#include "hardware/pio.h"


namespace myt {

class UART1 {
public:
  bool isValid; // Flag for whether or not the last message was valid

  // Debugging counters for various messaging errors
  uint32_t numFrameErrors;
  uint32_t numParityErrors;
  uint32_t numOverRunErrors;

  UART1(uint32_t baud, uint8_t txPin, uint8_t rxPin);
  ~UART1();

  void tx(uint16_t data);
  // Receive frame without first waiting for receive buffer to fill. Use
  // UART1::waitForRx() or manually poll the buffer before calling UART1::rx().
  uint16_t rx();

  // Wait `milliseconds` for the receive buffer to fill. Return true once it
  // fills and false if it does not before reaching timeout.
  bool waitForRx(uint8_t milliseconds);

  bool rxBufferEmpty();
  bool txBufferEmpty();

  void listenDataFrames();
  void ignoreDataFrames();

  void enableRxInterrupt();
  void disableRxInterrupt();
  bool isRxInterruptOn();

  void registerRxHandler(irq_handler_t handler);

  bool rxIrqHandler();
  
private:
  // Configure a timer for UART1::waitForRx
  void setupTimeout();
  void receive();

  PIO pio;
  uint8_t sm_tx;
  uint8_t sm_rx;

  bool _listenDataFrames = false;
  volatile uint16_t _data;
  volatile bool _dataAvailable = false;
  bool interruptsEnabled = false;
};

}

#endif /* __UART_HPP__ */
