/**
* Pozyx_core.cpp
* --------------
* This file contains the defintion of the core POZYX functions and variables
*
*/

#include "Pozyx.h"
#include "helpers.hh"
#include <bitset>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cstring>
#include <string>
#include <sstream>
#ifdef MINNOW
  #include <linux/i2c-dev.h>
#endif
#ifdef RPI
  #include <pigpiod_if2.h>
#endif
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <ctime>

using std::memcpy;

extern "C" {
  #include "Pozyx_definitions.h"
}

#ifdef MINNOW
  #define GPIO 338
#endif
#ifdef RPI
  #define GPIO 17
  #define SDA 2
  #define SCL 3
#endif
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT 6000 // 6 seconds

//#define DEBUG
#define PRINTERRORS

int PozyxClass::_interrupt = 0;
int PozyxClass::_mode;

int PozyxClass::_hw_version;       // pozyx harware version 
int PozyxClass::_sw_version;       // pozyx software (firmware) version. (By updating the firmware on the pozyx device, this value can change);

int PozyxClass::i2c_file;
int PozyxClass::gpio_file;

void printTime(bool cerr = false) {
  time_t rawtime;
  struct tm * timeinfo;

  time (&rawtime);
  timeinfo = localtime (&rawtime);
  if (cerr) {
    std::cerr << std::dec << "[" << mktime(timeinfo) << "] ";
  } else {
    std::cout << std::dec << "[" << mktime(timeinfo) << "] ";
  }
}

void PozyxClass::initI2C(int adapter) {

  #ifdef MINNOW
  std::stringstream ss;
  ss << "/dev/i2c-" << adapter;

  //int adapter_nr = 7; // minnowboard
  i2c_file = open(ss.str().c_str(), O_RDWR);
  if (i2c_file < 0) {
    #ifdef PRINTERRORS
    printTime(true);
    std::cerr << "Failed to open I2C device" << std::endl;
    #endif
  }

  if (ioctl(i2c_file, I2C_SLAVE, POZYX_I2C_ADDRESS) < 0) {
    #ifdef PRINTERRORS
    printTime(true);
    std::cerr << "Failed to open I2C slave" << std::endl;
    #endif
  }
  #endif

  #ifdef RPI
  // bit bang i2c
  int i2c_file = pigpio_start(NULL, NULL);
  if (i2c_file < 0) {
    #ifdef PRINTERRORS
    std::cerr << "Failed to connect to pigpiod!" << std::endl;
    #endif
  }
  int ok = bb_i2c_open(i2c_file, SDA, SCL, 400000); // default pins for i2c
  if (ok != 0) {
    #ifdef PRINTERRORS
    std::cerr << "Error opening i2c: " << ok << std::endl;
    #endif
  }
  #endif
}

/**
 * The interrupt handler for the pozyx device: keeping it uber short!
 */
void PozyxClass::IRQ()
{  
  _interrupt = 1;  
}

bool PozyxClass::waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  long timer = millis();
  int status;
  
  struct pollfd fdset[1];
  int nfds = 1;
  int timeout = POLL_TIMEOUT;

  fdset[0].fd = gpio_file;
  fdset[0].events = POLLPRI;

  int rc = 0;

  char buf[1];

  // stay in this loop until the event interrupt flag is set or until the the timer runs out
  while(millis()-timer < timeout_ms)
  {
    // in polling mode, we insert a small delay such that we don't swamp the i2c bus
    if( _mode == MODE_POLLING ){
      delay(3);
    } else {
      lseek(gpio_file, 0, SEEK_SET);
      read(gpio_file, buf, sizeof buf);
    }
    //if( (_interrupt == 1) || (_mode == MODE_POLLING))
    if( (_mode == MODE_POLLING) || (rc = poll(fdset, nfds, timeout_ms)))
    {
      if (_mode == MODE_INTERRUPT) {
        lseek(gpio_file, 0, SEEK_SET);
        read(gpio_file, buf, sizeof buf);
      }
      #ifdef DEBUG
      printTime();
      std::cout << "Polled (" << (_mode == MODE_POLLING) << ")" << std::dec << millis() << ", rc: " << rc << ", revents: " << fdset[0].revents << std::endl;
      #endif
      //_interrupt = 0;
      
      // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
      uint8_t interrupt_status = 0;
      status = regRead(POZYX_INT_STATUS, &interrupt_status, 1);
      if((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
      {
        // one of the interrupts we were waiting for arrived!
        if(interrupt != NULL)
          *interrupt = interrupt_status;
        return true;
      } else {
	// wrong interrupt
        #ifdef PRINTERRORS
          //std::cerr << "Interrupt status: " << std::hex << (int)interrupt_status << ", " << (int)interrupt_flag << ", " << (int)status << std::dec << std::endl;
        #endif
      }
    } else {
      #ifdef PRINTERRORS
      int i_status;
      uint8_t istatus = 0;
      i_status = regRead(POZYX_INT_STATUS, &istatus, 1);
      printTime(true);
      std::cerr << "Error polling (" << (_mode == MODE_POLLING) << "): " << i_status << ", 0x" << std::hex << (int)istatus << ", rc: " << std::dec << rc << ", " << std::dec << millis() << std::endl;
      #endif
    }
  } 
  // too bad, pozyx didn't respond 
  // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
  // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
  return false;  
}

bool PozyxClass::waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
  int tmp = _mode;
  _mode = MODE_POLLING;
  bool result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
  _mode = tmp;
  return result;
}

int PozyxClass::begin(int adapter, bool print_result, int mode, int interrupts, int interrupt_pin){
  
  int status = POZYX_SUCCESS;

  if(print_result){
    std::cout << "Pozyx Shield" << std::endl;
    std::cout << "------------" << std::endl;
  }

  initI2C(adapter);

  // check if the mode parameter is valid
  if((mode != MODE_POLLING) && (mode != MODE_INTERRUPT)) 
    return POZYX_FAILURE;
  
  // check if the pin is valid
  if((interrupt_pin != 0) && (interrupt_pin != 1)) 
    return POZYX_FAILURE;


  //Wire.begin();
  
  // wait a bit until the pozyx board is up and running
  delay(250);
  
  _mode = mode;
  
  uint8_t whoami, selftest;  
  uint8_t regs[3];
  //regs[2] = 0x12;
  
  // we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
  if(regRead(POZYX_WHO_AM_I, regs, 3) == POZYX_FAILURE){
    return POZYX_FAILURE;
  }  
  whoami = regs[0];
  _sw_version = regs[1];
  _hw_version = regs[2]; 

  if(print_result){
    std::cout << "WhoAmI: 0x" << std::hex << (int)whoami << std::endl;
    std::cout << "SW ver.: " << std::dec << (int)_sw_version << std::endl;
    std::cout << "HW ver.: " << (int)_hw_version << std::endl;
  }
  // verify if the whoami is correct
  if(whoami != 0x43) {    
    // possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
    status = POZYX_FAILURE;
  }
  
  // readout the selftest registers to validate the proper functioning of pozyx
  if(regRead(POZYX_ST_RESULT, &selftest, 1) == POZYX_FAILURE){
    return POZYX_FAILURE;
  } 

  if(print_result){
    std::bitset<8> st(selftest);
    std::cout << "selftest: 0b" << st << std::endl;
  }

  /*std::bitset<8> typec((int)_hw_version & POZYX_TYPE);
  std::bitset<8> hwv((int)_hw_version);
  std::cerr << "hw: 0b" << hwv << std::endl;
  std::cerr << "typecheck: 0b" << typec << std::endl;
  std::cerr << "Tag: " << std::dec << POZYX_TAG << std::endl;
  std::cerr << "Anchor: " << POZYX_ANCHOR << std::endl;*/

  if(((int)_hw_version & POZYX_TYPE) == POZYX_TAG)
  {
    // check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
    //if(selftest != 0b00111111) {
    if(selftest != 0x3f) {
      status = POZYX_FAILURE;
    }
  }else if(((int)_hw_version & POZYX_TYPE) == POZYX_ANCHOR)
  {
    // check if the uwb transceiver and pressure sensor are working
    //if(selftest != 0b0011000) {
    if(selftest != 0x18) {
      status = POZYX_FAILURE;
    }
    #ifdef DEBUG
    printTime();
    std::cout << "Return from type check" << std::endl;
    #endif
    return status;
  }
  
  // set everything ready for interrupts
  _interrupt = 0;
  if(_mode == MODE_INTERRUPT){

    /*
    if (gpio_request(GPIO+interrupt_pin, "Pozyx interrupt")) {
      #ifdef DEBUG
      std::cerr << "Error interruting" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    int irq = 0;
    if ((irq = gpio_to_irq(GPIO+interrupt_pin)) < 0) {
      #ifdef DEBUG
      std::cerr << "Error interruting" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    int result = request_irq(irq, IRQ, IRQF_TRIGGER_RISING, "Pozyx interrupt", "Pozyx");

    if (result) {
      #ifdef DEBUG
      std::cerr << "Error interrupting" << std::endl;
      #endif
      return POZYX_FAILURE;
    }
    */

    // export gpio
    int fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
      #ifdef PRINTERRORS
      printTime(true);
      std::cerr << "Error interrupting 1" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    char buf[5];
    sprintf(buf, "%d", GPIO+interrupt_pin);
    write(fd, buf, 5);
    close(fd);

    std::stringstream ss;
    ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO+interrupt_pin) << "/direction";
    std::string dir = ss.str();

    fd = open(dir.c_str(), O_WRONLY);
    if (fd < 0) {
      #ifdef PRINTERRORS
      printTime(true);
      std::cerr << "Error interrupting 2" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    write(fd, "in", 3);
    close(fd);

    ss.str("");
    ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO+interrupt_pin) << "/edge";
    std::string edge = ss.str();

    fd = open(edge.c_str(), O_WRONLY);
    if (fd < 0) {
      #ifdef PRINTERRORS
      printTime(true);
      std::cerr << "Error interrupting 3" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    write(fd, "rising", 7);
    close(fd);

    ss.str("");
    ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO+interrupt_pin) << "/value";
    std::string value = ss.str();

    //gpio_file = open(value.c_str(), O_RDONLY);
    gpio_file = open(value.c_str(), O_RDONLY | O_NONBLOCK);
    if (gpio_file < 0) {
      #ifdef PRINTERRORS
      printTime(true);
      std::cerr << "Error interrupting 4" << std::endl;
      #endif
      return POZYX_FAILURE;
    }

    // set the function that must be called upon an interrupt
    //attachInterrupt(interrupt_pin, IRQ, RISING);

    // use interrupt as provided and initiate the interrupt mask
    uint8_t int_mask = interrupts;
    if (interrupt_pin == 1){
      int_mask |= POZYX_INT_MASK_PIN;
    }          
    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE){
      return POZYX_FAILURE;
    }
  }  
  
  // all done
  delay(POZYX_DELAY_LOCAL_WRITE);
  return status;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int PozyxClass::regRead(uint8_t reg_address, uint8_t *pData, int size)
{  
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c reads in smaller chunks
   
  if(!IS_REG_READABLE(reg_address))
    return POZYX_FAILURE;
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
  uint8_t reg;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    reg = reg_address+offset;    
    
    if(i+1 != n_runs){      
      status &= i2cWriteRead(&reg, 1, pData+offset, BUFFER_LENGTH);
    }else{      
      status &= i2cWriteRead(&reg, 1, pData+offset, size-offset);
    }    
  }
  
  return status;
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int PozyxClass::regWrite(uint8_t reg_address, const uint8_t *pData, int size)
{  
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c writes in smaller chunks
   
  if(!IS_REG_WRITABLE(reg_address))
    return POZYX_FAILURE;
  
  int n_runs = ceil((float)size / BUFFER_LENGTH);
  int i;
  int status = 1;
    
  for(i=0; i<n_runs; i++)
  {
    int offset = i*BUFFER_LENGTH;
    if(i+1 != n_runs){
      status &= i2cWriteWrite(reg_address+offset, pData+offset, BUFFER_LENGTH);
    }else{
      status &= i2cWriteWrite(reg_address+offset, pData+offset, size-offset);
    }    
  }
  
  return status;
}

/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int PozyxClass::regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  assert(BUFFER_LENGTH >= size+1);           // Arduino-specific code for the i2c
  assert(BUFFER_LENGTH >= param_size+1);     // Arduino-specific code for the i2c

  if(!IS_FUNCTIONCALL(reg_address))
    return POZYX_FAILURE;

  uint8_t status;
  
  // this feels a bit clumsy with all these memcpy's
  uint8_t write_data[param_size+1];
  write_data[0] = reg_address;
  memcpy(write_data+1, params, param_size);
  uint8_t read_data[size+1];
  
  // first write some data with i2c and then read some data
  status = i2cWriteRead(write_data, param_size + 1, read_data, size+1);
  if(status == POZYX_FAILURE)
    return status;    
  
  memcpy(pData, read_data+1, size);

  
  // the first byte that a function returns is always it's success indicator, so we simply pass this through
  return read_data[0];
}


/**
 * Wirelessly write a number of bytes to a specified register address on a remote Pozyx device using UWB.
 */
int PozyxClass::remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_WRITABLE(reg_address))      return POZYX_FAILURE;    // the register is not writable
  if(size > MAX_BUF_SIZE-1)              return POZYX_FAILURE;    // trying to write too much data
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;              // the first byte is the register address we want to start writing to.
  memcpy(tmp_data+2, pData, size);         // the remaining bytes are the data bytes to be written starting at the register address.
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, size+2, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x04;    // flag to indicate a register write  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  return status;
}

/**
 * Wirelessly read a number of bytes from a specified register address on a remote Pozyx device using UWB. 
 */
int PozyxClass::remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
  if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
  if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[3];
  tmp_data[0] = 0;                  // the offset in the TX buffer
  tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
  tmp_data[2] = size;               // the number of bytes to read starting from the register address 
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x02;    // flag to indicate a register read  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;
    
  // wait up to x ms to receive a response  
  if(waitForFlag_safe(POZYX_INT_STATUS_RX_DATA, POZYX_DELAY_INTERRUPT))
  {   
    // we received a response, now get some information about the response
    uint8_t rx_info[3]= {0,0,0};
    regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
    uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
    uint8_t data_len = rx_info[2];
    
    if( remote_network_id == destination && data_len == size)
    {
      status = readRXBufferData(pData, size);        
      return status;
    }else{
      return POZYX_FAILURE;  
    }     
    
  }else{
    // timeout
    return POZYX_TIMEOUT;  
  }
}

/*
 * Wirelessly call a register function with given parameters on a remote Pozyx device using UWB, the data from the function is stored in pData
 */
int PozyxClass::remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  // some checks
  if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function  
  
  int status = 0;
  
  // first prepare the packet to send
  uint8_t tmp_data[param_size+2];
  tmp_data[0] = 0;  
  tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
  memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
  status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
  
  // send the packet
  uint8_t tx_params[3];
  tx_params[0] = (uint8_t)destination;
  tx_params[1] = (uint8_t)(destination>>8);
  tx_params[2] = 0x08;    // flag to indicate a register function call  
  status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);
  
  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;
    
  // wait up to x ms to receive a response  
  if(waitForFlag_safe(POZYX_INT_STATUS_RX_DATA, POZYX_DELAY_INTERRUPT))
  {    
    // we received a response, now get some information about the response
    uint8_t rx_info[3];
    regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
    uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
    uint8_t data_len = rx_info[2];
            
    if( remote_network_id == destination && data_len == size+1)
    {
      uint8_t return_data[size+1];
  
      status = readRXBufferData(return_data, size+1);   
      
      if(status == POZYX_FAILURE){
        // debug information
        // Serial.println("could not read from rx buffer");
        return status;    
      }
  
      memcpy(pData, return_data+1, size);
        
      return return_data[0];
    }else{
      return POZYX_FAILURE;  
    }     
    
  }else{
    // timeout
    return POZYX_TIMEOUT;  
  }
}

int PozyxClass::writeTXBufferData(uint8_t data[], int size, int offset)
{
  if (offset + size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }
    
  int i, status = 1;  
  int max_bytes = BUFFER_LENGTH-2;
  int n_runs = ceil((float)size / max_bytes);
  uint8_t params[BUFFER_LENGTH];

  // read out the received data.    
  for(i=0; i<n_runs; i++)
  {   
    params[0] = offset + i*max_bytes;      // the offset
    if(i+1 != n_runs){       
      memcpy(params+1, data+i*max_bytes, max_bytes);
      status &= regFunction(POZYX_TX_DATA, params, max_bytes + 1, NULL, 0);    
    }else{
      memcpy(params+1, data+i*max_bytes, size-i*max_bytes);        
      status &= regFunction(POZYX_TX_DATA, params, size-i*max_bytes+1, NULL, 0);          
    }       
  }  

  return status;
}

int PozyxClass::readRXBufferData(uint8_t* pData, int size)
{
  if (size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }
  
  int status;
  int i;
  uint8_t params[2];
  int max_bytes = BUFFER_LENGTH-1;
  int n_runs = ceil((float)size / max_bytes);

  // read out the received data.    
  for(i=0; i<n_runs; i++)
  {        
    params[0] = i*max_bytes;      // the offset
    if(i+1 != n_runs){
      params[1] = max_bytes;      // the number of bytes to read      
    }else{
      params[1] = size - i*max_bytes;      // the number of bytes to read      
    }  
    status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);    
  }
  
  return status;
} 

int PozyxClass::sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  delay(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}


/*
 * This function sends some data bytes to the destination
 */
int PozyxClass::sendData(uint16_t destination, uint8_t *pData, int size)
{
  if(size > MAX_BUF_SIZE)          return POZYX_FAILURE;        // trying to send too much data
  
  uint8_t status = 0;

  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;                        // the first byte is the offset byte.
  memcpy(tmp_data+1, pData, size);
  
  // set the TX buffer
  status = regFunction(POZYX_TX_DATA, tmp_data, size+1, NULL, 0);  
  
  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;
 
  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    // flag to indicate we're just sending data  
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
  
  return status;   
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int PozyxClass::i2cWriteWrite(const uint8_t reg_address, const uint8_t *pData, int size)
{

  #ifdef RPI

  #ifdef DEBUG
  std::cout << "i2c write writing: " << size << std::endl;
  #endif

  // bit banged i2c

  unsigned inLen = (unsigned)size+11;

  char* inBuf = new char[64];
  inBuf[0] = 0x04; // set address
  inBuf[1] = 0x4b; // pozyx address
  inBuf[2] = 0x02; // start
  inBuf[3] = 0x07; // write
  inBuf[4] = 0x01; // how many
  inBuf[5] = (char)reg_address; // data
  inBuf[6] = 0x02; // restart
  inBuf[7] = 0x07; // write
  inBuf[8] = (char)size; // how many
  //std::copy(std::begin(pData), std::end(pData), std::next(inBuf, 9)); // data
  memcpy(inBuf+9, pData, size);
  inBuf[9+size] = 0x03; // stop
  inBuf[10+size] = 0x00; // no more commands

  unsigned outLen = 1;

  char* outBuf = new char[1];

  int bytes_read = bb_i2c_zip(i2c_file, SDA, inBuf, inLen, outBuf, outLen);

  /*if (bytes_read != read_len) {
    #ifdef DEBUG
    std::cerr << "Error reading from I2C! Got " << bytes_read << std::endl;
    #endif
    return POZYX_FAILURE;
  } else {*/
    #ifdef DEBUG
    std::cout << "Hopefully worked, returned " << bytes_read << std::endl;
    #endif
  //}

  delete[] inBuf;
  delete[] outBuf;

  #endif

  #ifdef MINNOW

  int n, error, i;

  struct i2c_rdwr_ioctl_data io;
  struct i2c_msg msg[2];
  char buf[1] = {(char)reg_address};

  msg[0].addr = POZYX_I2C_ADDRESS;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = buf;

  msg[1].addr = POZYX_I2C_ADDRESS;
  msg[1].flags = I2C_M_NOSTART;
  msg[1].len = size;
  msg[1].buf = (char*)pData;

  io.msgs = msg;
  io.nmsgs = 2;

  if (ioctl(i2c_file, I2C_RDWR, &io) < 0) {
    #ifdef PRINTERRORS
    printTime(true);
    std::cerr << "Error repeated writing to I2C" << std::endl;
    #endif
    return POZYX_FAILURE;
  }

  #endif // MINNOW

  /*
  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  // write the starting register address
  n = Wire.write(reg_address);        
  if (n != 1)
    return (POZYX_FAILURE);
    
  error = Wire.endTransmission(false); // hold the bus for a repeated start
  if (error != 0)
    return (POZYX_FAILURE);  

  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (POZYX_FAILURE);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (POZYX_FAILURE);

  */
  return (POZYX_SUCCESS);         // return : no error
}

/**
  * Call a register function using I2C with given parameters
  */
int PozyxClass::i2cWriteRead(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len)
{
  #ifdef RPI

  #ifdef DEBUG
  std::cout << "i2c write-reading: " << read_len << std::endl;
  #endif

  // bit banged i2c

  unsigned inLen = (unsigned)write_len+10;

  char* inBuf = new char[128];
  inBuf[0] = 0x04; // set address
  inBuf[1] = 0x4b; // pozyx address
  inBuf[2] = 0x02; // start
  inBuf[3] = 0x07; // write
  inBuf[4] = (char)write_len; // how many
  //std::copy(std::begin(write_data), std::end(write_data), std::next(inBuf, 5)); // data
  memcpy(inBuf+5, write_data, write_len);
  inBuf[write_len+5] = 0x02; // restart
  inBuf[write_len+6] = 0x06; // read
  inBuf[write_len+7] = (char)read_len; // how many
  inBuf[write_len+8] = 0x03; // stop
  inBuf[write_len+9] = 0x00; // no more commands

  unsigned outLen = (unsigned)read_len;

  char* outBuf = new char[128];

  int bytes_read = bb_i2c_zip(i2c_file, SDA, inBuf, inLen, outBuf, outLen);

  if (bytes_read != read_len) {
    #ifdef DEBUG
    std::cerr << "Error reading from I2C! Got " << bytes_read << std::endl;
    #endif
    return POZYX_FAILURE;
  } else {
    #ifdef DEBUG
    std::cout << "Got data, len: " << bytes_read << std::endl;
    #endif
    memcpy(read_data, outBuf, bytes_read);
  }

  delete[] inBuf;
  delete[] outBuf;

  #endif // RPI

  #ifdef MINNOW
  int i, n;

  struct i2c_rdwr_ioctl_data io;
  struct i2c_msg msg[2];

  msg[0].addr = POZYX_I2C_ADDRESS;
  msg[0].flags = 0;
  msg[0].len = write_len;
  msg[0].buf = (char*)write_data;

  msg[1].addr = POZYX_I2C_ADDRESS;
  msg[1].flags = I2C_M_NOSTART | I2C_M_RD;
  msg[1].len = read_len;
  msg[1].buf = (char*)read_data;

  io.msgs = msg;
  io.nmsgs = 2;

  if (ioctl(i2c_file, I2C_RDWR, &io) < 0) {
    #ifdef PRINTERRORS
    printTime(true);
    std::cerr << "Error repeated read from I2C" << std::endl;
    #endif
    return POZYX_FAILURE;
  } else {
    #ifdef DEBUG
    printTime();
    std::cout << "Got data, len: " << read_len << std::endl;
    #endif
  }

  #endif // MINNOW

  /*
  Wire.beginTransmission(POZYX_I2C_ADDRESS);
  for(i=0; i<write_len; i++){
    n = Wire.write(*(write_data+i));  // write parameter bytes
  }

  if (n != 1)
    return (POZYX_FAILURE);
  
  n = Wire.endTransmission(false);    // hold the I2C-bus for a repeated start

  if (n != 0)
    return (POZYX_FAILURE);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(POZYX_I2C_ADDRESS, read_len, true);
  i = 0;
  
  while(Wire.available())
  {
    if(i<read_len)
      read_data[i++]=Wire.read();
    else  
      Wire.read();
  }

  if ( i != read_len){
    return (POZYX_FAILURE);
  }

  */
  return (POZYX_SUCCESS);  // return : no error
}
