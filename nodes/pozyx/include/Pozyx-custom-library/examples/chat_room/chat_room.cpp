/**
  The pozyx chat demo
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started

  This demo requires at least two pozyx shields and an equal number of Arduino's.
  It demonstrates the wireless messaging capabilities of the pozyx device.

  This demo creates a chat room. Text written in the Serial monitor will be broadcasted to all other pozyx devices
  within range. They will see your message appear in their Serial monitor.
*/

#include "helpers.hh"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <string>
#include <iostream>
#include <iomanip>

using namespace std;

uint16_t source_id;                 // the network id of this device
uint16_t destination_id = 0;        // the destination network id. 0 means the message is broadcasted to every device in range
string inputString = "test message";            // a string to hold incoming data
bool stringComplete = true;     // whether the string is complete

void loop();

int main(){

  // initialize Pozyx
  if(! Pozyx.begin(false, MODE_POLLING, POZYX_INT_MASK_RX_DATA, 0)){
    cerr << "ERROR: Unable to connect to POZYX shield" << endl;
    cerr << "Reset required" << endl;
    return 1;
  }

  // read the network id of this device
  Pozyx.regRead(POZYX_NETWORK_ID, (uint8_t*)&source_id, 2);

  // reserve 100 bytes for the inputString:
  inputString.reserve(100);

  cout << "--- Pozyx Chat started ---" << endl;

  while (true) {
    loop();
  }

  return 0;

}

void loop(){

  // check if we received a newline character and if so, broadcast the inputString.
  if(stringComplete){
    cout << "Ox" << hex << source_id << ": " << inputString << endl;

    int length = 13;
    uint8_t buffer[13] = "Test message";
    //inputString.getBytes(buffer, length);

    // write the message to the transmit (TX) buffer
    int status = Pozyx.writeTXBufferData(buffer, length);
    // broadcast the contents of the TX buffer
    status = Pozyx.sendTXBufferData(destination_id);

    //inputString = "";
    //stringComplete = false;
  }

  // we wait up to 50ms to see if we have received an incoming message (if so we receive an RX_DATA interrupt)
  if(Pozyx.waitForFlag(POZYX_INT_STATUS_RX_DATA,50))
  {
    // we have received a message!

    uint8_t length = 0;
    uint16_t messenger = 0x00;
    delay(1);
    // Let's read out some information about the message (i.e., how many bytes did we receive and who sent the message)
    Pozyx.getLastDataLength(&length);
    Pozyx.getLastNetworkId(&messenger);

    char data[length];

    // read the contents of the receive (RX) buffer, this is the message that was sent to this device
    Pozyx.readRXBufferData((uint8_t *) data, length);
    cout << "Ox" << hex << messenger << ": " << data << endl;
  }

}
