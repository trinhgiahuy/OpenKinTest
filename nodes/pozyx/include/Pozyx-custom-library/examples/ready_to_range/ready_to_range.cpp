/**
  The pozyx ready to range demo (c) Pozyx Labs
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started

  This demo requires two pozyx devices and one Arduino. It demonstrates the ranging capabilities and the functionality to
  to remotely control a pozyx device. Place one of the pozyx shields on the Arduino and upload this sketch. Move around
  with the other pozyx device.

  This demoe measures the range between the two devices. The closer the devices are to each other, the more LEDs will
  burn on both devices.
*/

#include "helpers.hh"
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <iostream>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t destination_id = 0x601C;     // the network id of the other pozyx device: fill in the network id of the other device
signed int range_step_mm = 100;      // every 1000mm in range, one LED less will be giving light.

////////////////////////////////////////////////

void loop();
int ledControl(uint32_t range);

int main(){

  if(Pozyx.begin(true, MODE_POLLING) == POZYX_FAILURE){
    std::cerr << "ERROR: Unable to connect to POZYX shield" << std::endl;
    std::cerr << "Reset required" << std::endl;
    delay(100);
    return 1;
  }

  std::cout << "------------POZYX RANGING V1.0------------" << std::endl;
  std::cout << "NOTES:" << std::endl;
  std::cout << "- Change the parameters:\n\tdestination_id (target device)\n\trange_step (mm)\n\t" << std::endl;
  std::cout << std::endl;
  std::cout << "- Approach target device to see range and\n led control" << std::endl;
  std::cout << "------------POZYX RANGING V1.0------------" << std::endl;
  std::cout << std::endl;
  std::cout << "START Ranging:" << std::endl;

  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t configuration_leds = 0x0;
  Pozyx.regWrite(POZYX_CONFIG_LEDS, &configuration_leds, 1);

  // do the same with the remote device
  Pozyx.remoteRegWrite(destination_id, POZYX_CONFIG_LEDS, &configuration_leds, 1);

  while (true) {
    loop();
  }
}

void loop(){

  int status = 1;
  device_range_t range;

  // let's do ranging with the destination
  status &= Pozyx.doRanging(destination_id, &range);

  if (status == POZYX_SUCCESS){
    std::cout << std::dec << (int)range.timestamp << "ms \t" << (int)range.distance << "mm \t" << std::endl;

    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
    if (ledControl(range.distance) == POZYX_FAILURE){
      std::cerr << "ERROR: setting (remote) leds" << std::endl;
    }
  }
  else{
    std::cerr << "ERROR: ranging" << std::endl;
  }
}

int ledControl(uint32_t range){
  int status = 1;

  // set the LEDs of this pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm));
  status &= Pozyx.setLed(3, (range < 2*range_step_mm));
  status &= Pozyx.setLed(2, (range < 3*range_step_mm));
  status &= Pozyx.setLed(1, (range < 4*range_step_mm));

  // set the LEDs of the remote pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), destination_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), destination_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), destination_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), destination_id);

  // status will be zero if setting the LEDs failed somewhere along the way
  return status;
}
