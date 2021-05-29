//#include <bitset>
#include <iostream>
//#include <stdlib.h>
//#include <iomanip>
//#include <cstring>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
//#include <sys/ioctl.h>
#include <poll.h>
//#include <cstdio>
//#include <cinttypes>

using namespace std;

#define GPIO 340
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT -1

#define DEBUG

int main() {

int gpio_file;

// export gpio
int fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);

if (fd < 0) {
  #ifdef DEBUG
  std::cerr << "Error interrupting 1" << std::endl;
  #endif
  return 1;
}

char buf[5];
sprintf(buf, "%d", GPIO);
write(fd, buf, 5);
close(fd);

std::stringstream ss;
ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO) << "/direction";
std::string dir = ss.str();

fd = open(dir.c_str(), O_WRONLY);
if (fd < 0) {
  #ifdef DEBUG
  std::cerr << "Error interrupting 2" << std::endl;
  #endif
  return 2;
}

write(fd, "in", 3);
close(fd);

ss.str("");
ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO) << "/edge";
std::string edge = ss.str();

fd = open(edge.c_str(), O_WRONLY);
if (fd < 0) {
  #ifdef DEBUG
  std::cerr << "Error interrupting 3" << std::endl;
  #endif
  return 3;
}

write(fd, "falling", 7);
close(fd);

ss.str("");
ss << SYSFS_GPIO_DIR << "/gpio" << (GPIO) << "/value";
std::string value = ss.str();

//gpio_file = open(value.c_str(), O_RDONLY);
gpio_file = open(value.c_str(), O_RDONLY | O_NONBLOCK);
if (gpio_file < 0) {
  #ifdef DEBUG
  std::cerr << "Error interrupting 4" << std::endl;
  #endif
  return 4;
}

struct pollfd fdset[1];
int nfds = 1;
int timeout = POLL_TIMEOUT;

fdset[0].fd = gpio_file;
fdset[0].events = POLLPRI;

int rc = 0;

char buf2[1];

lseek(gpio_file, 0, SEEK_SET);
read(gpio_file, buf2, sizeof buf);

if( rc = poll(fdset, nfds, timeout) ) {
  lseek(gpio_file, 0, SEEK_SET);
  read(gpio_file, buf2, sizeof buf);
  #ifdef DEBUG
  std::cout << "Polled, rc: " << rc << ", revents: " << fdset[0].revents << std::endl;
  #endif

  // shutdown
  system("shutdown -h now");

} else {
  #ifdef DEBUG
  std::cerr << "Error polling: " << rc << ", " << std::endl;
  #endif
}

return 0;
}
