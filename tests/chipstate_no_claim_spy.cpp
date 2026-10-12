// SPDX-License-Identifier: GPL-2.0-only
#include <libusb.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
int mutations = 0, closes = 0, exits = 0, reads = 0, writes = 0;
bool opened = false;
libusb_endpoint_descriptor endpoints[4]{};
libusb_interface_descriptor alt{};
libusb_interface iface{};
libusb_config_descriptor config{};
__attribute__((destructor)) void finish() {
  if (!opened) return;
  std::fprintf(stderr, "SPY mutations=%d closes=%d exits=%d reads=%d writes=%d\n",
               mutations, closes, exits, reads, writes);
  if (mutations || closes != 1 || exits != 1) std::_Exit(70);
}
}

extern "C" {
int LIBUSB_CALL libusb_init(libusb_context **ctx) {
  *ctx = reinterpret_cast<libusb_context *>(2); return 0;
}
libusb_device_handle *LIBUSB_CALL libusb_open_device_with_vid_pid(
    libusb_context *, uint16_t, uint16_t) {
  opened = true; return reinterpret_cast<libusb_device_handle *>(1);
}
libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
  return reinterpret_cast<libusb_device *>(3);
}
int LIBUSB_CALL libusb_get_device_descriptor(libusb_device *, libusb_device_descriptor *d) {
  *d = {}; d->idVendor = 0x0bda; d->idProduct = 0xa81a; return 0;
}
int LIBUSB_CALL libusb_get_device_speed(libusb_device *) { return LIBUSB_SPEED_HIGH; }
int LIBUSB_CALL libusb_get_active_config_descriptor(libusb_device *, libusb_config_descriptor **out) {
  const unsigned char addresses[]{0x84,0x05,0x06,0x08};
  for (int i=0;i<4;++i) {
    endpoints[i].bEndpointAddress=addresses[i]; endpoints[i].bmAttributes=2;
    endpoints[i].wMaxPacketSize=512;
  }
  alt.bInterfaceClass=255; alt.bNumEndpoints=4; alt.endpoint=endpoints;
  iface.altsetting=&alt; iface.num_altsetting=1;
  config.bNumInterfaces=1; config.interface=&iface; *out=&config; return 0;
}
void LIBUSB_CALL libusb_free_config_descriptor(libusb_config_descriptor *) {}
int LIBUSB_CALL libusb_clear_halt(libusb_device_handle *, unsigned char) { ++mutations; return -99; }
int LIBUSB_CALL libusb_release_interface(libusb_device_handle *, int) { ++mutations; return 0; }
int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *, int) { ++mutations; return 0; }
int LIBUSB_CALL libusb_reset_device(libusb_device_handle *) { ++mutations; return 0; }
void LIBUSB_CALL libusb_close(libusb_device_handle *) { ++closes; }
void LIBUSB_CALL libusb_exit(libusb_context *) { ++exits; }
int LIBUSB_CALL libusb_control_transfer(libusb_device_handle *, uint8_t type,
    uint8_t request, uint16_t address, uint16_t index, unsigned char *data,
    uint16_t size, unsigned int timeout) {
  if ((type!=0xc0 && type!=0x40) || request!=5 || index!=0 ||
      (size!=1 && size!=2 && size!=4) || timeout!=500) std::_Exit(71);
  if (type==0xc0) {
    ++reads;
    const uint32_t value=address==0x280 ? 0x04002003 :
                         address==0x284 ? 0x00020000 : address==0x288 ? 0 : 0x11223344;
    for (unsigned i=0;i<size;++i) data[i]=static_cast<unsigned char>(value>>(8*i));
  } else {
    ++writes; std::printf("SPY_WRITE bytes=");
    for (unsigned i=0;i<size;++i) std::printf("%02x",data[i]);
    std::printf("\n");
  }
  if (std::getenv("BXI_SPY_ERROR")) return -7;
  if (std::getenv("BXI_SPY_SHORT")) return size-1;
  return size;
}
}
