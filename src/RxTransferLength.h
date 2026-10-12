// SPDX-License-Identifier: GPL-2.0-only
#pragma once

#include <libusb.h>

namespace devourer {
inline int rx_transfer_length(libusb_transfer_status status, int actual, int capacity) {
  if (actual <= 0 || actual > capacity) return 0;
  // A later USB error can leave a transferred prefix; parsers reject its cut tail.
  return status == LIBUSB_TRANSFER_COMPLETED || status == LIBUSB_TRANSFER_ERROR ||
                 status == LIBUSB_TRANSFER_TIMED_OUT
             ? actual : 0;
}
} // namespace devourer
