// SPDX-License-Identifier: GPL-2.0-only
#include "RxTransferLength.h"
#include "jaguar3/FrameParserJaguar3.h"
#include "kestrel/FrameParserKestrel.h"
#include <array>
#include <cstdio>
#include <initializer_list>

int main() {
  int failures = 0;
  const auto check = [&](bool ok, const char* name) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", name); ++failures; }
  };
  for (const auto status : {LIBUSB_TRANSFER_COMPLETED, LIBUSB_TRANSFER_ERROR,
                            LIBUSB_TRANSFER_TIMED_OUT}) {
    check(devourer::rx_transfer_length(status, 100, 176) == 100, "received prefix");
    check(devourer::rx_transfer_length(status, 0, 176) == 0, "empty completion");
    check(devourer::rx_transfer_length(status, -1, 176) == 0, "negative length");
    check(devourer::rx_transfer_length(status, 177, 176) == 0, "over capacity");
  }
  for (const auto status : {LIBUSB_TRANSFER_STALL, LIBUSB_TRANSFER_NO_DEVICE,
                            LIBUSB_TRANSFER_OVERFLOW, LIBUSB_TRANSFER_CANCELLED})
    check(devourer::rx_transfer_length(status, 100, 176) == 0, "unsafe status");
  check(devourer::rx_transfer_length(static_cast<libusb_transfer_status>(7), 100, 176) == 0,
        "unknown status");
  std::array<unsigned char, 176> buffer{};
  buffer[0] = buffer[88] = 64;
  buffer[24] = buffer[112] = 0x88;
  const auto frames = [&](int length) {
    int count = 0;
    unsigned offset = 0;
    while (offset + jaguar3::RXDESC_SIZE_8822C <= static_cast<unsigned>(length)) {
      jaguar3::Rx8822cFrame frame{};
      if (!jaguar3::parse_rx_8822c(buffer.data() + offset, length - offset, frame)) break;
      ++count;
      offset += frame.next_offset;
    }
    return count;
  };
  check(frames(devourer::rx_transfer_length(LIBUSB_TRANSFER_ERROR, 100, 176)) == 1,
        "complete frame survives error; truncated tail discarded");
  check(frames(devourer::rx_transfer_length(LIBUSB_TRANSFER_COMPLETED, 176, 176)) == 2,
        "next complete buffer is parsed normally");
  std::array<unsigned char, 160> ax{};
  ax[0] = ax[80] = 64;
  const auto ax_frames = [&](int length) {
    int count = 0;
    unsigned offset = 0;
    while (offset + 16 <= static_cast<unsigned>(length)) {
      kestrel::KestrelRxFrame frame{};
      if (!kestrel::parse_rx_8852b(ax.data() + offset, length - offset, frame)) break;
      ++count;
      offset += frame.next_offset;
    }
    return count;
  };
  check(ax_frames(devourer::rx_transfer_length(LIBUSB_TRANSFER_ERROR, 92, 160)) == 1,
        "Kestrel complete prefix survives; short next descriptor rejected");
  check(ax_frames(devourer::rx_transfer_length(LIBUSB_TRANSFER_COMPLETED, 160, 160)) == 2,
        "Kestrel next complete buffer is parsed normally");
  if (!failures) std::puts("PASS: RX transfer prefix length and frame bounds");
  return failures ? 1 : 0;
}
