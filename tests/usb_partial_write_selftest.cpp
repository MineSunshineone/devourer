// SPDX-License-Identifier: GPL-2.0-only
// BXI modification 2026-10-15: cover bounded reads and reject failed channel writes.
#include "UsbTransport.h"
#ifdef DEVOURER_HAVE_JAGUAR3_8822E
#include "jaguar3/RtlJaguar3Device.h"
#endif

#include <atomic>
#include <barrier>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace {
std::atomic<int> calls{0};
int first_rc = 0;
int first_actual = 0;
int failures = 0;
libusb_transfer *pending = nullptr;
unsigned async_calls = 0;
bool parallel_probe = false;
bool override_control_read = false;
int control_read_result = 0;
int second_control_read_result = 0;
unsigned control_read_calls = 0;
bool exhaust_control_budget = false;
bool emulate_registers = false;
std::unordered_map<uint32_t, unsigned char> registers;
uint32_t failed_register = 0xffffffffu;
unsigned failed_register_writes = 0;
int failed_register_result = LIBUSB_ERROR_IO;
int probe_active = 0, probe_peak = 0, probe_entered = 0;
std::mutex probe_mutex;
std::condition_variable probe_ready;

#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
  ++failures; } } while (0)

void scenario(int rc, int actual, bool poisoned, bool concurrent = false) {
  first_rc = rc;
  first_actual = actual;
  calls = 0;
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  unsigned char bytes[1000]{};
  int a = 0, b = 0;
  if (concurrent) {
    std::barrier start(2);
    std::thread worker([&] {
      start.arrive_and_wait();
      a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    });
    start.arrive_and_wait();
    b = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    worker.join();
    CHECK(a < static_cast<int>(sizeof(bytes)) && b < static_cast<int>(sizeof(bytes)));
  } else {
    a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    b = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
    CHECK(a == (rc ? rc : actual));
    CHECK(poisoned ? b < 0 : b == static_cast<int>(sizeof(bytes)));
  }
  if (poisoned) {
    CHECK(transport.tx_sync_data(6, bytes, sizeof(bytes), 500) < 0);
    CHECK(!transport.tx_async(6, bytes, sizeof(bytes), 500));
  }
  CHECK(calls == (poisoned ? 1 : 2));
  CHECK(transport.tx_stats().needs_reinit == poisoned);
  CHECK(transport.tx_stats().submitted == static_cast<unsigned>(calls.load()));
  CHECK(transport.tx_stats().failed == static_cast<unsigned>(rc != 0 || actual < 1000));
  CHECK(transport.tx_stats().last_error_rc ==
        (rc ? rc : actual < 1000 ? devourer::kTxShortWriteRc : 0));
  CHECK(transport.tx_stats().last_was_timeout == (rc == LIBUSB_ERROR_TIMEOUT));
}

void endpoint_scenario(bool same_endpoint) {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  parallel_probe = true;
  probe_active = probe_peak = probe_entered = 0;
  unsigned char bytes[1000]{};
  std::barrier start(2);
  int a = 0, b = 0;
  std::thread worker([&] {
    start.arrive_and_wait();
    a = transport.tx_sync_data(8, bytes, sizeof(bytes), 500);
  });
  start.arrive_and_wait();
  b = transport.tx_sync_data(same_endpoint ? 8 : 6, bytes, sizeof(bytes), 500);
  worker.join();
  parallel_probe = false;
  CHECK(a == 1000 && b == 1000);
  CHECK(probe_peak == (same_endpoint ? 1 : 2));
  CHECK(!transport.tx_stats().needs_reinit);
}

void read_scenario(int first, int second, bool exhaust_budget = false) {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(reinterpret_cast<libusb_device_handle *>(1), logger);
  override_control_read = true;
  control_read_result = first;
  second_control_read_result = second;
  control_read_calls = 0;
  exhaust_control_budget = exhaust_budget;
  const unsigned expected_calls = first == LIBUSB_ERROR_IO && !exhaust_budget ? 2 : 1;
  const int result = expected_calls == 2 ? second : first;
  const auto before = transport.ctrl_xfers();
  try {
    CHECK(transport.read32(0x0cbc) == 0);
    CHECK(result == 4);
  } catch (const std::ios_base::failure& error) {
    CHECK(result != 4);
    CHECK(std::string(error.what()).find("result=" + std::to_string(result)) != std::string::npos);
  }
  CHECK(transport.ctrl_xfers() == before + expected_calls);
  CHECK(control_read_calls == expected_calls);
  override_control_read = false;
  exhaust_control_budget = false;
}

#ifdef DEVOURER_HAVE_JAGUAR3_8822E
void retune_write_failure(bool fast, uint32_t reg, int failure = LIBUSB_ERROR_IO) {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  auto transport = std::make_shared<devourer::UsbTransport>(
      reinterpret_cast<libusb_device_handle *>(1), logger);
  RtlJaguar3Device radio(RtlAdapter(transport, logger), logger,
                         jaguar3::ChipVariant::C8822E);
  emulate_registers = true;
  registers.clear();
  radio.SetMonitorChannel({.Channel = 153, .ChannelOffset = 0,
                           .ChannelWidth = CHANNEL_WIDTH_20});
  failed_register = reg;
  failed_register_result = failure;
  failed_register_writes = 0;
  bool rejected = false;
  try {
    if (fast) radio.FastRetune(165, true);
    else radio.SetMonitorChannel({.Channel = 165, .ChannelOffset = 0,
                                  .ChannelWidth = CHANNEL_WIDTH_20});
  } catch (const std::exception& error) {
    rejected = std::string(error.what()).find("radio register write failed") != std::string::npos;
  }
  CHECK(failed_register_writes != 0);
  CHECK(rejected);
  failed_register = 0xffffffffu;
  radio.SetMonitorChannel({.Channel = 157, .ChannelOffset = 0,
                           .ChannelWidth = CHANNEL_WIDTH_20});
  CHECK(registers[0x3c60] == 157 && registers[0x4c60] == 157);
  emulate_registers = false;
}
#endif

void control_write_failure_accounting() {
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(reinterpret_cast<libusb_device_handle *>(1), logger);
  for (const int failure : {static_cast<int>(LIBUSB_ERROR_IO), 0}) {
    for (unsigned operation = 0; operation < 5; ++operation) {
      const auto write = [&] {
        const unsigned char bytes[]{1, 2, 3, 4};
        switch (operation) {
          case 0: return transport.write8(0xc30, 1);
          case 1: return transport.write16(0xc30, 1);
          case 2: return transport.write32(0xc30, 1);
          case 3: return transport.write32_wide(0x10c30, 1);
          default: return transport.write_bytes(0xc30, bytes, sizeof(bytes));
        }
      };
      failed_register = operation == 3 ? 0x10c30u : 0xc30u;
      failed_register_result = failure;
      const auto before = transport.ctrl_write_failures();
      CHECK(!write());
      CHECK(transport.ctrl_write_failures() == before + 1);
      failed_register = 0xffffffffu;
      CHECK(write());
      CHECK(transport.ctrl_write_failures() == before + 1);
    }
  }
}
}  // namespace

// Exercise the real UsbTransport; replace only its libusb device I/O.
extern "C" libusb_device * LIBUSB_CALL libusb_get_device(libusb_device_handle *) {
  return reinterpret_cast<libusb_device *>(1);
}
extern "C" int LIBUSB_CALL libusb_get_device_descriptor(
    libusb_device *, libusb_device_descriptor *desc) {
  *desc = {};
  desc->idVendor = 0x0bda;
  desc->idProduct = 0xa81a;
  return 0;
}
extern "C" int LIBUSB_CALL libusb_get_device_speed(libusb_device *) {
  return LIBUSB_SPEED_HIGH;
}
extern "C" int LIBUSB_CALL libusb_clear_halt(libusb_device_handle *, unsigned char) {
  return 0;
}
extern "C" int LIBUSB_CALL libusb_handle_events_timeout_completed(
    libusb_context *, timeval *, int *) {
  return 0;
}
extern "C" int LIBUSB_CALL libusb_control_transfer(
    libusb_device_handle *, uint8_t type, uint8_t, uint16_t reg, uint16_t index,
    unsigned char *data, uint16_t length, unsigned) {
  const uint32_t address = reg | (static_cast<uint32_t>(index) << 16);
  if (type == 0x40) {
    if (address == failed_register) {
      ++failed_register_writes;
      return failed_register_result;
    }
    if (emulate_registers)
      for (unsigned i = 0; i < length; ++i) registers[address + i] = data[i];
    return length;
  }
  std::memset(data, 0, length);
  if (type == 0xc0 && override_control_read) {
    ++control_read_calls;
    if (exhaust_control_budget) std::this_thread::sleep_for(std::chrono::milliseconds(510));
    return control_read_calls == 1 ? control_read_result : second_control_read_result;
  }
  if (emulate_registers)
    for (unsigned i = 0; i < length; ++i) data[i] = registers[address + i];
  return length;
}
extern "C" int LIBUSB_CALL libusb_submit_transfer(libusb_transfer *transfer) {
  CHECK(pending == nullptr);
  pending = transfer;
  ++async_calls;
  return 0;
}
extern "C" int LIBUSB_CALL libusb_bulk_transfer(
    libusb_device_handle *, unsigned char, unsigned char *, int length,
    int *actual, unsigned timeout) {
  CHECK(timeout == 0);  // Keep the production multi-packet no-cancel policy.
  if (parallel_probe) {
    std::unique_lock lock(probe_mutex);
    probe_peak = std::max(probe_peak, ++probe_active);
    ++probe_entered;
    probe_ready.notify_all();
    probe_ready.wait_for(lock, std::chrono::milliseconds(100), [] { return probe_entered == 2; });
    --probe_active;
    *actual = length;
    return 0;
  }
  if (calls.fetch_add(1) == 0) {
    *actual = first_actual;
    return first_rc;
  }
  *actual = length;
  return 0;
}

void complete(libusb_transfer_status status, int actual) {
  auto *transfer = pending;
  pending = nullptr;
  transfer->status = status;
  transfer->actual_length = actual;
  transfer->callback(transfer);
}

void async_scenario(libusb_transfer_status status, int actual, bool poisoned) {
  async_calls = 0;
  auto logger = std::make_shared<Logger>();
  logger->set_level(Logger::Level::Silent);
  devourer::UsbTransport transport(
      reinterpret_cast<libusb_device_handle *>(1), logger, nullptr, {}, true,
      devourer::RxMode::Async, 0, 0, devourer::PoolExhaust::Backpressure, true);
  unsigned char bytes[1000]{};
  CHECK(transport.tx_async(8, bytes, sizeof(bytes), 500));
  complete(status, actual);
  CHECK(transport.tx_stats().needs_reinit == poisoned);
  const bool accepted = transport.tx_async(8, bytes, sizeof(bytes), 500);
  CHECK(accepted == !poisoned);
  if (accepted) complete(LIBUSB_TRANSFER_COMPLETED, sizeof(bytes));
  CHECK(async_calls == (poisoned ? 1U : 2U));
  CHECK(transport.tx_stats().inflight == 0);
  if (poisoned) {
    const auto before = calls.load();
    CHECK(transport.tx_sync_data(8, bytes, sizeof(bytes), 500) < 0);
    CHECK(calls == before);
  }
  if (status == LIBUSB_TRANSFER_COMPLETED && actual != 1000)
    CHECK(transport.tx_stats().last_error_rc == devourer::kTxShortWriteRc);
}

int main() {
  for (const auto result : std::array<int, 6>{LIBUSB_ERROR_IO, LIBUSB_ERROR_TIMEOUT,
                                           LIBUSB_ERROR_NO_DEVICE, 0, 2, 4})
    read_scenario(result, result);
  read_scenario(LIBUSB_ERROR_IO, 4);
  read_scenario(LIBUSB_ERROR_IO, LIBUSB_ERROR_NO_DEVICE);
  read_scenario(LIBUSB_ERROR_TIMEOUT, 4);
  read_scenario(LIBUSB_ERROR_IO, 4, true);
  control_write_failure_accounting();
#ifdef DEVOURER_HAVE_JAGUAR3_8822E
  for (const bool fast : {false, true})
    for (const auto reg : {0x3c60u, 0x4c60u, 0xc30u})
      retune_write_failure(fast, reg);
  retune_write_failure(true, 0x3c60u, 0);
#endif
  endpoint_scenario(false);
  endpoint_scenario(true);
  scenario(LIBUSB_ERROR_IO, 512, true);
  scenario(LIBUSB_ERROR_TIMEOUT, 512, true);
  scenario(0, 512, true);
  scenario(LIBUSB_ERROR_IO, 0, false);
  scenario(0, 0, false);
  scenario(LIBUSB_ERROR_IO, 1000, false);
  scenario(0, 1000, false);
  scenario(LIBUSB_ERROR_IO, 512, true, true);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 512, true);
  async_scenario(LIBUSB_TRANSFER_ERROR, 512, true);
  async_scenario(LIBUSB_TRANSFER_TIMED_OUT, 512, true);
  async_scenario(LIBUSB_TRANSFER_STALL, 512, true);
  async_scenario(LIBUSB_TRANSFER_CANCELLED, 512, false);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 0, false);
  async_scenario(LIBUSB_TRANSFER_ERROR, 0, false);
  async_scenario(LIBUSB_TRANSFER_ERROR, 1000, false);
  async_scenario(LIBUSB_TRANSFER_COMPLETED, 1000, false);
  if (!failures) std::puts("PASS: partial writes stop later submissions; complete/zero-byte errors retain retry behavior");
  return failures ? 1 : 0;
}
