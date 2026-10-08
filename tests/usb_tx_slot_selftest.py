#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile the real tx_async admission block with a controlled event pump.

Usage: python3 tests/usb_tx_slot_selftest.py <temporary-output-directory>
This checks queue admission, not USB submission, completion or teardown.
"""
from pathlib import Path
import subprocess
import sys
import tempfile

source = (Path(__file__).resolve().parents[1] / 'src/UsbTransport.cpp').read_text()
body = source.split('bool UsbTransport::tx_async(', 1)[1].split('{', 1)[1]
body = body.split('  libusb_transfer *transfer', 1)[0]
code = '''#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <sys/time.h>
int libusb_handle_events_timeout_completed(void*, timeval*, int*);
struct Probe {
  std::atomic<int> _tx_inflight{0};
  std::array<std::atomic<int>, 16> _tx_ep_inflight{};
  std::atomic<bool> _tx_shutdown{false};
  void* _ctx=this;
  int mode=0;
  std::atomic<int> calls{0};
  std::chrono::steady_clock::time_point now() const {
    if (mode==6) return std::chrono::steady_clock::now();
    return std::chrono::steady_clock::time_point{} +
           std::chrono::microseconds(1000 * (calls > 1 ? calls - 1 : 0));
  }
  void flush_writes() {}
  bool admit(uint8_t tx_ep=9) {
''' + body.replace('std::chrono::steady_clock::now()', 'now()') + '''    return true;
  }
};
int libusb_handle_events_timeout_completed(void* context, timeval* wait, int*) {
  auto& p=*static_cast<Probe*>(context);
  if (++p.calls > 8 && p.mode!=6) throw std::runtime_error("unbounded full-queue wait");
  if (wait->tv_usec) {
    assert(wait->tv_sec==0 && wait->tv_usec>0 && wait->tv_usec<=2000);
    if (p.mode==0) p._tx_ep_inflight[9]=15;
    if (p.mode==2) return -1;
    if (p.mode==3) { p._tx_shutdown=true; p._tx_ep_inflight[9]=0; }
    if (p.mode==4 && p.calls==3) p._tx_ep_inflight[9]=15;
    if (p.mode==5) p._tx_ep_inflight[12]=0;
    if (p.mode==6) return -1;
  }
  return 0;
}
int main() {
  try {
    for (int mode=0;mode<6;++mode) {
      Probe p; p.mode=mode; p._tx_inflight=16; p._tx_ep_inflight[9]=16;
      assert(p.admit()==(mode==0 || mode==4) && p.calls<=3);
    }
    Probe healthy;
    assert(healthy.admit() && healthy.calls==1);
    Probe stopped; stopped._tx_shutdown=true;
    assert(!stopped.admit() && stopped.calls==0);
    Probe bulk_full; bulk_full.mode=1; bulk_full._tx_inflight=256;
    bulk_full._tx_ep_inflight[9]=16;
    assert(bulk_full.admit(12) && bulk_full._tx_ep_inflight[12]==1);
    Probe concurrent; concurrent.mode=6;
    std::array<std::thread,32> workers;
    std::atomic<int> accepted{0};
    for (auto& worker:workers) worker=std::thread([&]{if(concurrent.admit())++accepted;});
    for (auto& worker:workers) worker.join();
    assert(accepted==16 && concurrent._tx_ep_inflight[9]==16);
  } catch(const std::exception& e) { std::fprintf(stderr,"%s\\n",e.what()); return 1; }
}
'''
with tempfile.TemporaryDirectory(prefix='usb-tx-slot-', dir=sys.argv[1]) as temporary:
    root = Path(temporary)
    (root / 'check.cpp').write_text(code)
    subprocess.run(['c++', '-std=c++20', '-O2', str(root / 'check.cpp'), '-o', str(root / 'check')], check=True)
    subprocess.run([str(root / 'check')], check=True)
print('PASS: bounded TX admission, endpoint isolation, and concurrent reservations')
