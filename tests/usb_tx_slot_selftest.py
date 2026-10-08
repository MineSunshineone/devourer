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
code = '''#include <atomic>
#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <sys/time.h>
int libusb_handle_events_timeout_completed(void*, timeval*, int*);
struct Probe {
  std::atomic<int> _tx_inflight{0};
  std::atomic<bool> _tx_shutdown{false};
  void* _ctx=this;
  int mode=0, calls=0;
  void flush_writes() {}
  bool admit() {
''' + body + '''    return true;
  }
};
int libusb_handle_events_timeout_completed(void* context, timeval* wait, int*) {
  auto& p=*static_cast<Probe*>(context);
  if (++p.calls > 8) throw std::runtime_error("unbounded full-queue wait");
  if (wait->tv_usec) {
    assert(wait->tv_sec==0 && wait->tv_usec==2000);
    if (p.mode==0) p._tx_inflight=255;
    if (p.mode==2) return -1;
    if (p.mode==3) { p._tx_shutdown=true; p._tx_inflight=0; }
  }
  return 0;
}
int main() {
  try {
    for (int mode=0;mode<4;++mode) {
      Probe p; p.mode=mode; p._tx_inflight=256;
      assert(p.admit()==(mode==0) && p.calls<=2);
    }
    Probe healthy;
    assert(healthy.admit() && healthy.calls==1);
    Probe stopped; stopped._tx_shutdown=true;
    assert(!stopped.admit() && stopped.calls==0);
  } catch(const std::exception& e) { std::fprintf(stderr,"%s\\n",e.what()); return 1; }
}
'''
with tempfile.TemporaryDirectory(prefix='usb-tx-slot-', dir=sys.argv[1]) as temporary:
    root = Path(temporary)
    (root / 'check.cpp').write_text(code)
    subprocess.run(['c++', '-std=c++20', '-O2', str(root / 'check.cpp'), '-o', str(root / 'check')], check=True)
    subprocess.run([str(root / 'check')], check=True)
print('PASS: healthy/draining/stalled/error/shutdown TX admission')
