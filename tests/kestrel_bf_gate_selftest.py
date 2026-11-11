"""Exercise the actual Kestrel CSI polling method without radio hardware."""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(__file__).resolve().parents[1] / 'src/kestrel/RtlKestrelDevice.cpp'
method = source.read_text(encoding='utf-8').split(
    'void RtlKestrelDevice::PollBeamforming', 1)[1].split(
    '\nvoid RtlKestrelDevice::ClearBeamforming', 1)[0]
code = r'''
#include "VhtSounding.h"
#include <atomic>
#include <cassert>
#include <mutex>
uint64_t now_ms = 100;
namespace devourer { uint64_t beamforming_now_ms() { return now_ms; } }
struct Hal {
  bool valid = true, matrix = false;
  unsigned reads = 0;
  bool su_sounding_valid() { ++reads; return valid; }
  void apply_su_matrix(bool enable) { matrix = enable; }
};
struct RtlKestrelDevice {
  bool _bf_armed = true;
  std::mutex _bf_mutex;
  std::atomic<bool> _bf_apply_requested{true}, _bf_active{false};
  std::atomic<uint64_t> _bf_feedback_ms{100};
  Hal _hal;
  void PollBeamforming(uint64_t, bool allow_apply = true);
};
'''
code += 'void RtlKestrelDevice::PollBeamforming' + method
code += r'''
int main() {
  RtlKestrelDevice radio;
  radio.PollBeamforming(500, false);
  assert(!radio._hal.matrix && radio._hal.reads == 0);
  radio.PollBeamforming(500);
  assert(radio._bf_active && radio._hal.matrix);
  now_ms = 601; // Expiry must retire CSI even with no pending apply request.
  radio.PollBeamforming(500, false);
  assert(!radio._bf_active && !radio._hal.matrix && radio._hal.reads == 1);
  radio._bf_feedback_ms = now_ms;
  radio._bf_apply_requested = true;
  radio.PollBeamforming(500);
  assert(radio._bf_active && radio._hal.matrix);
  radio._bf_apply_requested = true;
  radio._hal.valid = false;
  radio.PollBeamforming(500);
  assert(!radio._bf_active && !radio._hal.matrix);
}
'''
output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='kestrel-bf-gate-', dir=output) as directory:
    path = Path(directory)
    (path / 'check.cpp').write_text(code, encoding='utf-8')
    subprocess.run(['c++', '-std=c++20', '-I', str(source.parents[1]),
                    str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
    subprocess.run([str(path / 'check')], check=True)
print('PASS: Kestrel apply gating, CSI expiry and invalid-matrix retirement')
