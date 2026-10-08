"""Run the actual polling method against a register stub, without radio hardware."""
from pathlib import Path
import subprocess
import sys
import tempfile

source = Path(__file__).resolve().parents[1] / 'src/jaguar3/RtlJaguar3Device.cpp'
method = source.read_text(encoding='utf-8').split(
    'void RtlJaguar3Device::PollBeamforming', 1)[1].split(
    '\nvoid RtlJaguar3Device::ClearBeamforming', 1)[0]
code = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <mutex>
uint64_t now_ms = 100;
struct Registers {
  uint32_t control = 0x123 << 1;
  bool matrix = false;
  unsigned reads = 0;
  uint32_t rtw_read32(uint16_t address) {
    assert(address == 0x071c); ++reads; return control;
  }
};
namespace devourer {
uint64_t beamforming_now_ms() { return now_ms; }
namespace bf {
void apply_vmatrix(Registers& registers, bool enable, uint8_t) {
  registers.matrix = enable;
}
}}
struct Logger {
  template<class... T> void warn(T...) {}
  template<class... T> void info(T...) {}
};
struct RtlJaguar3Device {
  bool _bf_txbf_armed = true;
  std::atomic<bool> _bf_apply_requested{true}, _bf_apply_on{false};
  std::atomic<uint64_t> _bf_feedback_ms{100};
  std::atomic<uint32_t> _bf_report_control{0x123};
  std::atomic<uint8_t> _rx_bw_code{0};
  unsigned _bf_hw_mismatches = 0;
  std::mutex _reg_mu;
  Registers _device;
  Logger logger;
  Logger* _logger = &logger;
  bool steered_mapping = false;
  void set_bf_data_mapping(bool enable) { steered_mapping = enable; }
  void PollBeamforming(uint64_t);
};
'''
code += 'void RtlJaguar3Device::PollBeamforming' + method
code += r'''
int main() {
  RtlJaguar3Device radio;
  radio.PollBeamforming(500);
  assert(radio._bf_apply_on && radio._device.matrix && radio.steered_mapping);
  radio._bf_apply_requested = true;
  radio._bf_report_control = 0x456; // New host report, old hardware matrix.
  radio.PollBeamforming(500);
  assert(!radio._bf_apply_on && !radio._device.matrix && !radio.steered_mapping);
  assert(!radio._bf_apply_requested && radio._device.reads == 2);
  radio._device.control = 0x456 << 1;
  radio._bf_apply_requested = true;
  radio.PollBeamforming(500);
  assert(radio._bf_apply_on && radio._device.matrix && radio.steered_mapping);
  now_ms = 601;
  radio._bf_apply_requested = true;
  radio.PollBeamforming(500);
  assert(!radio._bf_apply_on && !radio._device.matrix && !radio.steered_mapping);
}
'''
output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='bf-gate-', dir=output) as directory:
    path = Path(directory)
    (path/'check.cpp').write_text(code, encoding='utf-8')
    subprocess.run(['c++', '-std=c++20', str(path/'check.cpp'), '-o', str(path/'check')], check=True)
    subprocess.run([str(path/'check')], check=True)
print('PASS: each CSI refresh is hardware-validated; mismatch and expiry restore normal TX')
