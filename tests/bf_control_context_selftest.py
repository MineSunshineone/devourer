"""Regression guard: valid feedback must not issue synchronous USB I/O in RX."""
from pathlib import Path

root = Path(__file__).resolve().parents[1]
jaguar = (root / 'src/jaguar3/RtlJaguar3Device.cpp').read_text(encoding='utf-8')
start = jaguar.index('// Register updates, sounding tokens')
callback = jaguar[start:jaguar.index('_packetProcessor(p)', start)]
kestrel = (root / 'src/kestrel/RtlKestrelDevice.cpp').read_text(encoding='utf-8')
kestrel_callback = kestrel.split('void RtlKestrelDevice::receive_beamforming_feedback')[1]
kestrel_callback = kestrel_callback.split('void RtlKestrelDevice::PollBeamforming')[0]
for receive in (callback, kestrel_callback):
    assert '_bf_apply_requested.store(true)' in receive
    assert not any(operation in receive for operation in
                   ('apply_vmatrix', 'apply_su_matrix', 'rtw_read', 'rtw_write',
                    'su_sounding_valid', 'PollBeamforming'))
assert 'void RtlJaguar3Device::PollBeamforming' in jaguar
assert 'void RtlKestrelDevice::PollBeamforming' in kestrel
assert 'set_bf_data_mapping(true)' in jaguar.split('void RtlJaguar3Device::PollBeamforming')[1].split('void RtlJaguar3Device::ClearBeamforming')[0]
assert 'set_bf_data_mapping(false)' in jaguar.split('void RtlJaguar3Device::ClearBeamforming')[1].split('void RtlJaguar3Device::set_bf_data_mapping')[0]
print('PASS: both RX feedback handlers defer USB control I/O to the TX owner')
