# HP-H09 SU TxBF bench

`bfbench` is an isolated GPL-2.0 experiment for the Jaguar3 RTL8822E backend.
It does not enable BF in the production transport. The sender steers a VHT
single stream; the receiver keeps both RX paths and returns SU compressed CSI.

Build the `bfbench` and `VhtSoundingSelftest` targets with C++20 and libusb.
Run `ctest -R vht_sounding --output-on-failure` for NDPA, feedback length/address/
dimension/token/FCS checks, freshness boundaries, TX descriptors, and PHY BF flags.

On the two assigned H09 adapters, after stopping their RF58 and link services,
load their existing RF environment and run the receiver first:

```sh
bfbench rx 95 1 0
# On the sender, sequentially:
bfbench txoff 30 1 100
bfbench txon 30 1 101
```

The bench is fixed to channel 153 / 20 MHz, VHT1SS MCS7 / LDPC / SGI,
1100-byte payloads, local sender MAC `02:42:58:49:00:11`, and receiver
`02:42:58:49:00:12`. Both TX arms request the same -24 qdB offset and require a non-saturated
hardware readback. This is a nominal index-based offset, not measured RF dB;
no flat TXAGC override is used. These identities are specific to this bench.
The chip's calibration and both receive chains remain intact. No USB reset is
issued. Restore the original RF58/link services after the experiment.

Data uses QoS TID 0 with the destination as BSSID, matching the production
ACK/aggregation path. Paced batches reuse `send_packets` and its existing
three-frame USB aggregation. `BF_USB` records time spent sending and hard
submission failures; a failed partial URB is not counted as fully submitted.

For a control-read comparison, `txread` keeps BF off but reads the same CSI
register at 10 Hz. `BF_PHY` separates ordinary/BF PHY samples by run ID and
only counts full decoded pages for SNR/EVM. Values are raw sums: divide the
SNR/EVM sums by `full_n` and then 2 for dB, following the vendor PHY format.
Per-path SNR is not the post-equalizer EVM and neither establishes range.

Both TX arms sound every 100 ms with a real 19-byte VHT NDPA and a hardware
2-stream NDP. `txoff` receives CSI but never applies it. `txon` calls
`PollBeamforming(500)` every 10 ms: a complete current-token peer report must
arrive within 150 ms and match `RX_CSI_RPT_INFO[24:1]` before apply. The final
two seconds deliberately stop sounding to exercise stale-CSI fallback.
Channel/bandwidth changes clear feedback; identity changes retire the BF session.
These APIs belong to the single TX/control owner, never the RX callback.

Compare receiver unique payload counts by run ID, PHY `beamformed` flags,
per-path signal observations, feedback age, USB errors and TX reports. The
1100-byte count includes the bench header. A PHY BF flag proves the transmitted
format, not antenna gain or range. TX report counts are sampled and are not a
complete delivery ledger. This static near-field bench cannot establish range,
mobile-channel gain, production throughput, control latency, or coexistence.
