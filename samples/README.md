# Sample CAN logs

Reference captures checked into the repo for analysis and regression.

| File | Notes |
|------|-------|
| `jeep_topfly_torch_x310_scan_20260831.trc` | Jeep (Stellantis 29-bit), **Topfly Torch X310** + our vehicle scan. Valid multi-ECU Mode 01 PID 01 (PCM `18DAF110` → MIL ON / 1 DTC) and Mode 03 → **P0113**. VIN `1C4RJYB66S8701179`. PEAK logger, 2026-08-31 ~13:49. |
| `jeep_topfly_torch_x310_traffic_20260831.trc` | Same vehicle/session window — longer Topfly Torch X310 bus traffic (PID polling, Mode 03 `43 01 01 13` = P0113). PEAK logger, 2026-08-31 ~13:51. |
| `jeep_bouncie_mil_dtc_20260831.trc` | Earlier Jeep capture (misnamed “Bouncie”; device was Topfly). Kept for history. |

Raw runtime logs stay in `logs/` (gitignored). Copy keepers into `samples/` to version them.
