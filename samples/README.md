# Sample CAN logs

Reference captures checked into the repo for analysis and regression.

| File | Notes |
|------|-------|
| `chevy_equinox_2027_tft_scan_20260901_1657.trc` | **2027 Chevrolet Equinox** + **TFT** OBD tool. Vehicle scan / traffic session starting ~**16:57** (closest to 4 PM). VIN `3GNARHEG9VL117043` via UDS DID F190 on `18DAF180`. Multi-ECU J1979-2 F4xx polling. ~1.2 MB, PEAK logger, 2026-09-01. |
| `chevy_equinox_2027_tft_scan_20260901_1452.trc` | Same Equinox — shorter **vehicle scan** from our logger (~14:52). VIN + Mode 01/03 / J1979-2 requests; responses on `18DAF180` / `18DAF111`. |
| `chevy_equinox_2027_tft_20260901_1441.trc` | Same Equinox — earlier capture (~14:41) with VIN multi-frame on `18DAF180`. |
| `chevy_equinox_2027_tft_20260901_1715.trc` | Same Equinox — short TFT follow-on (~17:15) after the 16:57 session. |
| `jeep_topfly_torch_x310_scan_20260831.trc` | Jeep (Stellantis 29-bit), **Topfly Torch X310** + our vehicle scan. Valid multi-ECU Mode 01 PID 01 (PCM `18DAF110` → MIL ON / 1 DTC) and Mode 03 → **P0113**. VIN `1C4RJYB66S8701179`. PEAK logger, 2026-08-31 ~13:49. |
| `jeep_topfly_torch_x310_traffic_20260831.trc` | Same Jeep/session — longer Topfly Torch X310 bus traffic. |
| `jeep_bouncie_mil_dtc_20260831.trc` | Earlier Jeep capture (misnamed “Bouncie”; device was Topfly). Kept for history. |

Raw runtime logs stay in `logs/` (gitignored). Copy keepers into `samples/` to version them.
