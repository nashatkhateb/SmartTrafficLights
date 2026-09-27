# config/

`scenarios.json` holds the study §10 scenarios S1–S10 as data. `smarttraffic/traffic_gen.py`
loads and validates it:

- all 8 movements are listed in every rate set;
- rates are valid for the §5 headway model (q = 0, or 0 < q < 2400 veh/h);
- rate segments cover the whole time axis without gaps;
- events are well-formed.

Only `segments` and `events` affect generation. Their SHA-256 is stored in every arrival
file, so an arrival file made with an older config is detected instead of reused.

`hardware.json` (Stage 9) maps real detectors and physical signal heads to ESP32 devices and
channels. Every detector not listed there is virtual. See `hardware/README.md`.
