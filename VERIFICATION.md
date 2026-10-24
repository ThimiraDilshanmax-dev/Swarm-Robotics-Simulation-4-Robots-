# Verification: fading trails and communication triangles

Checked on 24 October 2026 with Webots R2025a and Python 3.12. All three rendered scenarios passed: nominal moving turns, radio-loss braking/recovery, and a stationary master turn. The indoor environment remains in place.

## Current visual behaviour

- Recent paths use single-pixel, muted-colour segments with a peak opacity of 24%.
- Segment opacity decreases continuously using a cubic smoothstep over 12 seconds, reaching zero with a smooth slope. Invisible slots are reused later.
- Trails continue fading while robots are stationary. The early-stop and late-stop screenshots were inspected to confirm disappearance without a hard cutoff.
- Red triangle: master R1, orange R3, purple R4. Green triangle: master R1, blue R2, purple R4.
- Both colours are visible along the shared master-purple diagonal. Every follower links to the master and another follower; purple bridges both triangles.
- Links follow the physical bodies and dim smoothly when radio data becomes stale. All five unique links were fresh at the end of every scenario.

## Communication implementation

The master transmits only measured motion and diagnostic state. It now receives follower status and reports receipt timestamps. Followers exchange peer telemetry between blue-purple and orange-purple using channel 44, with the configured delay and independent packet loss. Peer traffic does not change follower motor steering. The supervisor receives telemetry and draws visual geometry only; no controller receives global pose or future turn targets from it.

## Measured moving-route results

| Measurement | Result |
| --- | --- |
| Rendered run including final fade observation | 106.02 simulated seconds |
| Largest relative turn error across four robots | 0.325 degrees |
| Lowest robot speed during a moving turn | 0.104 m/s |
| Largest formation-position error | 3.01 cm |
| Final formation-position error | 0.72 cm |
| Minimum robot centre separation | 0.772 m |
| Largest observed body tilt | 0.00032 degrees |
| Flips, collisions and mission timeouts | None observed |

All eight moving turns completed. The radio-loss scenario also confirmed a watchdog stop, continued master rotation during the blackout, and resumed follower tracking. The stationary scenario confirmed that all followers detected the master's in-place turn through radio.

The robots still rely on known aligned starting geometry, gyro/encoder odometry and simulated scalar ranging. Results validate this Webots configuration, not hardware ranging or arbitrary-start localization.

The original Simulations directory retains all 85 original files with unchanged SHA-256 hashes and no additional files. See `runtime/verification/source_integrity.json` and the baseline `source_folder_hashes.json`.

![Moving robots with the two reference triangles](runtime/verification/nominal/turn_04.jpg)
