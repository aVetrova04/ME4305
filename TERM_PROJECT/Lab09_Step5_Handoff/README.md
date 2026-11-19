# Lab 09 Step 5 handoff

**Baseline:** `lab09-step5-10ms-observer-p`. This is the verified build that completed the 2 and 12 rad/s trials. Begin Step 6 from this checkpoint.

| Path | Contents |
| --- | --- |
| `firmware/` | The original Romi source and build manifest, verified against the completed run's hashes. |
| `romi_lab09_step5_auto.py`, `requirements.txt` | Original Mac collector and its dependencies. |
| `step4_reference/` | The exact Step 4 CSVs used for the Step 5 comparison, plus their config. Keep this path next to the collector. |
| `results/step5/` | Both raw CSVs, individual and comparison plots, the original run config, serial log, and scheduler profile. |
| `results/step6/` | Folder for new PI runs. |

## Install the original Step 5 firmware

If Romi is running the experimental R1 or R2 firmware, stop the robot and keep the wheels raised. From `firmware/`, replace `main.py`, `taskbaseline.py`, `taskmotor.py`, and `observer.py` on Romi's USB drive. The other original firmware files are included for a full source snapshot. Leave the working `cotask` module on Romi.

The recorded checkpoint depends on `cotask` but does not contain its source. If the board has a file named `cotask.py`, copy that existing file into `firmware/` before committing so the dependency is included. If the module is built into the board's firmware, use the same course firmware image.

Eject the drive, close PuTTY, and from this folder run:

```sh
python romi_lab09_step5_auto.py
```

Confirm the ready message ends in `lab09-step5-10ms-observer-p`. Follow the raised checks before moving to the track. To check only with the wheels raised, pass `--bench-only`.

## For Step 6

Use the two Step 5 CSVs as the observer-P reference and the two Step 4 CSVs as the encoder-P reference. Keep the 10 ms nominal period, ±6 V limits, and 2 and 12 rad/s test setpoints for the comparisons. The Step 5 profile records real timing variation; the 10 ms target is not a hard timing guarantee. Use a separate build ID and collector for Step 6 so the original reference stays reproducible.
