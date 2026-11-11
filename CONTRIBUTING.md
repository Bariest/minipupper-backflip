# Contributing

Thanks for helping. This project improves whenever a real flip is fed back into the simulator, so the most useful contributions are recordings from other robots.

## Share a flip

1. Record it: firmware web page → *Record the next flip* (IMU + 12 servos at 200 Hz), then download the CSV. A phone video at 60 fps (side view, robot fully in frame) helps a lot.
2. Add the CSV to `data/recordings/` as `flip_<n>.csv` and a row to `data/recordings/index.csv`: table played, floor, shoes, and notes. Note the battery voltage if you know it, plus anything unusual about the robot (no display, different servos, extra weight).
3. Re-encode the video small (`ffmpeg -i in.mp4 -an -vf scale=960:-2 -c:v libx264 -crf 27 out.mp4`), put it in `media/` and add a row with its sync offset to `media/README.md`.
4. Run `python scripts/analyze_recording.py flip_<n> --sim <table> --plot check.png` and say in the pull request what you saw.

Not ready for a pull request? Open an issue with the CSV and video attached.

## Code

```bash
pip install -r requirements.txt
python -m pytest            # must pass; includes the C player == Python player check (needs a C compiler)
```

* Keep the firmware player (`firmware/backflip_player.c`) dependency-free C99. It must keep matching `Preset.play()` tick for tick.
* After changing a table in `presets/`, run `python scripts/export_c.py` so the C headers match.
* A figure that changes should be regenerated with `python scripts/make_figures.py <name>` in the same pull request.
* New tables: say in `presets/README.md` whether they have been tried on a real robot, and with what result. Results from the sim alone are welcome but must be labelled as such.

## Safety

Never ask anyone to try an untested table without a soft mat behind the robot. A flip that lands wrong can break a servo horn.
