# Diagnostics

Scripts from bringing the arm up, kept for reference. They are not used by the backend.

- `bus_probe.py`: raw pyserial probe to tell Feetech (`FF FF`) framing from Elephant's `FE FE` framing.
- `pymycobot_test.py`: tries the official pymycobot library over `/dev/ttyAMA0`.
- `gpio_enable_test.py`: tries GPIO enable pins on the Pi base board's bus buffers.

The pymycobot ones expect a `pymycobot/` checkout in the repo root. `pip install -r tools/diagnostics/requirements.txt`.
Stop the backend first: only one program can have the serial port open.
