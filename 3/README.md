# Two-board communication test

This folder has two independent PlatformIO projects:

```text
3/s3_sender    flash this to the ESP32-S3 CAM/MPU board
3/v1_receiver  flash this to the V1 ESP32 car board
```

Pins match the Python version:

```text
S3 GPIO45 TX -> V1 GPIO22 RX
S3 GPIO46 RX <- V1 GPIO23 TX
GND common
baud 115200
```

The sender emits:

```text
IMU,seq,pitch,roll,yaw*CRC
```

The receiver is a C++ translation of `line_tracker/interfaces/attitude_link.py`.
