# Inertial sensor channels: raw or processed? A question for the data/electronics team

Draft for review by the author before sending. Contains no lap times, speeds,
forces or setup values; tracks and loggers are anonymised.

## Why we ask

Our analysis tool estimates the car's sideslip angle (the angle between where
the car points and where it travels) from the yaw-rate gyro and the lateral
accelerometer. Sideslip is not measured directly, so it is built by
integrating the difference between the two signals over time. Integration
makes it very sensitive to small sensor errors: a constant offset of a
fraction of a degree per second becomes several degrees of sideslip within a
corner. We therefore need to know exactly what the logged inertial channels
are, before we can correct them properly.

## What we observe in the logged data

1. **The sensor-cluster accelerometers (sclu_acc_*) are filtered before
   they reach the logger.** Both lateral accelerometers are logged at
   100 Hz. Below about 10 Hz sclu_acc_y and log_acc_y carry the same
   frequency content. Above about 15 Hz they separate: log_acc_y shows the
   car's vibration up to the top of the logged band, while sclu_acc_y falls
   15-20 dB below it. The same pattern holds for x and z. A raw sensor would
   show the vibration; a low-pass filter removes it. The sclu gyro channels
   also roll off strongly (about 30 dB). With no unfiltered gyro to compare
   against, that alone does not prove filtering, because yaw and roll motion
   naturally fade at high frequency. It is consistent with the
   accelerometers, though.
2. **The sclu signals arrive as a bus message.** Every file also contains
   sclu_crc_tx1-3, sclu_msg_cnt_tx1-3, per-signal *_stat channels and
   sclu_HW_index. The message counter is a 4-bit alive counter (0-15). It
   advances by about 10 per 0.1 s, i.e. a 100 Hz frame. This is the usual
   pattern of a safety-relevant control unit that sends processed values
   over CAN. The status channels read 0 throughout.
3. **The roll-rate channel contains part of the yaw rate.** Over each corner
   the integrated roll rate does not return to zero. The leftover grows with
   the corner's heading change and takes the sign of the turn direction. That
   behaviour fits an axis misalignment of about 1.2-2.5 degrees, or a
   cross-axis sensitivity of similar size. The apparent angle differs from
   session to session and groups by track (about 1.2-1.3 deg on one track,
   1.9-2.5 deg on another), on a single sensor bolted to the chassis. A
   fixed mounting misalignment would give one constant angle. Changes in the
   car's static attitude (rake, ride height) are far too small to explain the
   differences, and they point the wrong way.
4. **Two lateral accelerometers disagree by a constant factor.** sclu_acc_y
   reads 4-8.5 % lower than log_acc_y on every session, on both loggers. The
   data cannot tell which one is right. log_acc_y also reaches its ±5 g
   limit on some files, while sclu_acc_y stays below about ±3 g. On one
   logger, log_acc_y also carries a constant offset of about 0.04-0.05 g
   that does not follow the car's attitude.
5. **The second yaw-rate channel is a copy.** MRR_EgoYawRate (10 Hz) matches
   sclu_yaw_rate almost exactly (correlation 0.998-0.999) but arrives
   0.24-0.30 s later. It is a delayed, down-sampled copy of the same signal
   on the bus, not an independent second gyro. It is all zeros on the
   other logger.
6. **A third accelerometer set (ADR2_acc_x/y/z)** is logged at 10 Hz with a
   coarse 0.07 g resolution. It is too coarse and too slow for our use, but
   it shows the logger can record more than one inertial source.
7. **Small constant gyro bias per session.** The yaw gyro shows a constant
   bias of up to about 0.35 deg/s that differs between sessions. We can
   measure it after the fact from heading closure over a lap, but only
   roughly.

## What raw channels would let us do

- **Remove sensor bias and scale before integrating**, using the sensor's own
  known characteristics instead of estimating them from driving data.
- **Measure body roll properly.** With the yaw leak removed, a clean roll
  rate is needed to correct the lateral accelerometer for the tilt of the
  car.
- **Settle which lateral accelerometer is correct**, which today is a 4-8 %
  uncertainty in every lateral force we compute.
- **Know the filter.** If the processed channels must stay, their filter
  (type, cut-off, delay) lets us account for it instead of guessing.

## The concrete ask

1. **Channel list:** which inertial signals exist on the bus where the
   sensor cluster sits? In particular, are there raw (unfiltered, uncompensated)
   gyro and accelerometer values besides the sclu_* channels we log?
2. **Definition of the logged channels:** for sclu_yaw_rate,
   sclu_roll_rate, sclu_pitch_rate and sclu_acc_x/y/z: raw sensor output or
   processed (filtered, bias-compensated, axis-transformed)? If processed,
   which filter and which compensation? Is sclu_roll_rate a body-axis rate?
3. **Mounting:** the sensor cluster's mounting orientation (any intended
   tilt), and whether log_acc_* sit in the logger itself and where.
4. **One test day:** if raw channels exist on the bus, add them to the logging
   configuration for one test day, at their native rate. This is a logging
   configuration change only, no new hardware.
