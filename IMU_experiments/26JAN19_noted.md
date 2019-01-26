# 26JAN19 Meeting Notes

## IMU Measurements

Each IMU has different resting acceleration and gyroscope measurements - comparing different devices to see how this can be accounted for.

### studying effect of rotation - if axes have their own offsets this may show up in the data

IMU4, IMU5, IMU6 are measurements taken from the same IMU but rotated so that each axis is the principal direction.

### studying position measurement

Lots of drift with existing integration estimation
Estimating 10cm movement

study drift behaviour, compare integrated estimations to accurately calculating from acceleration measurements once all data is saved.

## Changes to code

Update code to be more modular.