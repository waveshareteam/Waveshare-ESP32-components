# ChangeLog

## v2.0.1 - 2026-08-18

### bugfix:

* Fixed the 24-bit hardware timestamp unwrap in `qmi8658_read_sensor_data()`. It overwrote `dev->timestamp` instead of accumulating onto it, so the value jumped backwards at every counter wrap instead of staying monotonic.
