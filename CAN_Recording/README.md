# CAN Recording — MiviaCar

## candump-2026-06-24_154259.log

**Date**: 2026-06-24 15:42:59  
**Interface**: can1  
**Vehicle**: MiviaCar — MIVIA Lab, UNISA  
**Radar**: Continental ARS408-21 (SensorID=0, front)  
**Firmware**: v4.30.2  
**Duration**: ~37.9 seconds  

### Radar configuration
| Parameter | Value |
|-----------|-------|
| SensorID | 0 (front) |
| OutputType | 1 (Objects) |
| MaxDistance | 200 m |
| SendExtInfo | 1 ✅ |
| SendQuality | 1 ✅ |
| MotionRxState | SPEED+YAW_MISSING (vehicle stationary) |

### Scene description
Vehicle stationary in parking area. 3 stable objects detected:

| ID | Class | X (m) | Y (m) | Range | RCS |
|----|-------|--------|--------|-------|-----|
| 56 | POINT | 5.60 | +0.60 | 5.63 m | +14.0 dBm² |
| 32 | **CAR** | 11.40 | +0.20 | 11.40 m | **+26.0 dBm²** |
| 0 | WIDE | 17.60 | -0.40 | 17.60 m | +27.5 dBm² |

### Replay
```bash
# Replay on vcan0
canplayer -I candump-2026-06-24_154259.log vcan0=can1 -l 99

# Decode with Python
python3 test_decode.py candump-2026-06-24_154259.log
```
