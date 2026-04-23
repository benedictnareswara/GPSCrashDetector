# Mega to ESP32 Bridge Protocol

## Transport
- UART: Mega Serial2 <-> ESP32 Serial2
- Baud: 115200
- Framing: one CSV packet per line (`\n` terminated)

## Event Packet (current)
`EVT,<seq>,<event>,<valid>,<lat>,<lon>,<age_ms>,<tilt_deg>,<accel_g>`

Fields:
- `seq`: monotonically increasing unsigned integer from Mega
- `event`: `MANUAL`, `CRASH_START`, `CANCELED`, `CRASH_CONFIRMED`, `CLEAR`
- `valid`: `1` if GPS fix fresh, otherwise `0`
- `lat`, `lon`: decimal degrees
- `age_ms`: age of GPS fix in milliseconds; `0` when invalid
- `tilt_deg`: computed tilt in degrees
- `accel_g`: acceleration magnitude in g

## Legacy Packet (migration support)
`GPS,<seq>,<valid>,<lat>,<lon>,<age_ms>`

ESP32 parser currently accepts both formats.

## Trigger Model on Mega
- Crash trigger when: `tilt >= 35.0` OR `accel >= 1.35g`
- Crash cancel window: 10 seconds after `CRASH_START`
- If button pressed inside window: emit `CANCELED`, buzzer OFF, return standby
- If window expires without button press: emit `CRASH_CONFIRMED`, buzzer stays ON until button press
- Standby button behavior: first press emits `MANUAL` and latches buzzer ON; next press emits `CLEAR` and turns buzzer OFF
- Cooldown after crash exit: 3 seconds before next crash trigger
