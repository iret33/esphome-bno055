# ESPHome BNO055 Component

ESPHome component for the Bosch BNO055 9-DOF Absolute Orientation Sensor with onboard sensor fusion.

## Features

- Euler angles (heading, roll, pitch)
- Quaternion orientation (w, x, y, z)
- Linear acceleration (gravity removed)
- Gravity vector
- Raw accelerometer
- Gyroscope
- Magnetometer
- Temperature

## Installation

Add this to your ESPHome configuration:

```yaml
external_components:
  - source: github://iret33/esphome-bno055
    components: [bno055]
```

## Usage

### Minimal Configuration

```yaml
external_components:
  - source: github://iret33/esphome-bno055
    components: [bno055]

i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: bno055
    euler_heading:
      name: "Heading"
    euler_roll:
      name: "Roll"
    euler_pitch:
      name: "Pitch"
```

### Full Configuration

```yaml
external_components:
  - source: github://iret33/esphome-bno055
    components: [bno055]

i2c:
  sda: GPIO21
  scl: GPIO22
  frequency: 50kHz  # Recommended for ESP32

sensor:
  - platform: bno055
    address: 0x29
    update_interval: 1s
    # Euler angles
    euler_heading:
      name: "BNO055 Heading"
    euler_roll:
      name: "BNO055 Roll"
    euler_pitch:
      name: "BNO055 Pitch"
    # Quaternion
    quaternion_w:
      name: "BNO055 Quaternion W"
    quaternion_x:
      name: "BNO055 Quaternion X"
    quaternion_y:
      name: "BNO055 Quaternion Y"
    quaternion_z:
      name: "BNO055 Quaternion Z"
    # Linear acceleration
    linear_accel_x:
      name: "BNO055 Linear Accel X"
    linear_accel_y:
      name: "BNO055 Linear Accel Y"
    linear_accel_z:
      name: "BNO055 Linear Accel Z"
    # Gravity
    gravity_x:
      name: "BNO055 Gravity X"
    gravity_y:
      name: "BNO055 Gravity Y"
    gravity_z:
      name: "BNO055 Gravity Z"
    # Raw accelerometer
    accel_x:
      name: "BNO055 Accel X"
    accel_y:
      name: "BNO055 Accel Y"
    accel_z:
      name: "BNO055 Accel Z"
    # Gyroscope
    gyro_x:
      name: "BNO055 Gyro X"
    gyro_y:
      name: "BNO055 Gyro Y"
    gyro_z:
      name: "BNO055 Gyro Z"
    # Magnetometer
    mag_x:
      name: "BNO055 Mag X"
    mag_y:
      name: "BNO055 Mag Y"
    mag_z:
      name: "BNO055 Mag Z"
    # Temperature
    temperature:
      name: "BNO055 Temperature"
```

## Configuration Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `address` | No | `0x29` | I2C address (0x28 or 0x29) |
| `update_interval` | No | `1s` | How often to read sensor |

### Sensor Outputs

| Category | Sensors | Unit |
|----------|---------|------|
| Euler Angles | `euler_heading`, `euler_roll`, `euler_pitch` | degrees |
| Quaternion | `quaternion_w`, `quaternion_x`, `quaternion_y`, `quaternion_z` | unitless |
| Linear Acceleration | `linear_accel_x`, `linear_accel_y`, `linear_accel_z` | m/s² |
| Gravity | `gravity_x`, `gravity_y`, `gravity_z` | m/s² |
| Raw Acceleration | `accel_x`, `accel_y`, `accel_z` | m/s² |
| Gyroscope | `gyro_x`, `gyro_y`, `gyro_z` | °/s |
| Magnetometer | `mag_x`, `mag_y`, `mag_z` | µT |
| Temperature | `temperature` | °C |

## ESP32 I2C Note

The BNO055 uses I2C clock stretching which can cause issues on some ESP32 variants. If you experience communication problems, reduce the I2C frequency to 50kHz:

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22
  frequency: 50kHz
```

## Calibration

The BNO055 performs automatic calibration during operation:

1. **Gyroscope**: Keep sensor still for a few seconds after power-on
2. **Magnetometer**: Move sensor in figure-8 pattern
3. **Accelerometer**: Place sensor in multiple orientations

## Links

- [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [ESPHome PR](https://github.com/esphome/esphome/pull/13616) - Official ESPHome integration (pending)

## License

MIT License
