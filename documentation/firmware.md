# Firmware setup and configuration

This section provides installation and configuration notes for the firmware for various components: the Dynamixel servos, the openRB 150 controller, the INA219 sensors, and the ESP32 controller (used for the sensors). 

## Sections
- [OpenRB 150](./firmware.md#openrb_150)
- [Dynamixel servos](./firmware.md#Dynamixel_servo)
- [ESP32 and Current Sensors](./firmware.md#esp32_and_current_sensors)

## OpenRB 150
- link to Robotis documentation: https://emanual.robotis.com/docs/en/parts/controller/openrb-150/
<p align="center">
  <img src="../images/openRB.sketch.jpg" alt="open rb 150 sketch" width="600">
</p>

## Dynamixel servos
- setting each servo id
- changing the baud rate
<p align="center">
  <img src="../images/setup.dyn.wiz.scan.jpg" alt="scan" width="600">
  <img src="../images/successful.dyn.wiz.scan.jpg" alt="successful scan" width="600">
</p>

- attempting to restrict the range of motion

<p align="center">
  <img src="../images/servo-2.dyn.wiz.max.min.pos.jpg" alt="range of motion for servo 2" width="600">
</p>

- additional telemetry

<p align="center">
  <img src="../images/servo-6.present.current.jpg" alt="more telemetry" width="600">
</p>

## ESP32 and Current Sensors

- I2C addresses and current sensors

<p align="center">
  <img src="../images/ina219-i2c-address.jpg" alt="i2c addressing on INA219" width="600">
</p>
