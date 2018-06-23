# ChickenControl
Arduino-based chicken coop controlling and monitoring software

This project deals with putting as much electronics as possible into a chicken coop. It covers useful things like automatic coop door control as well as because-we-can things like RFID-based egg-laying monitoring.

**Features**

* solar power monitoring (current from panel, to/from battery, to load; battery and panel voltage)
* water level monitoring (alerting when water reservoir level is below threshold
* water temperature monitoring (alerting when pipes are about to freeze
* door control (manual/light level/time)
* egg logging (RFID tags on legs of chickens, reader at entrance of nest, microswitch where egg falls down)
* ethernet connectivity (coop has LAN that connects to upstream LAN via WLAN bridge.)
* uploading of metrics (temperatures, water alarm, humidity, pressure, light level, door state) to time series database (InfluxDB)

**Hardware**

* I chose an Arduino mega because I wanted more than 2 interrupt pins (I use them for RFID readers).
* Real time clock
* I2C light sensor
* I2C temp/pressure/humidity sensor BME280
* I2C 12-bit ADC for higher resolution when measuring current
