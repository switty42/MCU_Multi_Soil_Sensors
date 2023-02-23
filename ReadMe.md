# MCU Multi Soil Sensors
Docs 2-22-23 V1 (For questions or comments:  Stephen Witty switty@level500.com)  

### Project Overview:
The project is a POC to test using multiple soil sensors and types using an MCU.   In this case, the MCU is a RAK4631 Helium LoRa development board.   Two brands of soil sensors were tested with support for multiple numbers of each.  The code supports independent calibration of each soil sensor.  The code also supports operating a relay that engages a water pump that engages when a specified soil sensor reports a given level of moisture.  The soil sensor status is periodically written to a serial port to display on a debugger.  The soil sensor data is periodically sent over Helium LoRa to provide backend data for analysis.

This project is demo/POC ready only.

### Many thanks:
- The Mycelium Networks Builder Program for support and encouragement
- RAK Wisblock for excellent hardware offerings and code examples

<img src="Pics/Test_Board.jpg" width="200"> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;<img src="Pics/Mycelium.png" width="100">

### Demo Link:
https://www.youtube.com/watch?v=1ZjJgnmptXM

### Hardware:
- RAK4631 - MCU
- RAK5005-0 - Baseboard
- RAK13002 - Expansion board
- LED and a 1K resistor
- ACEIRMC Soil Moisture Sensor(s)
- AITRIP Capacitive Soil Moisture Sensor(s)
- Ximimark 16CH Analog Digital Multiplexer Breakout Board CD74HC4067
- Relay to operate a water pump if desired

### Project files:
- Documentation - ReadMe.md
- Primary Source Code - Multi_Soil_Sensors.ino (Aurdrino sketch)
- Header file - keys.h (Contains Helium user application keys)
- DataCake decoder file - DataCake_Soil_Decoder.txt
- Ubidots decoder file - Ubidots_Soil_Decoder.txt

### Circuit:
- WB_IO1 - Connected to MUX S0 - Mux control to select Soil Sensor
- WB_IO3 - Connected to MUX S1 - Mux control to select Soil Sensor
- WB_IO4 - Connected to MUX S2 - Mux control to select Soil Sensor
- WB_IO5 - Connected to relay controlling pump control if desired
- WB_IO6 - Connect to LED with 1K resistor to ground
- Soil Sensors connect to MUX analog I/O pins

### Operation:
When powered on, the MCU displays soil sensor and pump status every few seconds via the serial port.  The soil sensor data is sent over Helium LoRa periodically if connected.  The pump relay control output will engage if the selected soil sensor is read as too dry.  If the relay output is engaged, the LED connected to WB_IO6 will also light.

The following data is output across serial for each soil sensor connected:

- Raw Sensor Data
- [Average of the Raw Sensor Data]
- (Percent of moisture based on calibration)
- (A remapped number to a commercial sensor range)

After sensor data is displayed, a pump running string will display if the relay control pin is engaged.


### LED meaning:
LED on WB_IO6 will light if pump logic is engaged.

### Misc:

Relevant values that can be changed can be found as #define values in the source.  These are documented as code comments.

Sensor calibration is also done by #define values.  These values can be determined by using a given sensor in dry air and water to get extreme values.

Two decoders are supplied.  The DataCake decoder is designed to run without any Helium function decoder attached.  The Ubidots decoder is to be installed as a Helium Console function decoder.  The free version of Ubidots does not allow decoders.

The code supports mixing sensor types and numbers.

Insert your own Helium LoRa keys into keys.h
