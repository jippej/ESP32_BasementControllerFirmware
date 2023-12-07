# ESP32 Basement Controller Firmware
Firmware for the ESP32 Basement controller board

This programm is running on the Wireless-tag board: WT32-ETH01 ( http://www.wireless-tag.com/portfolio/wt32-eth01/ ) . The WT32-ETH01 is mounted as a shield on the JEEC Basement controller board to be found in my repositories

Following functionality is included:
1) Rainwater barrel level measurement 
  - Based on the principle as described here: https://playground.arduino.cc/Main/Waterlevel/
2) Rainwater and Mains water meter readout
  - Based on the implementation as described here: https://www.pieterbrinkman.com/2022/02/02/build-a-cheap-water-usage-sensor-using-esphome-home-assistant-and-a-proximity-sensor/
3) Temperature measurements of our swimming pond
  - By using two DS18B20 waterproof temperature sensors
4) Basement ventilation system 
  - Dewpoint measurements by using two BME280 sensors. One inside, one outside
  - Control of input valve, output valve and fan
  - idea and working is based on the work of: https://github.com/mvniemi/arDewpoint
5) All data is sent further via MQTT 
  
