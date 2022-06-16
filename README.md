# D1M-WX1-APRS-ThingSpeak-OTA-WFM

This firmware is for the *IoT-Kits<sup>&copy;</sup>* wireless, solar-powered weather station based on the Lolin Wemos D1 Mini. It posts your weather data to both ThingSpeak and APRS-IS. You must be a licensed Amateur Radio Operator to use APRS-IS. A ThingSpeak only version is available for people without a ham license.

A captive portal served by the D1 Mini allows easy configuration of the kit with a Wi-Fi-connected cell phone. 

The general steps are:
- Build the kit.
- Configure the Arduino IDE and ThingSpeak channel.
- Upload the firmware to the kit.
- Configure the kit with a Wi-Fi-connected cell phone.
- Place the weather station into service.

## Detailed Instructions
1. Build the kit using the instructions for your version:
 - Single-board kit see https://w4krl.com/iot-kits/d1m-wx1-solar-powered-weather-station/
 - Stacked version see https://w4krl.com/iot-kits/d1s-wx1-solar-powered-weather-station/
2. Open a free ThingSpeak account at https://thingspeak.com/ Click the **Get Started For Free** button.
- Create and configure your ThingSpeak channel according to the instructions at https://w4krl.com/iot-kits/open-a-thingspeak-account/
- Record the API Write key and channel number.
3. Obtain your station information:
- APRS passcode.
- Latitude and longitude in decimal degrees. This is for accurate location of your station on the APRS network.
- Station elevation above sea level in meters. This ensures correct translation of your station barometric pressure to sea level pressure.
4. Install and configure the Arduino IDE:
- Download the version for your computer operating system at https://www.arduino.cc/ Do not use the web version or the Windows app.
- Install the Arduino core for the ESP8266 at https://github.com/esp8266/Arduino Follow the instructions under the heading **Installing with Boards Manager**
5. Add needed libraries to the Arduino IDE using menu item *Sketch>Include Library>Manage Libraries...* Search for and install each of the following libaries. Be aware that there may be several libraries with similar names. Check that you are installing the library by the author noted:
  - **WiFiManager** by tzapu
  - **ArduinoJson** by Benoît Blanchon
  - **BH1750** by Christopher Laws
  - **Adafruit_BME280** by Adafruit
  - **DoubleResetDetector** by Stephen Denne
6. Download the firmware from this Github repository:
- Click on the **Code** button and select **Download ZIP**. The file will download as **D1M-WX1-APRS-ThingSpeak-OTA-WFM-main.zip**. 
- Unzip the file. The unzipped folder will contain several files and a folder **D1M-WX1-APRS-ThingSpeak-OTA-WFM**. Copy this folder to your Arduino sketchbook folder.
  - If you need to find your sketchbook folder, open the Arduino IDE and use menu **File>Preferences**. The first line tells you where the sketchbook resides on your computer.
- Use menu item *File>Sketchbook* to open **D1M-WX1-APRS-ThingSpeak-OTA-WFM**
- Set *Tools>Board>ESP8266 Boards* to **LOLIN(WEMOS) D1 R2 & mini**
- Do a test compile from *Sketch>Verify/Compile* or type Control+R. Correct any problems.
7. Upload the firmware:
  - Set the PROG/RUN switch to PROG on the weather station. 
  - Set the power switch to OFF or unplug the LiPo cell to prevent back feed from the D1 Mini to the cell. 
  - Connect your PC to the D1 Mini with a Micro USB cable.
  - Open the serial monitor and set the baud rate to 115200.
  - Compile and upload the firmware.
  - Note status messages from the D1 Mini.

## Software Configuration
These instructions are for a cell phone with Wi-Fi capability. The same method could be used with a Wi-Fi connected computer. 
You must be familiar with how to open the Wi-Fi connection settings on your phone, how to open your phone’s web browser, and how to enter an URL into the browser’s address bar.
This procedure will temporarily turn the weather station into a Wi-Fi access point. You will connect your cell phone to the weather station's Wi-Fi signal. Then use your cell phone’s Internet browser to open a web page hosted by the weather station. After you enter the configuration information into the web page, the weather station will connect to your Wi-Fi network and begin normal operation.  

1.	Have all the configuration information at hand ready to enter:  
    *	Your Wi-Fi name (SSID) and password.
    *	Your APRS callsign with SSID. The recommended SSID is 13.
    *	Your APRS passcode
    *	Your latitude and longitude in decimal degrees
    *	Your elevation in meters
    *	Your ThingSpeak channel ID and API Write key
2.	Connect your PC to the D1 Mini with a Micro USB cable.
    * Turn off the power switch or unplug the LiPo cell to prevent backfeed.
    * Set the PROG/RUN switch to RUN
    * Open the IDE serial monitor
3. Press the *RESET* button on the D1 Mini twice to start the WiFiManager. This is indicated by the blue LED being illuminated. You may have to do this a few times to get it to recognize the double resets.
4.	Open your phone’s Wi-Fi connection settings. After a brief delay *D1M-WX1-portal* will appear in the list of available networks. Select it and wait for your phone to connect. You may safely ignore any warning that no Internet is available.
5.	Open your phone’s browser. Enter **192.168.4.1** into the address bar. A web page will appear.
6.	Enter the following information:  
    * Your Wi-Fi SSID (you may select this from the available networks shown at the top of the page)
    *	Your Wi-Fi password
    *	Your callsign-SSID. 
    * Your latitude in decimal degrees. Positive for north latitudes, negative for south latitudes.
    *	Your longitude in decimal degrees. Positive for east longitudes, negative for west longitudes.
    *	Your elevation in meters
    *	Your ThingSpeak channel ID and API Write key
    *	Hit **Save**
6. See an example serial output in the photo.
7.	**If you ever need to change any parameter, press the *RESET* button on the D1 Mini twice in quick succession no more than 10 seconds apart. You may have to do this a few times. The unit goes into configuration mode when the blue LED on the D1 Mini turns on.**

| Click to enlarge |   |   |   |   |
|:----:|:----:|:----:|:----:|:----:|
|<img src="/images/AvailableNetworks.jpg" width="40%"/>|<img src="/images/CaptivePortalLogin.jpg" width="40%"/> | <img src="/images/CaptivePortalTopjpg.jpg" width="40%"/> | <img src="/images/CaptivePortalLower.jpg" width="40%"/>| <img src="/images/SerialOutput.png" width="40%"/>| 
|Find *D1M-WX1-portal* in Available Networks|Open *192.168.4.1* in cell phone browser | Upper part of portal| Lower part of portal | Serial Output |
