# ESP32_MedicalDevice
IoT devices to collect SPo2 and ECG for cancer treatment.

Downloading Repo from Github
---------------------------------------------------------------------------------------------------------------------------
- 1.) Navigate to "https://github.com/quocnh/OSA_Medical_Devices"
- 2.) Copy HTTPS link
- 3.) Open Git Bash in a local folder
- 4.) enter "git clone <Insert HTTP link here>"
---------------------------------------------------------------------------------------------------------------------------

Installing Firmware to ESP32
---------------------------------------------------------------------------------------------------------------------------
- 1.) Add ESP32 boards to Arduino IDE
	-File -> Preferences 
	-Enter "https://dl.espressif.com/dl/package_esp32_index.json" into "Additional Board Manager URLs" field
	-Tools -> Board -> Boards Manager...
	-Find "esp32" and install version 1.05
- 2.) Add necessary libraries to Arduino IDE
	-Sketch -> Include Library -> Add .ZIP library
	-Select .ZIP from "libraries" folder
	-Repeat for all libraries in the "libraries" folder

   	ArduinoHtppClient (0.5.0)
  	NTPClient(3.2.1)
  	AduinoJson(6.21.4)
  	AutoConnect(1.4.2)
  	PageBuilder(1.5.6)
  	PubSubCleint(2.8)
  	TPBubSupClient(2.9.1)
  	Thingsboard(0.10.2)
  
- 3.) Upload sketch to ESP32
	-Tools -> Board -> ESP32 dev module
	-Tools -> upload speed -> 115200
	-Tools -> Port -> select your COM port
	(https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) iff missing driver
	-upload (arrow symbol)
	-If connection doesn't work, close sketch, disconnect and reconnect USB, reopen sketch and upload
---------------------------------------------------------------------------------------------------------------------------

Opening Thingsboard Dashboard
---------------------------------------------------------------------------------------------------------------------------
- 1.) Open "http://134.129.97.131:8080/" and sign in
	-login: "tenant@thingsboard.org"
	-password: "tenant"
- 2.) Navigate to ESP32 dashboard
	-Dashboard groups -> ESP32 -> Devkit Test
---------------------------------------------------------------------------------------------------------------------------

WiFi Autoconnect
---------------------------------------------------------------------------------------------------------------------------
- 1.) Power on programmed device and connect to the SpO2ap/ECGap with the default password "12345678"
- 2.) Navigate to "192.168.10.101/_ac" with SpO2ap and "192.168.10.102/_ac" with ECGap on your mobile device's web browser
- 3.) Press settings icon
- 4.) Press menu icon
- 5.) Add wifi network credentials
- 6.) Device will now autoconnect to WiFi
---------------------------------------------------------------------------------------------------------------------------
![alt tag](https://github.com/quocnh/OSA/blob/main/Documentation/Screen%20Shot%202022-07-12%20at%2010.34.52%20PM.png)
![alt tag](https://github.com/quocnh/OSA/blob/main/Documentation/Screen%20Shot%202022-07-12%20at%2010.36.49%20PM.png)
