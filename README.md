# IOT Final Project iNetworkAnalyzer

iNetworkAnalyzer (a.k.a iNA) is a tool for taking basic internet performance measurements over cellular, such as latency, upload bitrate and  download bitrate.

iNA workflow:
------------
1. Initializaiton of GPS and Cellular modules.

2. Welcoming user with application instructions.
  BTN 0 - Exit the application and turn off the MCU and modules.
  BTN 1 - Test internet performance measurements.
  
3. Pre-testing setup:
 3.1 Pull GPS data and save current device location.
 3.2 Set internet connection profile (apn).
 3.3 Set HTTP and socket service profiles.
 3.4 Find available cellular operators networks (Partner, Cellcom and Pelephone).
  
4. Testing performance:
   Iterating over found cellular operators, register each one and verify signal quality.
 4.1 Test upload & download bitrates:
     Upload:
     Take start time
     Send 10 packets (each 1500 Bytes) (we chose 10, for the purpose of quick demo)
     When the server will receive the 10x1500 bytes it will send a special ACK packet ("UPLOAD_FINISHED")
     Take finish time
     
     Download:
     Take start time
     Receive 10 packets (each 1500 Bytes) (we chose 10, for the purpose of quick demo)
     Take finish time
     
     Calculate bitrate - bits per second.
 4.2 Test latency by using AT^SISX ping command:
     Send 30 ICMPs and extract the meanRTT by parsing the statistics results.
     return meanRTT / 2
     
5. Transmit results over HTTP to the InfluxDB

6. Print results on screen

Server side:
-----------
We have implemented a server side as C++ application.
The server listening to welcome socket and accepting clients, and will manages the upload and download test accrodingly.

Grafana Monitoring panel:
-------------------------
![Grafna-Panels](https://github.com/Netanel-f/IoT-Final_Project/blob/master/grafana_screenshot.png?raw=true)
