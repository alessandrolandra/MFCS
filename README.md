<div id="top"></div>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">

<h3 align="center">MFCS21</h3>

  <p align="center">
	Moth Flight Control System
	<br />
    Electronic flight control system for the moth class sailboat to be mounted on a skiff developed by Polito Sailing Team
    <br />
    <a href="https://github.com/alessandrolandra/PST_FlightControl"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/alessandrolandra/PST_FlightControl/issues">Report Bug</a>
    ·
    <a href="https://github.com/alessandrolandra/PST_FlightControl/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Project main image](images/model/model_frontview.jpg)](https://github.com/alessandrolandra/PST_FlightControl)

<p align="right">(<a href="#top">back to top</a>)</p>



### Built With

* [STM32F4xx](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html)
* [Arduino](https://www.arduino.cc/)

<p align="right">(<a href="#top">back to top</a>)</p>

The board we used is the STM32F411RE which offers the support for the Arduino Uno, and also for Arduino IDE.
The decision to keep using Arduino Libraries is because they are very well documented, easy to use and compatible for both the Arduino Uno and our STM32 board.
Here is the list of all libraries used for this project and the relative documentation: <br>
* [Adafruit BNO08x IMU](https://github.com/adafruit/Adafruit_BNO08x) **Adafruit library**
* [MFRC522 RFID](https://www.arduino.cc/reference/en/libraries/mfrc522/) **Arduino library**
* [PID](https://playground.arduino.cc/Code/PIDLibrary/) **Arduino library**
* [IWatchdog](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog) **ST-Microelectronics library**
* [Servo](https://www.arduino.cc/reference/en/libraries/servo/) **Arduino library**
<br>
Talking about ultrasonic sensors, we used HC-SR04 drivers, that allows us to trigger and read the sensors through 2 digital signals.<br>
In particular, as you can see in our <a href="src/stm32INOsketch/stm32INOsketch.ino">code</a>, we need to set as high the trigger for 10us, after keeping it low for 2us, to make the sensor emit the sound wave.<br>
The echo pin will immediately go up and will assume the low value when the wave will be received back.
For all the other sensors and actuators, no additional driver has been used; the RFID reader is already communicating through SPI, the servomotor is driven by a PWM and the buzzer too. IMU uses I2C communication protocol.

### Functional Specification

This project aims at automatically regulate and control the height of the flight of a <a href="#moth">Moth</a>.<br>
Commercial products use mechanical elements to change the angle of the flap mounted below the drift, while the one on the rudder is directly regulated by the sailor. Our goal is to avoid the mechanical "measurement" of the height, using both ultrasonic sensors and an <a href="#imu">IMU</a>, and automatically derive the right angle the foil must assume according to that.<br>
In particular, our sensing system is composed of 3 ultrasonic sensors mounted one at the bow, and the other two at stern, to the left and right extremes of the skiff, and the IMU, that allows us to retrieve the spatial orientation of the boat. Taking into account the pitch and roll angles, we can properly weight the measured distances and find the height of the center of gravity of the boat.<br>
First of all some offsets have been added to the ultrasonic measures to consider the boat as a flat surface on which the sensor are mounted, independently of the real position of the sensors on the z axis.<br>
Then, the following offsets have been taken into account to retrieve the impact of the orientation of the boat on each measured data:

![Offset explanation](images/moth/boatScheme.jpg)

The output of the system is the angle the flap must assume in order to reach and maintain the TARGET height.<br>
This has been obtained using two different PIDs, a more aggressive one, to quickly reach the target, and a conservative one, to smootly maintain the height.
The change between the two depends on the absolute difference from the target height: if this is above a defined THRESHOLD, we are using the first, otherwise we are using the second.<br>
TARGET and THRESHOLD are then selected by the sailor between 3 different options for each parameter, just by approaching one of the two RFID cards to the reader.

<hr>
<div id="moth">The Moth is a small sailing boat designed to plane; since 2000 it has become an expensive and largely commercially-produced class of boat designed to hydroplane on foils.</div>
<p align="middle">
  <img src="images/moth/frontView.jpeg" width="45%" />
  <img src="images/moth/drift.jpg" width="45%" />
</p>
<div id="imu">IMU stands for Inertial Measurement Unit, and is an electronic device that measures and reports a body's specific force, angular rate and orientation of the body, using a combination of accelerometers, gyroscopes, and sometimes magnetometers.</div>
<p align="right">(Wikipedia)</p>

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

*	STM32F4xx microcontroller
*	3 HC-SR04 waterproof ultrasonic sensors
*	BNO085 IMU
*	MFRC522 RFID reader
*	servomotor (torque to be calculated according to flap and skiff dimensions/profile)
<br>
Follow the schematic below to setup the system:
<br> 

![schematic](images/schematic.jpg)
#### Pin assignment
IMU
<table>
	<th>Board</th><th>Sensor</th>
	<tr><td>PB8(SCL)</td><td>SCL</td></tr>
	<tr><td>PB9(SDA)</td><td>SDA</td></tr>
</table>
<br>
HC-SR04
<br>
<table>
	<th>Board</th><th>Sensor</th>
	<tr><td>PA0,PA1,PA4</td><td>Trig</td></tr>
	<tr><td>PB3,PB4,PB5</td><td>Echo</td></tr>
</table>
<br>
RFID
<br>
<table>
	<th>Board</th><th>Sensor</th>
	<tr><td>PA8</td><td>SS(SDA)</td></tr>
	<tr><td>PA9</td><td>RST</td></tr>
	<tr><td>PB13</td><td>SCK</td></tr>
	<tr><td>PB14</td><td>MISO</td></tr>
	<tr><td>PB15</td><td>MOSI</td></tr>
</table>
<br>
Buzzer
<br>
<table>
	<th>Board</th><th>Sensor</th>
	<tr><td>PC7</td><td>+</td></tr>
</table>
<br>
Servomotor
<br>
<table>
	<th>Board</th><th>Sensor</th>
	<tr><td>PB10</td><td>signal</td></tr>
</table>
<br>
All the 3v3,5v and GND connections are not showed in the picture but have to be properly managed.
<br>
<br>

![CU](images/CU.jpg)

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/alessandrolandra/PST_FlightControl.git
   ```
2. Copy all the libraries contained in <a href="libraries">libraries</a> in your respective Arduino folder
3. Compile the <a href="src/stm32INOsketch/stm32INOsketch.ino">stm32INOsketch</a> and upload it on the board

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Power on the device, calibrate the IMU, by performing the usual 8 in air around all the 3 different axes (a buzzer sound will let you know when the calibration is done) and mount everything in place.
Use the wanted RFID card to change either the target height or the sensibility of the system (when to perform the switch between the two PID networks); click <a href="videos/simulation_test.mp4">here</a> for a quick demo.
An example of how the system works and ties to maintain a certain target height is provided <a href="videos/parameter_change.mp4">here</a>.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- Physical system and model building -->
### Physical system and model building

Below you can find some images documenting the different phases of work followed to build the entire system.

![components](images/model/model_components.jpg)
![drift](images/model/model_drift.jpg)
![3dSupport](images/model/3dSupport.jpeg)
![dutycycle](images/behind_the_scenes/working_on_dutycycle.jpg)
![soldering1](images/behind_the_scenes/working_on_soldering_1.jpg)
![soldering2](images/behind_the_scenes/working_on_soldering_2.jpg)

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [LICENSE](LICENSE.txt) for more information.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Luca Dalmasso, Omar Gai, Alessandro Landra

Project Link: [https://github.com/alessandrolandra/PST_FlightControl](https://github.com/alessandrolandra/PST_FlightControl)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [PolitoSailingTeam](https://areeweb.polito.it/politosailingteam/)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/badge/contributors-link-red
[contributors-url]: https://github.com/alessandrolandra/PST_FlightControl/graphs/contributors
[forks-shield]: https://img.shields.io/badge/forks-link-blue
[forks-url]: https://github.com/alessandrolandra/PST_FlightControl/network/members
[stars-shield]: https://img.shields.io/badge/stars-link-yellow
[stars-url]: https://github.com/alessandrolandra/PST_FlightControl/stargazers
[issues-shield]: https://img.shields.io/badge/issues-link-orange
[issues-url]: https://github.com/alessandrolandra/PST_FlightControl/issues
[license-shield]: https://img.shields.io/badge/license-link-green
[license-url]:https://github.com/alessandrolandra/PST_FlightControl/blob/main/LICENSE.txt
