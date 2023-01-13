# Microprocessor-Lidar-System




<!-- GETTING STARTED -->
## Getting Started

Designed and constructed an embedded spatial measurement system using a time-of-flight sensor to gather information about the surrounding environment. The system includes a rotary mechanism that allows for a 360-degree measurement of distance within a single vertical geometric plane (e.g. y-z). The collected data is stored in onboard memory and later transmitted to a personal computer or web application for visualization and analysis.

### Description
 The microcontroller serves as a bridge between the time-of-flight sensor and the computer. The sensor is mounted on a stepper motor that rotates in both clockwise and counterclockwise directions to obtain spatial distance measurements in the y-z plane. It completes three full rotations, each representing a 360-degree surface that will later be visualized on a personal computer. The sensor uses a photon emitter to emit light and measures the time it takes for the light to bounce back after hitting a surface. The sensor includes all necessary analog-to-digital conversion such as transduction, signal conditioning and analog-to-digital conversion. The microcontroller communicates with the sensor using I2C protocol to get distance measurements and then communicates with the personal computer using UART. The data is then visualized using Python, specifically Pyserial to read data from UART and Open3D to create a 3D representation of the environment
This is an example of how to list things you need to use the software and how to install them.




### Built With

This section should list any major frameworks/libraries used to bootstrap your project. Leave any add-ons/plugins for the acknowledgements section. Here are a few examples.

Python: OpenGL,Pyserial
C: Extracting Distance Measurements, I2C Communication, UART Communication

SimpleLink MSP432E401Y Microcontroller
Stepper Motor, Time of Flight Sensor

Images
![alt text](https://github.com/rushi231/Microprocessor-Lidar-System/blob/main/Output.png)



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>





