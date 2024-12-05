# ScioSense ENS220 Drop detection demo
Firmware example to detect falls using the ENS220 pressure and temperature sensor.

## Working principle
The ENS220 can monitor falls by measuring the pressure changes. 
Two different factors are used to detect a fall: 
- A pressure increase. It indicates that the sensor is now at a lower position.
- An impact spike.

The spike in pressure upon impact is crucial for differenciating a drop from normal movement. This signal is caused by stress on the PCB due to the collision with the ground, and is therefore affected by the geometries of the PCB and housing. Adjustment of the impact strength parameter must therefore be adjusted once the final configuration was created to maximize the successful detections and avoid false positives.

A detailed explanation of the working principle can be found at the [ENS220 Application note - Fall Detection](https://www.sciosense.com/wp-content/uploads/2024/04/ENS220-Application-Note-Fall-Detection.pdf).
> [!IMPORTANT]
> Please note that the described algorithm is subject of a patent application. Users of the ENS220S pressure sensor are granted royalty-free rights to utilize the algorithm.

## Connections
The example was designed to work with a ESP32 and an ENS220 Evaluation kit PCB.

The firmware is expecting the following signals in these pins of the ESP32 DevKitC v4:
- I2C SDA: GPIO21
- I2C SCL: GPIO22
- INT: GPIO23

<img src="images/ens220_i2c+int_connections.png" width="1000">

## Output
Upon opening the Serial port at a 921600 baudrate the demo will output:
```c
Algorithm ready                                                                 
Starting ENS220 acquisition                                                     
ENS220 configured                                                               
T[C]:23.18                                                                      
Ready to detect drop events 
```
Then, after each drop is detected, the height of the drop will be output on the terminal:
```c
Drop 0.97m                                                                      
Drop 1.18m                                                                      
Drop 0.88m 
```
## Adjustment of the algorithm
Depending on the geometry of the application and the expected use, some parameters would need to be adjusted to increase the detection success. These parameters are expressed in meters for a more intuitive understanding by the user, which the firmware then transforms into a raw pressure value. 

The parameters that should be targeted first to fine-tune the algorithm are:
- **MIN_IMPACT_HEIGHT_M**: Minimum pressure spike to register a fall expressed in meters. A lower value should be used for smaller drop heights or more cushioned landings but it can lead to higher number of false positives.
- **MIN_HEIGHT_DIFF_M**: Minimum difference height in meters to register a fall. If the difference in height between the pre-drop and post-drop positions is lower than this value, the drop will not be registered.

## Debug

If the code is not detecting the drops in a satisfactory way for your application, or to explore the signals generated from drops and other events, the measured signals can be printed on the Serial port. For this, uncomment the line 
```c
//#define DEBUG_MEASUREMENTS
```
The program will now output on the Serial port not only the detected drops, but a continuous stream of the measured values in the following comma separated order:
- Time in milliseconds
- Raw pressure measurement. Divide by 64 to obtain the value in Pa
- Algorithm state. A "***1***" indicates that a possible drop event is being investigated
- Algorithm window index. Index of the data point in the moving window used for analyzing the data
- Drop event detected. A "***1***" indicates a successful detection

To transform the raw pressure ***i*** into a relative height in meters, the following formula gives a rough approximation by substracting it from the initial pressure and dividing by a conversion factor:
```math
Height_{meters}[i] = \frac{Pressure_{Raw}[0]-Pressure_{Raw}[i]}{768}
```
The firmware calculates a more accurate conversion factor taking into account the measured temperature and pressure, but this calculation gives a result within 20% over a wide range of atmospheric conditions.


