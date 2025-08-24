![Greenbotics 2025](other/readmephotos/logo.png)

# This Incredible Repository

Welcome to the Greenbotics 2025 repository! This repository contains everything we've worked on over the past five months, including all our code and resources. You can also use this repository as a complete guide to understand and reconstruct the robot from the ground up.

# Our Team

<img src="other/readmephotos/Devansh.jpg" width="150" align="left" style="margin-right:20px;"/>

**Devansh**  
Hi! I’m Devansh doing my second WRO season. Last year I represented India in the RoboMission category coming #25 in Internationals. This year I wanted a new challenge and learn new things. My hobbies are coding and robotics.
<br clear="left"/>

---

<img src="other/readmephotos/Sheel.jpg" width="150" align="left" style="margin-right:20px;"/>

**Sheel**
Hi! I'm Sheel. I'm 15, and doing my second WRO season. Last year, our team made it to the International Finals in Robomission Junior. Additionally, I'm learning competetive debate. I also have a dog, and both of us are obsessed with Monkeys. 
<br clear="left"/>

---

<img src="other/readmephotos/Rakshith.png" width="150" align="left" style="margin-right:20px;"/>

**Rakshith**  
I'm Rakshith, a 9th grader passionate about robotics and aspiring to study engineering in Germany. I love building drones, experimenting with sensors. I also have 3 dogs, and two of them live with my grandparents in Hyderabad. 

<br clear="left"/>

---

<img src="other/readmephotos/PareshGambhava.jpg" width="150" align="left" style="margin-right:20px;"/>

**Paresh Gambhava**  
Mr.Paresh Gambhava is our chief coach from The Robotronics Club Ahmedabad. He is a robotics enthusiast, and electrical engineer by profession. He has vast experience training students for Robotics competition and projects. 

<br clear="left"/>

# The Challenge

The [Future Engineer Challenge](hhttps://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf "Vew the season rulebook") for the 2025 WRO Season involves building an autonomous vehicle that can complete two challenges. The first challenge is the Open Challenge. This challenge involves the robot completing three full laps on the field. The field consists of outer boundary walls, and 4 inner walls that are randomly placed to form a closed rectangle.

The second challenge is the Obstacle Challenge. In the Obstacle Challenge red or green cuboids called Traffic Signs are placed along the course. The robot must complete three laps around the track white making sure to pass the red traffic signs from the left, and the green traffic signs from the right. The Obstacle Challenge also has a dedicated Parking Space. The robot must start and end in the parking space to attain full points. The walls placement is fixed for the Obstacle Challenge.


# Photos of our robot



Having spend a lot of time working on Lego based hardware in the 2024 season, we chose to use Lego for our Robot's Hardware. However the robot also uses a Raspberry Pi, Raspberry Pi camera, and other off the shelf electronic components.

# A video of our robot on [Youtube]()


# Obastacle Management

## Open Challenge

For the Open Challenge, our robot uses the two distance sensors attached to the left and right sides of the robot. As the robot already starts in a straightforward section, it can directly enter the main movement loop. 

The robot starts by measuring the distance between itself and the walls. If the readings from any of the sensors are 7cm or less, the robot must first move itself away from the walls to the center of the track.

The robot starts moving straight using readings from the attached IMU, while the left and right sensors start measuring the distance between the robot and the 2 sets of walls. The bot will continue moving forward until one of its sensors returns a NONE reading. Our distance sensors cannot measure distances greater than 2m, and will return NONE if such a situation arises. On the WRO course, this can only occur when the inner wall ends at a corner, leaving open space till the wall on the opposite side of the track. When this occurs the robot knows to take a turn. If the right sensor returns a NULL reading, the robot will turn right, and vice versa. 
This scenario will repeat 12 times for each of the 12 turns a robot must take as it completes 3 entire loops with 4 turns each. On completing its 12th turn, the robot will continue moving for 1 to 2 seconds to end in the same straightforward section it started in.

## Obstacle Challenge

For the obstacle challenge, as our robot starts within the parking area, it must first maneuver its way out. However, before it does so, it also judges the driving direction. If the robot is placed within the parking area with its right sensor closer to the outer wall, the robot will know to move anticlockwise, and vice versa.

The robot will proceed to take a 90 degree turn to maneuver out of the parking area. However, when 2/3rds of the turn has been completed, and the robot is at 60 degrees relative to where it started, it will scan to check if a block is present. If a block is present, the robot will complete its 90 degree turn and move forwards or backwards to better orient itself.

If the driving direction is anticlockwise and a green block is detected, the robot will move forward, and then complete its turn. If the block is red, the robot will move backward before completing the turn. 

If the driving direction is clockwise and a green block is detected, the robot will move backward before completing the turn. If the block is red, the robot will move forward, and then complete the turn.

The robot will move straight using the IMU, and the distance sensors for ‘wall following’, 
While moving in the straightforward section, the robot will be consistently scanning for blocks. When a block is detected, the robot will take a 45 degree turn in the stipulated turning direction (right for red and left for green). When the block has been crossed, the robot will take a 45 degree turn in the opposite direction to return to the center of the 2 walls. The robot also keeps a count of how many cubes it has scanned. If it has scanned 0 (such as when it enters a straightforward section), it will expect a further 1 or 2 cubes. If 2 cubes have been scanned, it will not look for any more cubes until it has detected a wall up ahead and turned.

The robot will take 12 turns, before moving forward a few seconds to end up in the same straightforward section as where the parking area is located 

# Mobility Management

## The Powertrain

### Our Motor

Keeping with the idea of a Lego-Oriented design, we chose to use a standard LEGO EV3 Medium Motor. While this motor easily integrates with the chassis and axle, it doesn’t directly connect to our motor driver. Instead, we identified the pinout of the LEGO motor, and directly connected the wires to the motor driver. The pinout diagram is shown below

<img src="other/readmephotos/motordiagram.jpeg" alt="Motor Pinout Diagram" width="1500"/>


---
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/MediumMotor.jpeg" alt="EV3 Medium Motor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: LEGO EV3 Motor</li>
      <li>Voltage: 9V</li>
      <li>No Load Speed: 240 to 250 RPM</li>
      <li>Weight: 40g</li>
      <li>Encoder resolution: 1°</li>
    </td>
  </tr>
</table>

---

Potential improvements: 

- Use a more powerful non-lego motor such as a 12V DC motor that is  

    -Compact and lightweight

    -More powerful
    
    -Cheaper

- Use the built-in encoder for more accurate movement and turns of the robot. Using the encoder allows for movement inputs to be given in centimeters or degrees, rather than in time, helping in more complex tasks like parking.


### Our Drivetrain

Our EV3 motor is connected to a LEGO axle that drives a gear connected to a LEGO differential, which splits torque between two wheels. The differential lets both wheels rotate at different speeds when turning, thereby reducing the friction at each wheel, and making turns smoother.

Potential Improvements

- Use a different type of differential gear to avoid the wheels slipping, especially on the smooth surface of the mat.


### The Motor Driver

To control the current received by the motor and to control the speed of the drive motor, we utilized a SparkFun Dual TB6612FNG motor driver. The motor driver is connected to our PCB, and receives signals from the Raspberry Pi.

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/TB6612FNG.jpeg" alt="Motor Driver" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: TB6612FNG</li>
      <li>Power Supply Voltage: 15V (Max)</li>
      <li>Average Output current: 1.2A</li>
      <li>Peak Output current: 3.2A</li>
      <li>Standby control to save power</li>
      <li>Built-in thermal shutdown circuit</li>
    </td>
  </tr>
</table>

Potential Improvements:

- Use a motor driver that can output higher currents, to allow for the use of higher performance motors

## The Steering System

For our steering mechanism, we needed a design that was precise. Like all other aspects of our robot’s design, our first choice was to use a LEGO based mechanism. To do that we could have used any encoder based motor (such as a spike prime small motor) as a makeshift servo motor. 

The problem was, that while any LEGO motor would have been both highly precise and easy to integrate, even the smallest motor was too bulky and required separate wires. 

Therefore, we decided to use an off the shelf servo motor. To integrate it into our LEGO based design, we 3D printed a custom servo mount which could be attached with lego screws to the front of the chassis directly behind the front wheel, while simultaneously hot gluing the servo horn to the axle of the front wheels.

### The Servo Motor

Our steering system is centered around the SG90 servo motor, which provides precise angular control, making it highly suitable for steering applications where accuracy is essential. Unlike standard DC motors, the SG90 has a built-in feedback mechanism that allows it to hold a commanded angle reliably without additional sensors. Despite its compact size, it delivers sufficient torque to handle the demands of turning the steering mechanism, ensuring stability.

To connect the servo motor to our robot, we designed and 3D printed a custom Servo Mount. The diagram is shown below.

---
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/servo.jpeg" alt="Servo Motor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: SG90</li>
      <li>Operating voltage: 3.0V - 7.2V</li>
      <li>Weight: 9 gm</li>
    </td>
  </tr>
</table>

A diagram of our servo mount is schown below: 

<img src="other/readmephotos/ServoMount.jpeg" alt="Servo Mount" width="300"/>


Potential Improvements:

- 3D print the servo horn to directly connect to the LEGO Chassis. 
- Try out different steering mechanisms such as the Ackermann system


## Our Chasis

When starting this project, we realized that the bulk of our effort must go into designing the autonomous driving used by this robot. As we wanted to spend more time on software, and having spent a considerable amount of time using LEGO hardware for our 2024 Robomission campaign, we decided to make a LEGO chassis. 

The LEGO chassis has many benefits, the primary one being ease of integration. Lego parts can easily be snapped on and off, making chassis development and chassis far easier and less time consuming. Making changes to a non-lego chassis would have meant that each time we wanted to make a change to our chassis, we would have to make new 3D prints, increasing time, cost, waste and materials used.

Our chassis consists of the components that make up the drivetrain and steering system. The current chassis supports our EV3 motor, differential gear, back wheels and axel, front wheels and axel, servo motor and servo mount.

# Power and Sense Management

Our robot recieves inputs from off-the-shelf sensors that have been mounted on the robot. Additionally, it also uses a camera for the obstacle challenge. Inputs are processed by our Raspberry Pi. This entire assembly is powered by a Li-Po battery

## Our LiPo battery 

Our robot uses the Bonka 11.1V 2200mAh 35C 3S Lithium Polymer (LiPo) battery. This LiPO battery is often used for drones and quadcopters. We picked this battery, as one of our team members had it readily available, and had used it in other projects beforehand. He found the battery reliable and hence we decided to use it. 

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/LiPobattery.jpeg" alt="LiPo Battery" width="80%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Maximum Voltage: 12.6V </li>
      <li>Nominal Voltage: 11.1V </li>
      <li>Capacity: 2200mAh</li>
      <li>Discharge Current: 35C</li>
      <li>Weight: 175g</li>
    </td>
  </tr>
</table>

---
Potential Improvements

- Implement a battery level indicator
- Use a Lithium-Ion battery, as it 

    - Is safer

    - Has Charge Cycles

    - Allows for a smaller robot
---
## Our Raspberry Pi

The heart of our robot is the Raspberry Pi 5 microcomputer. It processes inputs from our distance sensors, runs image processing algorithms on the live camera feed and controls the motors to drive the robot to achieve its desired goals.

We chose this particular model as it seamlessly integrates with a camera, making it easier to use than similar computers or microcontrollers such as an Arduino or ESP32. Additionally, we also wanted to take advantage of the Raspberry Pi 5 being far more powerful than other processors, allowing us to use newer and improved image processing technologies that deliver more accurate results but may require greater computing power.


Potential Improvements

- Use Arduinos and compatible cameras to make this robot much cheaper to design


<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/raspberrypi.png" alt="Raspberry Pi 5" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: Raspberry Pi 5 </li>
      <li>RAM: 8GB</li>
      <li>Pins: 40</li>
      <li>Input Voltage: 5V</li>
    </td>
  </tr>
</table>

## Our Voltage Convertor

Our battery provides 12V, but our Raspberry Pi requires 5V and our motor requires 9V. To supply these lower voltages safely, we used a XY-3606 step-down (buck) voltage converter.

Potential Improvements

- Use a linear convertor to reduce costs
- Use a buck converter that is more efficient and provides more power at different voltages. This ensures that all components running at all voltages can be used


<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/VoltageConvertor.jpg" alt="Voltage Convertor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: XY-3606 </li>
      <li>Working voltage: DC 9V–36V</li>
      <li>Output voltage: 5.2V</li>
    </td>
  </tr>
</table>

## Our Printed Circuit Board (PCB)

In our first draft of the robot, there were a lot of criss-crossing wires between many different components. There was no clear arrangement for the wires, making the robot very messy and confusing to change. During practice runs, wires would routinely get loose, making it very difficult to troubleshoot errors. 

To solve this, we designed a PCB which cleanly connected all of our components together, making the robot far cleaner and reliable.

Here is a comparison showing our robot before and after the PCB

Before:

<img src="other/readmephotos/PrePCB.jpg" alt="Robot Before PCB" width="200"/>

After

<img src="other/readmephotos/PostPCB.jpeg" alt="Robot After PCB" width="200"/>

An image and printable file for the PCB are provided below

<img src="other/readmephotos/PCB.jpeg" alt="PCB" width="200"/>


## Our Camera

For image detection, we are using the Raspberry Pi Camera Module 3. Apart from its ease of integration with the raspberry pi reducing a major complexity for us, the Camera module 3 has autofocus, wide-angle and low-light performance, all of which are very useful in a competition setting.

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/RPI Camera 3.jpeg" alt="RPI Camera" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Name: Camera Module 3 </li>
      <li>Resolution: 11.9 megapixels</li>
      <li>Horizontal/vertical: 4608 × 2592 pixels</li>
      <li>Autofocus, Wide-angle and Low-light performance</li>
    </td>
  </tr>
</table>

Potential Improvements

- Use the Raspberry Pi AI camera to proccesing data on the camera itself. This will reduce proccesing load on the Raspberry Pi

## Our IMU

Our robot uses the BNO055 IMU (Inertial Measurement Unit) to help it maintain direction when moving straight, and to make turns more accurate. The BNO055 combines data from an accelerometer, gyroscope, and magnetometer to provide absolute orientation, which reduces drift and allows the robot to correct its heading in real time.

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/IMU.jpeg" alt="IMU" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>I2C Addess: 0x28 </li>
      <li>Gyroscope: Range ±125°/s to ±2000°/s</li>
      <li>Accelerometer: Range ±2g, ±4g, ±8g, ±16g</li>
    </td>
  </tr>
</table>

Potential Improvements

- Move the IMU away from areas of magnetic interference
- Use higher quality IMUs such as the BNO085

## Our Distance Sensor

Apart from the camera, our robot also uses three VL53L1X distance sensors to gather inputs. The sensors are mounted on the front, back, left, and right of the robot respectively. The front sensor is used to avoid hitting the wall, while the back sensor is used in (((()))). The left and right sensors are used to detect when to take a turn. 

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readmephotos/DistanceSensor.png" alt="Distance Sensor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Operating Voltage: 3V / 5V </li>
      <li>Range: 4m</li>
      <li>I2C Address: 0x29</li>
    </td>
  </tr>
</table>


Potential Improvements

- Use sensors like VL53L8CP for better, more accurate detection.
- Use a lidar sensor

## The Structure of This Repository

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
