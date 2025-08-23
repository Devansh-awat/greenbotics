![Greenbotics 2025](other/readmephotos/logo.png)

# This Incredible Repository

Welcome to the Greenbotics 2025 repository! This repository contains everything we've worked on over the past five months, including all our code and resources. You can also use this repository as a complete guide to understand and reconstruct the robot from the ground up.

# Our Team


## Devansh
![Devansh](other/readmephotos/Devansh.png)

Bio Placeholder

## Sheel
![Sheel](other/readmephotos/Sheel.png)

Hi! I'm Sheel. I'm 15, and this year, I'm doing my second WRO season. Last year, along with Devansh, our team made it to the International Finals in the Robomission Junior category. Alongside Robotics I'm learning competetive debate. I kinda suck though. I have a dog, and both of us are obsessed with Monkeys. 

## Rakshith
![Rakshith](other/readmephotos/Rakshith.png)

I'm Rakshith, a 9th grader passionate about robotics and aspiring to study engineering in Germany. I spend my time building drones, experimenting with sensors, and tackling tough math and physics problems. Beyond tech, I love table tennis, and write imaginative futuristic scripts.

I have 3 dogs - Riki, Cookie, and Cream. Cookie and Cream live with my grandparents in Hyderabad.

## Paresh Gambhava
![Paresh Gambhava](other/readmephotos/PareshGambhava.png)

Mr.Paresh Gambhava is our chief coach from The Robotronics Club Ahmedabad. He is a robotics enthusiast, engineer by profession, and teacher at heart. He has his degree in Electronics engineering and has been the chief coach at The Robotronics Club since 10 years. He has a deep experience in training students for different Robotics competition and projects. We are fortunate to learn from him the ethical practice, guidance and fundamentals.

------

# The Challenge

The [Future Engineer Challenge](hhttps://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf "Vew the season rulebook") for the 2025 WRO Season involves building an autonomous vehicle that can complete two challenges. The first challenge is the Open Challenge. This challenge involves the robot completing three full laps on the field. The field consists of outer boundary walls, and 4 inner walls that are randomly placed to form a closed rectangle.

The second challenge is the Obstacle Challenge. In the Obstacle Challenge red or green cuboids called Traffic Signs are placed along the course. The robot must complete three laps around the track white making sure to pass the red traffic signs from the left, and the green traffic signs from the right. The Obstacle Challenge also has a dedicated Parking Space. The robot must start and end in the parking space to attain full points. The walls placement is fixed for the Obstacle Challenge.


# Photos of our robot



Having spend a lot of time working on Lego based hardware in the 2024 season, we chose to use Lego for our Robot's Hardware. However the robot also uses a Raspberry Pi, Raspberry Pi camera, and other off the shelf electronic components.

# A video of our robot on [Youtube]()


# Obastacle Management**

## Open Challenge

For the Open Challenge, our robot uses the two distance sensors attached to the left and right sides of the robot. As the robot already starts in a straightforward section, it can directly enter the main movement loop. 

The robot starts by measuring the distance between itself and the walls. If the readings from any of the sensors are 7cm or less, the robot must first move itself away from the walls to the center of the track.

The robot starts moving straight using readings from the attached IMU, while the left and right sensors start measuring the distance between the robot and the 2 sets of walls. The bot will continue moving forward until one of its sensors returns a NONE reading. Our distance sensors cannot measure distances greater than 2m, and will return NONE if such a situation arises. On the WRO course, this can only occur when the inner wall ends at a corner, leaving open space till the wall on the opposite side of the track. When this occurs the robot knows to take a turn. If the right sensor returns a NULL reading, the robot will turn right, and vice versa. 
This scenario will repeat 12 times for each of the 12 turns a robot must take as it completes 3 entire loops with 4 turns each. On completing its 12th turn, the robot will continue moving for 1 to 2 seconds to end in the same straightforward section it started in.

# Mobility Management**

## The Powertrain

### Our Motor

### Our Drivetrain

### The Motor Driver

## The Steering System

For our steering mechanism, we needed a design that was precise. Like all other aspects of our robot’s design, our first choice was to use a LEGO based mechanism. To do that we could have used any encoder based motor (such as a spike prime small motor) as a makeshift servo motor. 

The problem was, that while any LEGO motor would have been both highly precise and easy to integrate, even the smallest motor was too bulky and required separate wires. 

Therefore, we decided to use an off the shelf servo motor. To integrate it into our LEGO based design, we 3D printed a custom servo mount which could be attached with lego screws to the front of the chassis directly behind the front wheel, while simultaneously hot gluing the servo horn to the axle of the front wheels.


Potential Improvements:

3D print the servo horn to directly connect to the LEGO Chassis. 
Try out different steering mechanisms such as the Ackermann system

### The Servo Motor




# Power and Sense Management**

Our robot recieves inputs from off-the-shelf sensors that have been mounted on the robot. Additionally, it also uses a camera for the obstacle challenge. Inputs are processed by our Raspberry Pi. This entire assembly is powered by a Li-Po battery



## The Structure of This Repository

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
