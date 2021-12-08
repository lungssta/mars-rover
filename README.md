# mars-rover
Webots simulation of a robotic system, designed using CAD, composed of a rover that will set 
up a network of GPS beacons at high ground locations that have been roughly determined from satellite 
imagery. These GPS beacons will then facilitate precise navigation of a multirotor which will navigate 
to and hover at designated GPS locations to collect the air quality samples. 

Project focuses on the control of the rover in navigating the landscape and the design and operation of a manipulator to place the GPS beacons. 

• The task will be carried out within an open environment with rough terrain.
• The robot must operate autonomously.
• The robot will initially be placed in the centre of the map at a bearing of 0 degrees. 
• The GPS beacons will be cylinders with the following dimensions and properties:
o Radius: 0.25m
o Height: 0.35m
o Mass: 2kg
• The task will have three levels of difficulty to be completed:
▪ The terrain geometry is fixed (i.e. it will not be changed from the provided file)


** Webots Environment Description**
The task will be carried out in an environment which simulates an area on the surface of Mars. The task 
will be performed in a simulated 100x100m portion of the Martian landscape. There are 4 high ground 
points, with 1 in each quadrant.

This repository provides the Webots project folder which contains the world file with the final system (wheeled 
robot, sensors, manipulator etc.) and the latest version of controller code. 

PID control, C, mechatronics, Autodesk Inventor, Webots, Engineering 
