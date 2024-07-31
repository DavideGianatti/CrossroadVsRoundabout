November 2022

## Traffic Lights vs Roundabout
This project contains C++ scripts to simulate the traffic flows through intersections with traffic lights and roundabouts. 
The goal of the simulation is to compare the time wasted by traffic in both scenarios to determine whether it is more beneficial to keep traffic lights or to replace them with a roundabout. This comparison helps in making informed decisions about traffic management and infrastructure improvements.

### Project Structure
The project is organized as follow:
- **roundabout.h**: declaration of classes and methods usefull for the simulation of a roundabout;
- **roundabout.cpp**: implementation of roundabout.h;
- **trafficlights.h**: declarations of classes and methods usefull for the simulation of a crossroad with traffic-lights;
- **trafficlights.cpp**: implementation of trafficlights.h.

### Main Classes and Functionality

#### Car Class (car)
The **car** class represents a vehicle in the simulation. It includes the following main functionalities:
- Accelerate: Cars can increase their speed according to acceleration parameters.
- Turning Indicators: Cars can use turning indicators to signal lane changes or exits.
- Change Lanes: Cars can switch lanes if the side lane is clear.
- Enter and Exit: Cars can enter and exit the roundabout based on their and other cars' destination and current position.

#### Roundabout Class (round2)
The **round2** class models the roundabout and handles car movements within it. Key functionalities include:
- Car Arrival: Cars arrive at the roundabout based on a Poisson distribution that simulates random car arrivals at each intersection.
- Exit Probabilities: Probability distributions are used to determine the likelihood of cars exiting at each of the roundabout's exits.
- Roundabout Definition: Manages the roundabout's parameters such as the number of intersections, their positions, and the length of the roundabout.

#### Traffic Light Class (trafficlight)
The **trafficlight** class simulates traffic lights at an intersection. It includes:
- Timing: Defines the duration for which each traffic light remains green or red.
- Time Step: Updates the state of the traffic light over time, switching between green and red phases according to the specified timings.

#### Crossroad Class (crossroad)
The **crossroad** class represents an intersection controlled by traffic lights. It encompasses:
- Queue Management: Handles the queue of cars waiting at each traffic light and updates it based on car arrivals and departures.
- Traffic Light Scheduling: Manages the timing of traffic lights to ensure that cars can pass through the intersection efficiently.
- Time Wasted Calculation: Tracks and calculates the total time wasted by cars due to waiting at traffic lights.


