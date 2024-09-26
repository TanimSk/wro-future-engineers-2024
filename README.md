# Engineering materials (Team Heisenberg)

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2024.

## Introduction

## How does it work?

Our project integrates five ultrasonic sensors, positioned at the front, left, right, front-left, and front-right along with an Rapberry Pi Zero with Camera for detecting green and red traffic towers. The Arduino UNO efficiently controls the steering via Ackermann steering system and a servo and adjusts the rear motor's speed for smooth forward or reverse motion. This system combines real-time sensor input and intelligent decision-making to ensure robust obstacle avoidance and accurate navigation.

### Machine learning:

We used an ESP32CAM to process images and detect traffic towers in real-time for our project.

![ESP32CAM](https://i.ibb.co.com/g3BGp0H/esp32.jpg)

The model was trained using Edge Impulse, which made it easy to build and deploy machine learning models on the ESP32CAM. Our dataset included a large collection of images of both green and red traffic towers, with 4,017 images for training and 1,006 for testing. After training, the model reached an accuracy of 97.61%. Here is the confusion matrix:

![Confusion Matrix](https://i.ibb.co.com/fNXP2Dn/confusion-matrix.png)

The model works by capturing an image with the ESP32CAM, identifying the type of traffic tower (green or red), and providing a bounding box around the detected tower. This data is then sent to an Arduino via Serial communication. The Arduino processes the information and controls servo motors to steer a vehicle in the correct direction based on the traffic tower in front of it.

In short, our system can automatically detect traffic towers and move a vehicle accordingly, combining image processing with real-time control, all powered by an efficient edge AI setup.

## Components Description

We are using the following components in our obstacle-avoiding vehicle:

- **Arduino UNO**: Acts as the central processing unit, handling all the logic and decision-making for the vehicle. It processes data from various sensors and components to determine the appropriate actions, such as steering adjustments or speed control.

- **ESP32-Cam**: This module is responsible for image recognition tasks, such as detecting traffic towers and other visual markers. It performs complex image processing and sends the results to the Arduino UNO via the UART bus, allowing the main controller to make informed decisions based on visual data.

- **L298N H Bridge Motor Driver**: Controls the rear wheels, managing both the direction and speed of the vehicle's movement. The motor driver allows the Arduino UNO to precisely adjust the vehicle’s speed for forward or reverse motion.

- **Servo**: Controls the vehicle's steering by adjusting the front wheels according to commands from the Arduino UNO. This servo ensures the vehicle navigates correctly, especially when avoiding obstacles or following the path.

- **HC-SR04 Ultrasonic Sensor**: This sensor measures the distance to nearby obstacles in different directions (front, left, right, etc.). The data is sent to the Arduino UNO, which uses it to calculate safe paths and avoid collisions.

- **Li-Po 3S 1500mAh Battery**: Provides power to all the components, ensuring a stable and long-lasting energy supply for both sensors and motors.

- **Buck Converter**: Provides 5 volts from the 11.1v Li-Po battery as we are using components that run on 5v.

- **Generic DC Motor**: Drives the rear wheels, providing the vehicle with the necessary power to move. The motor is controlled via the L298N driver, which adjusts the speed and direction as per the logic handled by the Arduino.

### 3D modeling and mechanisms:

For our project, we used Fusion 360 for designing the 3D model and printed the parts using an Ender 3 V2 3D printer. The vehicle was designed to be modular, meaning that the individual parts can easily be swapped out if they get damaged or if we need to modify the design. In total, the vehicle consists of 17 different parts.

To control the steering of the front wheels, we incorporated an Ackerman steering mechanism, which provides smoother turns. The steering is powered by a servo motor, ensuring precise control.

![Ackerman Steering](https://i.ibb.co.com/NVyh4wZ/07488c45bdede8bfbb35a9dfa589d4e5747080ad-2-645x500.png)

After designing the front module, we moved on to the base of the vehicle. This is where all the critical components like the battery, Arduino, motor controllers, and other hardware are housed. On top of the base, we included a 16cm long camera holder, which allows the ESP32CAM to have an elevated and clear view of the surroundings.

Finally, in the rear module, we designed a wheel controller that can rotate using a motor. The entire 3D body of the vehicle was printed using PLA filament, with the final weight of the printed parts coming in at 120 grams. This lightweight yet durable design keeps the vehicle agile and easy to control.

### Overall description:

Our project is a self-driving vehicle designed to detect obstacles and traffic towers in real-time, using a combination of sensors and AI. At the heart of the system is an Arduino UNO, which controls the vehicle based on input from five ultrasonic sensors and an ESP32CAM. The ultrasonic sensors help it avoid obstacles, while the ESP32CAM is trained to recognize red and green traffic towers using a machine learning model built with Edge Impulse.

The vehicle’s design is modular and was 3D-printed using Fusion 360 and an Ender 3 V2 printer. This allows us to easily swap out parts if they get damaged or need upgrades. The steering system uses an Ackerman mechanism controlled by a servo motor, which ensures smooth and precise turns. The rear wheels are powered by a DC motor, with the speed and direction managed by an L298N motor driver.

The lightweight design (just 120 grams for the 3D-printed body) keeps the vehicle quick and nimble. Power comes from a Li-Po 3S 1500mAh battery, which provides enough juice for extended runs. Overall, our vehicle combines smart decision-making, AI-based traffic tower detection, and reliable obstacle avoidance to navigate complex environments efficiently.
