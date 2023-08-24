# wall_follower_robot
This project is for the [Osoyoo robot car](https://osoyoo.com/2020/05/12/v2-1-robot-car-kit-for-arduino-tutorial-introduction/) which uses an Arduino board. 

The robot is capable of navigating a space and avoiding obstacles with the use of an ultrasonic sensor mounted on a servo. The robot periodically stops, scans its environment, and if it sees an obstacle in its path it will turn to avoid it. 

The wall-following feature uses a PID algorithm to maintain a constant distance from the wall. When following the wall the robot will turn its ultrasonic sensor to face the wall to continuously measure the distance to the wall. This, in combination with the PID algorithm, allows it to maintain a constant distance from the wall and react to sudden turns or corners in the wall.  

## To-do
- Add MPU6050 gyroscope and accelerometer to allow for better orientation, more precise turns and cornering. 
