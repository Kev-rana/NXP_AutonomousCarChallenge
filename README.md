# NXP_AutonomousCarChallenge-TEAM LEGO
This was a code I and my colleague developed for the NXP Autonomous cup competetion '22. 
The Implementation is on ROS2 Foxy , and the CNN model was trained on Google Collab

Round 1:
The ROS framework is used to facilitate communications between the different nodes(sensors and actuators) in an autonomous car. In this project, a simulated Pixy camera sensor was used to read the images and send to the ROS framework for processing. The Open CV library works on the subscribed images of the road and defines two(or one) vector(s) that are the 2 lanes of the road. These two vectors are stored and published via ROS messages to the subscriber node/script that performs the lane keeping function. The variables that are adjusted by the line follower node inorder to navigate the car are the linear velocity and angular/ steer velocity.
The goal is: - Smooth lane keeping without oscillations. - To not touch the sides of the road. - Minimize jerking/ bouncing of the car.
The Aim line follow algorithm: Initialize the node as subscribe to the Pixy Vector message topic.
- Define the: linear velocity,- Angular velocity. - Single line steer scale.
- Create a publisher to the command velocity topic that goes to the actuators. - The listener callback function is called by the Pixy Vector message subscriber and the number of vectors received is checked.
- Case 1: Number of vectors = 0 - Stay in the previous state as long as the later conditions are encountered.
- Case 2: Number of vectors = 1 - From the slope of the vector determine whether the lane is on the left or the light. - Whenever only one vector is detected, the steer speed is increased to make the car move towards the frame centre so that both the lines are visible so that the car can drive properly by having proper visibility.
- The steer speed is calculated based on the steepness of the vector -the steeper the vector’s slope, the more the steer speed. Appropriate factors have been multiplied to the resulting developed equation based on observations made during the simulation to ensure smooth turns and transitions.
- The linear velocity is also curbed by appropriate negation factor of the steer speed. To resolve the linear velocity going negative in the calculations, we used the absolute function to always make sure the linear velocity is positive.
- Case 3: Number of vectors = 2 - The average of the front x coordinates of the 2 vectors is compared against the windows x centre value

In the event of the model deviating from the centre frame, it is steered in the direction towards the centre of the frame based on the amount of offset of the car from the desired location, that is, center of the track.
To navigate the jerking at the overpass: - We observed that while climbing , the distances between the lanes, as observed by the camera, increases and on reaching the top, the lanes appear close to each other.
- We used this observation to increase the speed of the car to facilitate climbing and decrease the speed while descending to minimize bouncing off the road.
- Finally, the linear and angular velocities are published as command velocities for the actuator nodes.
Changes made: - Aim_line_follow.py file was modified to make adjustments to the algorithm. - Nxp_cupcar.sdf.jinja file was modified to include our .dae models that replaced the base models, and to change the color of the wheels.
- nxp_gazebo/models/nxp_cupcar/meshes folder was added with the custom files( finaltest7.dae and pallete.png) file to replace the CUPCarCollisionBody.stl file.
