MSc Thesis N. Dijkshoorn

The small size of micro aerial vehicles (MAVs) allows a wide range of robotic applications, such as surveillance, inspection and search & rescue.
In order to operate autonomously, the robot requires the ability to known its position and movement in the environment.
Since no assumptions can be made about the environment, the robot has to learn from its environment.
Simultaneous Localization and Mapping (SLAM) using aerial vehicles is an active research area in robotics.
However, current approaches use algorithms that are computationally expensive and cannot be applied for realtime navigation problems.
Furthermore, most researchers rely on expensive aerial vehicles with advanced sensors.

This thesis presents a realtime SLAM approach for affordable MAVs with a down-looking camera.
Focusing on realtime methods and affordable MAVs increases the employability of aerial vehicles in  real world situations.
The approach has been validated with the AR.Drone quadrotor helicoptor, which was the standard platform for the International Micro Air Vehicle competition.
The development is partly based on simulation, which requires both a realistic sensor and motion model.
The AR.Drone simulation model is described and validated.

Furthermore, this thesis describes how a visual map of the environment can be made.
This visual map consists of a texture map and a feature map. The texture map is used for human navigation and the feature map is used by the AR.Drone to localize itself.
To do so, a localization method is presented.
It uses a novel approach to robustly recover the translation and rotation between a camera frame and the map.
An experimental method to create an elevation map with a single airborne ultrasound sensor is presented.
This elevation map is combined with the texture map and visualized in realtime.

Experiments have validated that the presented methods works in a variety of environments.
One of the experiments demonstrates how well the localization works for circumstances encountered during the IMAV competition.
Furthermore, the impact of the camera resolution and various pose recovery approaches are investigated.