# toucheyFacey


## System using a Makey Makey and Kinect to allow one person to make a 3D map of the other's face through finger touches.

Project video: https://www.youtube.com/watch?v=bVH5vHlovxc

<b>Touchey Facey is an experience for two people which requires them to transgress usual social boundaries:</b>
One person touches the other’s face, repeatedly and deliberately, while a Kinect records the exact locations the touches occur in three-dimensional space. The result is a point cloud that serves as a map of an unusually intimate encounter.

The original impetus for this project was an inquiry into the possibility of using a robot arm as a probe for characterizing the shape of a person’s face or body. The project answers the next question that this inquiry led to: what would it look like to use your finger as a probe to describe another person’s body? Relying on a kind of touching that is not typically condoned between any except intimate partners, though not at all necessarily erotic, a Makey Makey sends a keystroke to a computer when one person’s finger completes a circuit with the other’s body. The touching finger has a colored marker so that a simple computer vision procedure can locate it; when the Makey Makey detects a touch, data from a Kinect is parsed to record the exact location the touch happened. In this way, a map of a face (or any other conductive surface) is built up.

The system operator has a simple control screen that allows her to actively calibrate the color, color sensitivity, and contour size that the vision system is using to identify the touching finger. Additionally, there are a few other tweaks available through keyboard commands. As they are generated, the data points are automatically added to a scroll/pan/zoomable point cloud, a projection of which is visible on the right half of the window.

Observing the patterns of face touches recorded with the system, certain interesting patterns emerge; these are, after all, maps of the places that people touched each other. They represent, in this sense, a kind of map of comfort and discomfort. People frequently do not touch each others’ eyes (perhaps because they are very delicate and sensitive?), but even less often do they touch each others’ lips (perhaps because of their traditionally highly intimate role in kissing). Strangers touch each other differently than partners. And each point cloud that’s created is indelibly affected by both the shape of the face being touched, and the choices made by the person touching that face. (Both of these are layered, of course, with data gathering glitches that add a kind of fuzziness.)

This software transmits OSC data describing the point cloud to a separate piece of display software, called "Facey Receiver," posted under its own repo.
