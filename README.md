# Live-motion-joint-angle-range

Author: Ryan Harvey
Date Created: 04/04/19
Files: VideoIsolation.py, VideoIsolation.txt, Summary of Proprioceptive Training for the Prevention of Ankle Sprains

Program was written for Engineering III at Holy Innocents Episcopal School

This program is meant to claculate the angle in which an ankle is bending in order to keep the ankle within a certain rangle while preforming a motion.
Four colored dots are placed on the toe and just below the ankle and just above the ankle and shin.
The program isolates these colors and tracks their movement, calculating the angle and telling the user when they have flexed to far.

It is written it Python using numpy and cv2.  The program places masks on the video feed and determines the center of mass of each colored dot.
It then calculates the slope of both lines form dot to dot and calculates the angle between the lines.
The angle is currently set to be in range between 100 degrees and 130 which is generally the maximum range of motion withought risk of injury
