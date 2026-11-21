# Cube accelerometer during a physical pickup

The overhead camera shows the cube leave the shelf, travel through two opposed 20° commands, and return to the shelf. The robot's stock pickup uses a printed fork. This is a separate acquisition from the earlier improperly engaged trial. Here the cube's own three-axis accelerometer ran through the pickup.

![Cube acceleration and lift feedback](pickup-overlay.png)

[Overhead video](pickup.mp4) · [Selected frames](pickup-frames.jpg) · [Raw Bluetooth packets](pickup.tsv) · [Decoded counts](pickup.csv) · [Action timestamps](action-times.csv) · [Robot feedback](robot.csv)

At rest before pickup, median XYZ was (+0.002, −0.018, +0.971) g. During the relatively still raised interval 6.5–8.3 s after the command it was (−0.032, −0.528, +0.823) g. The angle between those median specific-force vectors is about 32°. This estimates the cube's change in tilt when translational accelerations are small. The accelerometer alone cannot identify fork contact points or rotation about gravity. The largest sampled magnitude during the two turns was 1.25 g and during the placement command it was 1.67 g. In the 0.9 s after the disconnect request, median XYZ was (+0.003, −0.019, +0.970) g. The largest magnitude in the entire file, 3.34 g, occurred much later during docking.

The radio recorder listened to the existing Bluetooth daemon socket, filtered notifications on the cube's accelerometer characteristic, and never wrote to the socket. Robot firmware offered stock app version 2018-08-14T17:40. Each type-2 packet holds a tap count and three XYZ observations as little-endian signed 16-bit values, nominally 8192 counts/g. The lift session yielded 1,309 acceleration packets and 3,927 XYZ observations. Radio and host delivery are batched. The longest gap between robot packet receipt timestamps was 280 ms. The action overlay uses pod receipt times for both robot and cube, with all three slots plotted at packet arrival. These times do not establish a precise subpacket impact time. The tap byte is available in the decoded files.

A separate docked baseline yielded 1,929 XYZ observations across 23.19 s. Mean magnitude was 0.971 g. On the first attempt the stream went stale during undocking and the robot returned to dock before pickup. Connecting after undocking obtained the movement recording. Disconnect was requested after placement. Notifications continued for a time after the request. The disposable cell reported at least 1.56 V across the recordings.

[Three capture segments](cube-xyz.png) · [Socket listener](robot_packets.pl) · [Decoder](decode.py) · [Overlay generator](pickup_plot.py). Run `python decode.py resting.tsv undock-check.tsv pickup.tsv` followed by `python pickup_plot.py`.
