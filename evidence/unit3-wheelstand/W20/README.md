# Dock-start wheelstand with a visible cube

With the head set to −8°, unit 3 saw cube ID 1 from the charger. A first setup attempt ended before a wheelstand request when cube Bluetooth connection failed. On the repeat, Bluetooth again failed but a fresh visual sighting supplied the target for the stock wheelstand command. The robot drove off the dock under that command. The experimenter observed the cube being tossed off the shelf edge. Record this as a cube ejection, not a successful supported wheelstand. Its subsequent landing and condition were not measured.

The engine returned UNEXPECTED_PITCH_ANGLE after 5.904 seconds. Across 99 SDK states during the action, reported chassis pitch ranged from −11.55° to +3.85°, and lift height reached 91.55 mm. The cube accelerometer recorder received zero packets, so impact acceleration is unknown. The post-action native dock completed and the battery report confirmed platform contact at 4.047 V, full. Quiet was on at handback. Further shelf wheelstands stopped following the ejection.

[Corrected twin-camera clip](dock-request.mp4) has the wheelstand request about 3.1 s in. [Action log](trial.csv) · [SDK states](state.csv.gz) · [Object sightings](sightings.csv). Times are relative to the pre-command sighting check. The large cube-stream age in the action log reflects a recorder that never received its first packet.

### Ejection compared with three other physical wheelstands

Both corrected views show the cube moving away during the lift downstroke. In the 8 fps video it is still upright at about 5.03 s after the request and visibly displaced and rotating by 5.28 s. The first SDK sample with a lift drop greater than 3 mm arrived at 5.215 s. The video identifies the departure from the contact area. The experimenter supplied the off-shelf outcome; the landing is outside these views. Camera frame spacing is 125 ms, and SDK times are host receipt times.

Within 400 ms of that first lift drop, the peak mean wheel speed was 141.5 mm/s in W20, compared with 140.0 and 137.5 mm/s in W4 and W8. All three lowered the fork to about 30 mm. Peak reported chassis pitch in the same window was 3.0° in W20 versus 72.3° and 72.5° in W4 and W8. W12 reached 29.0°. These recordings give the contact model three observed outcomes despite similar actuator motion. The wheel encoders alone cannot establish tread traction, force at the fork, or which tooth was engaged. [Aligned traces and camera sequence](ejection.png) · [Window scores](ejection.csv) · [Reproduce](ejection.py).
