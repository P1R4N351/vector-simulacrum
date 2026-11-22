# Dock-start wheelstand with a visible cube

With the head set to −8°, unit 3 saw cube ID 1 from the charger. A first setup attempt ended before a wheelstand request when cube Bluetooth connection failed. On the repeat, Bluetooth again failed but a fresh visual sighting supplied the target for the stock wheelstand command. The robot drove off the dock under that command. The experimenter observed the cube being tossed off the shelf edge. Record this as a cube ejection, not a successful supported wheelstand. Its subsequent landing and condition were not measured.

The engine returned UNEXPECTED_PITCH_ANGLE after 5.904 seconds. Across 99 SDK states during the action, reported chassis pitch ranged from −11.55° to +3.85°, and lift height reached 91.55 mm. The cube accelerometer recorder received zero packets, so impact acceleration is unknown. The post-action native dock completed and the battery report confirmed platform contact at 4.047 V, full. Quiet was on at handback. Further shelf wheelstands stopped following the ejection.

[Corrected twin-camera clip](dock-request.mp4) has the wheelstand request about 3.1 s in. [Action log](trial.csv) · [SDK states](state.csv.gz) · [Object sightings](sightings.csv). Times are relative to the pre-command sighting check. The large cube-stream age in the action log reflects a recorder that never received its first packet.
