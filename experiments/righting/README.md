# Two-Vector righting trials

Run a deterministic side-by-side comparison with a known patient pose. Both robots are free bodies. In the direct trial the rescuer approaches the patient's belly, drives with the lift low, raises it after fork contact and backs out. In the cube trial the sideways patient receives a free light cube and the rescuer approaches an adjacent face. This places the intended cube roll axis parallel to the patient's long axis. Contact can slip or separate at every step.

## Run and replay

From the repository root, use Python with NumPy and MuJoCo (baseline version 4.0.1). Use a fresh output directory per run. The commands below also prepare the optional SDK visuals from a local copy of the SDK wheel.

```sh
python -m pip download --no-deps wirepod_vector_sdk==0.8.1 -d /tmp/righting-sdk
python -m zipfile -e /tmp/righting-sdk/*.whl /tmp/righting-sdk/extracted
export VECTOR_SDK_ASSETS=/tmp/righting-sdk/extracted/anki_vector/opengl/assets
python web/tools/fetch_sdk_model.py --assets "$VECTOR_SDK_ASSETS"
npm ci --prefix web
npm ci --prefix experiments/righting
(cd experiments/righting && npx playwright-core install chromium)

python experiments/righting/run.py /tmp/direct-pair
python experiments/righting/run.py /tmp/roll-control --cube-obj "$VECTOR_SDK_ASSETS/cube.obj" --cube-control
python experiments/righting/run.py /tmp/patient-pair --cube-obj "$VECTOR_SDK_ASSETS/cube.obj" --patient-raise-deg 15
node experiments/righting/render.cjs /tmp/patient-pair
```

The renderer uses FFmpeg on PATH and starts a local web server for the duration of the replay. Set CHROMIUM_PATH to use an existing Chromium. VECTOR_SDK_ASSETS supplies the SDK's textured cube. Without that variable, the replay draws the contact boxes. Each run saves the exact collision scene, its hash, both pose traces, and a summary. Rendering writes righting.mp4 into the same directory. Frame time is simulation time. The orange dot shows a rescuer fork contact with the target.

Use --offset-mm to shift the approach along the patient, mirrored across the two trials. Direct trials accept --power and --delay-ms. Cube trials accept --presentation beside-fork or carry-pose, --patient-raise-deg, --cube-mass-g and --cube-friction. The control moves the patient clear while preserving the cube and rescuer setup. The carry-pose seed uses the engine's carried-cube offset of (26, 0, −12.5) mm from the wrist, then releases the cube into free dynamics. Initial contact penetration is reported in mm.

## What the cube represents

The collision generator samples the SDK cube mesh at its surface breakpoints, merges occupied cells into 17 boxes for the reference asset, and preserves the edge recesses that a single convex hull fills in. Each axis is scaled to the engine's 44 mm envelope. The mesh is a visual reference for the relative dimensions of the recesses. Sub-millimetre edge details and the printed patient's fork profile still need physical checks. The experiment uses centered solid-cube inertia. Its nominal mass is the existing simulacrum's assumed 50 g. The 85 g sensitivity case comes from the vendor Webots model. Neither value is a weighing of the household cube. The peer simulator separately models the carried cube's accelerometer and packet timing. This trial concerns free-body contact. [Vendor cube and face markers](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/simulator/protos/LightCube.proto) · [Vendor Webots size and mass](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/simulator/protos/Block1x1.proto)

The rescuer follows a scripted approximation of the low-roll branch. It approaches with lift at carry height, lowers toward 32 mm at 0.75 rad/s, and requests 60 mm/s reverse when the lift crosses 68 mm. Approach uses the known cube pose, a 13 mm offset from the near face and a bounded reverse interval. The stock voice intent delegates to PlayRollBlock, which normally selects an approach itself. Vision, voice, the full docking controller, and a learned rescue policy are outside this replay. [Voice behavior](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/resources/config/engine/behaviorComponent/behaviors/victorBehaviorTree/highLevelDelegates/voiceCommands/rollCubeVoiceCommand.json) · [Controller](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/supervisor/src/pickAndPlaceController.cpp)

## First comparison

The direct baseline reproduced both recorded final patient tilts to machine precision (100.88° from upright on either side). The cube-alone control settled on the adjacent face, 90.03° from its starting up vector. At the nominal mass and friction, a 15° patient lift sweep from the carry-pose seed produced 0.374 s and 0.384 s of patient fork contact. Both had zero simultaneous fork-contact time during the rescuer's action. Both patients finished on their sides. Contact durations accumulate solver steps and do not establish a load-bearing grip. None of the sampled mass/friction combinations sustained engagement through handoff. [Per-trial measurements](cube-trials.csv)

The next discriminating observation is whether a sideways patient's fork can retain a cube through a gentle pull from an adjacent face. Record the contact edge, cube orientation, and whether the fork releases before the patient begins rolling. That observation would constrain the fork-to-lip contact geometry independently of the rescuer's controller.

For the follow-up with an already-engaged patient, use the [leverage control and morning close-up protocol](morning-grip.md).

For a patient equipped with Dauler's printed lifting fork, supply `--patient-fork-stl /path/to/forks7.stl` with CoACD 1.0.14 installed. The run builds the wrist collision surface from convex parts using the web model's registration. It retains the wrist mass and placement reference. The shelf replay documents where this geometry transfers to unit 3 and where the grip still slips. [Printed fork shelf trials](../../evidence/unit3-printed-fork-contact/README.md)
