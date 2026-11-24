# Contributing to vector-simulacrum

The model's parameter table (see the README section "The parameter table")
holds one assumed value or one measured value for each physical constant on
the robot. "Measured" means one or more real Vectors have been recorded
through a known motion and the value fits that recording. Right now that
rests on a small number of units; every additional unit narrows the spread
on anything per-unit.

**What you can contribute today.** Recordings from your own Vector through
the same recorder scripts this project uses, with a short metadata file.
A `contrib/kit/` command runs the recorder, applies the privacy scrub, and
produces a folder you add to your fork and open a pull request with.

- Code and scripts are MPL-2.0 (see LICENSE).
- Contributed robot recordings and metadata land under CC BY 4.0 by default;
  the licence line in the pull request records your choice.

## When not to contribute from this kit

Skip the kit and open an issue instead when:

- You cannot run a Python 3.10+ environment that can talk to your Vector.
- Your Vector is in a place where the camera feed would include faces,
  mail, screens, address labels, or anything else a public dataset must
  not carry. The recorder scripts referenced below do not stream frames
  for that reason, but a future scope might; read the kit's README before
  recording the camera.
- You are not the owner of the robot and cannot grant a CC BY 4.0 licence
  on recordings from it.

## The submission folder

Each contribution is one folder under `contrib/units/<short-handle>-<n>/`:

```
contrib/units/<handle>-<n>/
  metadata.yaml              # required; schema under contrib/
  evidence/
    rest.jsonl               # at least one recording; one JSONL per run
    spins.jsonl              # ...
    straights.jsonl          # ...
  README.md                  # optional, a few sentences on the setup
```

`<handle>` is any short lowercase-ASCII label you pick (your GitHub handle
is fine, or a word that is not your name). `<n>` is `1` for your first
unit, `2` for a second one, and so on.

### Metadata

`metadata.yaml` has a small, fixed set of fields. A template and the live
schema are in `contrib/kit/metadata-template.yaml` and `contrib/validator.py`.

Every submission must have:

- `hardware_generation`: `1.0` or `2.0`
- `firmware_version`: the string your Vector reports (for example `1.9.0`)
- `body_modifications`: free text (`stock`, or what is different)
- `recorded_at`: ISO date, the day you ran the recorder
- `licence`: `CC BY 4.0` (default) or another SPDX identifier
- `sign_off`: `I release this data under <licence>.`

Optional:

- `contributor`: your name or handle. The project does not need it; leave
  it blank or omit it if you would rather not be credited.
- `surface`: a short phrase for what the robot drove on (`laminate`,
  `polished wood`, `carpet`, etc.).
- `room_temperature_c`: integer degrees Celsius.
- `notes`: free text.

The validator refuses a submission that names a specific person outside
`contributor`, or that names an email address, IP address, Bluetooth
address or SDK GUID anywhere.

## The kit

```bash
# From a clone of your fork:
python3 contrib/kit/record.py --unit-handle <handle>-<n> \
    --vector-serial <your-robot-serial> \
    --phases rest,headlift,spins,straights,actions
```

What the kit does, in order:

1. Checks the Vector SDK is importable and `anki_vector` can connect.
2. Prompts for every required metadata field it cannot read off the robot,
   and writes `metadata.yaml`.
3. Runs `hardware/record_barriered.py` for each named phase; see that
   script for what each phase commands and how much clear space it needs.
4. Writes the result into `contrib/units/<handle>-<n>/evidence/`.
5. Runs `contrib/kit/scrub.py` over the folder. The scrub removes the
   serial, any IP address, the SDK session id, any Bluetooth address and
   any email. It rejects a folder it cannot leave clean.
6. Runs `contrib/validator.py` and reports the result.

If you would rather drive the recorder directly, the kit is a thin wrapper
and you can read it in one sitting. The validator and the scrub are
self-contained, run in Python's standard library, and do not need a
network connection.

## Opening the pull request

- Fork `P1R4N351/vector-simulacrum`.
- Add your `contrib/units/<handle>-<n>/` folder on a branch.
- Open a pull request against `main`. The pull request template gives you a
  checklist, including the data licence sign-off.
- The `validate-contrib` GitHub Action runs `contrib/validator.py` against
  the branch; it must pass before the pull request can be merged.

## How a contribution lands in the model

A contribution does not change the generated MJCF directly. It lands as a
validated unit folder under `contrib/units/` and is recorded in
[`contrib/UNITS.md`](contrib/UNITS.md). Today `fit_recording.py` does not scan
`contrib/units/` automatically; accepted recordings are incorporated into
per-unit refits and spread analyses once merged.

## Issues, questions and small corrections

Open an issue if you are unsure whether to submit, found a bug in the kit,
or want to point out that a parameter is wrong in a way a recording would
not catch. The `robot-data` issue form is the simplest way to share a
small observation without running the kit.
