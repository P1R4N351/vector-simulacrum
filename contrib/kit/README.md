# Running the kit

The kit records your Vector through the project's standard phases and
packages the result as a contribution. Before you run it:

- Python 3.10 or newer with `anki_vector` installed (the Vector SDK).
- Your Vector authenticated to the SDK (see the Vector SDK docs). The
  robot can be behind wire-pod or a stock engine; what matters is that
  `anki_vector.Robot(...).connect()` succeeds.
- A table with a wall or soft landing on every side. Several of the
  phases drive the robot by up to 450 mm in one direction.
- About five minutes once the robot is on the table.

```bash
python3 contrib/kit/record.py \
    --unit-handle <handle>-1 \
    --vector-serial <8-hex-digit-serial> \
    --phases rest,headlift,spins,straights,actions
```

The arguments:

- `--unit-handle` is a short label (lowercase ASCII, digits, dashes). The
  kit refuses one that carries a person's first name or email prefix.
- `--vector-serial` is the eight-hex-digit serial your Vector reports
  (the SDK reads it off `robot.serial`). The scrub replaces it everywhere
  in the output.
- `--phases` is a comma-separated subset of the recorder's phases. See
  `hardware/record_barriered.py` for what each phase does. The first
  contribution usually runs `rest,headlift,spins,straights,actions`.

What the kit does, step by step, is described in [CONTRIBUTING.md](../../CONTRIBUTING.md).

The kit is a thin wrapper. You can run `hardware/record_barriered.py`
directly, then run `contrib/kit/scrub.py` on the output and
`contrib/validator.py` on the folder. The validator is what matters; the
kit just saves you a few keystrokes.
