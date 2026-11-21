# contrib/

Everything in this directory is contribution infrastructure: the recording
kit an outside Vector owner runs, the validator the project runs on a
pull request, and the folder where contributed units land.

See [`CONTRIBUTING.md`](../CONTRIBUTING.md) for the step-by-step.

```
contrib/
  README.md                  # this file
  UNITS.md                   # list of contributed units once there are any
  validator.py               # validates a submission folder; also used by CI
  kit/
    README.md                # how to run the kit on your own robot
    record.py                # the recorder wrapper
    scrub.py                 # strips serials, addresses, GUIDs
    metadata-template.yaml   # the metadata file layout
  units/                     # contributed units land here, one folder each
  tests/
    test_validator.py
    fixtures/
      good-submission/       # minimal example that validates
      bad-submission-with-leak/   # fails the privacy scrub
```

The project's own recordings live under `evidence/`, not here. `contrib/`
is reserved for third-party contributions and the kit itself.
