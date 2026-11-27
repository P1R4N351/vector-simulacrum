"""Read only motor keyframes from the two stock animation archives."""
import json
import sys
import tarfile

clips={}
for file in sys.argv[1:]:
    with tarfile.open(file) as tar:
        for member in tar:
            if "straight" not in member.name and "binaryeyes" not in member.name:
                continue
            name,frames=next(iter(json.load(tar.extractfile(member)).items()))
            clips[name.removeprefix("anim_chargerdocking_comeoff_")]=[r for r in frames if r["Name"] in
                ("BodyMotionKeyFrame","HeadAngleKeyFrame","LiftHeightKeyFrame")]
print(json.dumps(clips))
