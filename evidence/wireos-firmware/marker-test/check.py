"""Build and run the upstream ARM detector on the recorded and rendered inputs."""
import gzip
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile

fixtures = Path(__file__).resolve().parent
repo = Path(sys.argv[1]).resolve()
build = repo / "_build/vicos/Simulator"
sdk = Path.home() / ".anki/vicos-sdk/dist" / (repo / "project/VICOS_SDK_VERSION").read_text().strip()
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    compile_db = json.loads((build / "compile_commands.json").read_text())
    entry = next(e for e in compile_db if e["file"].endswith("/markerDetector.cpp"))
    cmd = shlex.split(entry["command"])
    cmd[cmd.index("-o") + 1] = str(work / "probe.o")
    cmd[cmd.index("-c") + 1] = str(fixtures / "detector.cpp")
    subprocess.run(cmd, cwd=entry["directory"], check=True)

    commands = subprocess.check_output(["ninja", "-t", "commands", "vic-engine"], cwd=build, text=True)
    link = shlex.split(commands.splitlines()[-1].split(" && ")[1])
    link[link.index("-o") - 1] = str(work / "probe.o")
    link[link.index("-o") + 1] = str(work / "probe")
    index = link.index("lib/libcozmo_engine.so")
    link[index:index] = ["lib/libcti_vision.a", "lib/libcti_common_robot.a"]
    subprocess.run(link, cwd=build, check=True)

    for name in ["blank.png", "retreat.png", "reorientation.png"]:
        shutil.copyfile(fixtures / name, work / name)
    for name in ["prior.nv12", "revised.nv12"]:
        (work / name).write_bytes(gzip.decompress((fixtures / (name + ".gz")).read_bytes()))
    command = ["bwrap", "--unshare-user", "--unshare-pid"]
    for path, target in [(sdk / "sysroot/usr", "/usr"), (build, "/anki"), (work, "/probe"),
                         (repo / "3rd/avs-device-sdk/vicos/lib", "/avs"), (repo / "3rd/mpg123/vicos/lib", "/mpg"),
                         (Path(shutil.which("qemu-arm-static")).resolve(), "/qemu")]:
        command.extend(["--ro-bind", str(path), target])
    command.extend(["--symlink", "usr/lib", "/lib", "--dev", "/dev", "--proc", "/proc", "--tmpfs", "/tmp",
                    "--chdir", "/probe", "--", "/qemu", "-E", "LD_LIBRARY_PATH=/anki/lib:/avs:/mpg", "/probe/probe",
                    "blank.png", "retreat.png", "reorientation.png", "prior.nv12", "revised.nv12"])
    subprocess.run(command, check=True)
