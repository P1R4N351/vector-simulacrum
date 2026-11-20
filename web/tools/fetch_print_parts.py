"""Fetch Dauler's Vector replacement meshes for the local noncommercial demo."""
import hashlib
from io import BytesIO
from pathlib import Path
import sys
from urllib.request import urlopen
from zipfile import ZipFile

DEST = Path(__file__).resolve().parents[1] / "public/robot/sdk/print"
ARCHIVES = (
    ("3731746", "Anki_Vector_Lifting_Forks", "0344d40dea0106b11f922af91d4f949abad4698338dcf86e1d3292c2d345a350",
     ("forks7.stl",)),
    ("3764077", "Anki_Vector_Wheels_v2", "b041e299f4f225746aae14c8d6f53fe825edfd6f91205a563b5b3ef4f58c9944",
     ("wheel_front_stock.stl", "wheel_back_stock.stl")),
)


PART_SHA256 = {
    "forks7.stl": "c9dcd46e3c4b2c4b4aa896256109cd0b583a68adec432d615499c9c506dd281a",
    "wheel_front_stock.stl": "37afeacb4b811d02537eeca9c1090675a2188b7a9348ce4893636af79873fe88",
    "wheel_back_stock.stl": "e0cf202676553ae621cbb32086747d5d47cd0bfa3671d29d86493926997400bf",
}


def fetch_parts():
    files = {}
    for number, title, digest, parts in ARCHIVES:
        license_name = title + "-LICENSE.txt"
        if (DEST / license_name).is_file() and all(
            (DEST / name).is_file() and hashlib.sha256((DEST / name).read_bytes()).hexdigest() == PART_SHA256[name]
            for name in parts
        ):
            continue
        url = f"https://archive.org/download/thingiverse-{number}/{title}_{number}.zip"
        with urlopen(url, timeout=30) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError(f"unexpected archive hash for {title}")
        with ZipFile(BytesIO(data)) as archive:
            for name in parts:
                files[name] = archive.read("files/" + name)
            files[license_name] = archive.read("README.txt") + b"\n" + archive.read("LICENSE.txt")
    DEST.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        path = DEST / name
        if not path.exists() or path.read_bytes() != data:
            path.write_bytes(data)


if __name__ == "__main__":
    try:
        fetch_parts()
    except (ValueError, OSError) as error:
        sys.exit(str(error))
