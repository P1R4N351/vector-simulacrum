"""Variable spacing for the published contact_trial.py auxiliary tread rollers.

A 0 mm inset retains the proxy machinery for a matched experimental control.
Use the unmodified source XML separately for the production baseline.
"""
from copy import deepcopy
import math
import xml.etree.ElementTree as ET


def contact_xml(xml, inset_mm, forward_shift_mm=0.0):
    """Move only the load-bearing roller centers inward along chassis x."""
    if not math.isfinite(inset_mm) or not 0 <= inset_mm < 12:
        raise ValueError("inset must be finite, in [0, 12) mm per end")
    if not math.isfinite(forward_shift_mm) or abs(forward_shift_mm) > inset_mm:
        raise ValueError("contact centers must remain between the axles")
    root = ET.fromstring(xml)
    root.find("option/flag").set("multiccd", "disable")
    timestep = float(root.find("option").get("timestep"))
    settle_s = max(0.010, 2 * timestep)
    chassis = root.find(".//body[@name='chassis']")
    for suffix in ("fl", "fr", "rl", "rr"):
        name = "wheel_" + suffix
        axle = chassis.find(f"body[@name='{name}']")
        position = list(map(float, axle.get("pos").split()))
        original_x = 0 if suffix[0] == "f" else -0.048
        if abs(position[0] - original_x) > 1e-9:
            raise ValueError(f"{name} expected at stock axle, got {position[0]} m")
        position[0] += (-1 if suffix[0] == "f" else 1) * inset_mm / 1000
        position[0] += forward_shift_mm / 1000
        roller = ET.SubElement(chassis, "body", name="patch_" + suffix,
                               pos=" ".join(f"{x:.12g}" for x in position))
        shaft = axle.find("joint")
        armature = float(shaft.get("armature"))
        shaft.set("armature", str(0.9 * armature))
        ET.SubElement(roller, "joint", name="patch_" + suffix, type="hinge",
                      axis="0 1 0", armature=str(0.1 * armature))
        tread = axle.find("geom")
        contact = deepcopy(tread)
        contact.attrib.update(name="wheel_patch_" + suffix, mass="0.000001")
        roller.append(contact)
        inertia = deepcopy(tread)
        inertia.attrib.update(name="mass_" + suffix, contype="0", conaffinity="0",
                              mass=str(float(tread.get("mass")) - 0.000001))
        axle.append(inertia)
        tread.attrib.update(mass="0", size=str(float(tread.get("size")) - 0.0003))
        ET.SubElement(root.find("equality"), "joint", joint1="patch_" + suffix,
                      joint2=name + "_joint", polycoef="0 1 0 0 0", solref=f"{settle_s} 1")
    return ET.tostring(root, encoding="unicode")
