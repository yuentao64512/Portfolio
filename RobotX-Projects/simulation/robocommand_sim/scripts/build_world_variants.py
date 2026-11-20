#!/usr/bin/env python3
"""Build core and advanced Task-1 worlds without changing the original world."""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

COURSE_MODELS = {
    "entry_blue", "green_1", "red_1", "black_1a", "green_2",
    "red_2", "green_4", "red_4", "black_4a", "exit_blue",
}


def set_rgba(material: ET.Element, tag: str, value: str) -> None:
    child = material.find(tag)
    if child is None:
        child = ET.SubElement(material, tag)
    child.text = value


def advanced_variant(root: ET.Element) -> None:
    world = root.find("world")
    if world is None:
        raise ValueError("SDF has no world")
    for model in world.findall("model"):
        name = model.get("name", "")
        if name not in COURSE_MODELS:
            continue
        body = model.find("./link/visual[@name='body']")
        beacon = model.find("./link/visual[@name='top_beacon']")
        if body is not None and name not in {"black_1a", "black_4a"}:
            material = body.find("material")
            if material is not None:
                set_rgba(material, "ambient", "0.12 0.12 0.12 1")
                set_rgba(material, "diffuse", "0.18 0.18 0.18 1")
                set_rgba(material, "emissive", "0 0 0 1")
        if beacon is not None:
            geometry = beacon.find("geometry")
            if geometry is not None:
                for child in list(geometry):
                    geometry.remove(child)
                cylinder = ET.SubElement(geometry, "cylinder")
                ET.SubElement(cylinder, "radius").text = "0.28"
                ET.SubElement(cylinder, "length").text = "0.035"
            pose = beacon.find("pose")
            if pose is not None:
                pose.text = "0 0 0.82 0 0 0"
    for light in list(world.findall("light")):
        if light.get("name", "").removesuffix("_light") in COURSE_MODELS:
            world.remove(light)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    original = ET.parse(args.source)
    world = original.getroot().find("world")
    if world is None:
        raise ValueError("SDF has no world")
    boat = world.find("./include[name='usv_four_thruster']/uri")
    if boat is None:
        raise ValueError("USV include missing from source world")
    boat.text = "model://usv_four_thruster_cv"
    core_path = args.output_dir / "task1_core.world"
    advanced_path = args.output_dir / "task1_advanced.world"
    original.write(core_path, encoding="unicode", xml_declaration=True)
    advanced = copy.deepcopy(original)
    advanced_variant(advanced.getroot())
    advanced.write(advanced_path, encoding="unicode", xml_declaration=True)
    print(core_path)
    print(advanced_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

