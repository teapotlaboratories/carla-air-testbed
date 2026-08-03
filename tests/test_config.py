#!/usr/bin/env python3
"""The unified config renders correctly, and the generated files are not stale.

`configs/testbed.yaml` is the single source; `configs/sim/settings.json` and
`ros2_ws/src/bringup/config/testbed.yaml` are rendered from it because their readers - the CarlaUE4
binary and rclpy's parameter system - will not accept anything else.

Generated files that drift from their source are worse than no generation at all: the file
says one thing, the simulator does another, and nothing errors. `test_generated_files_are_current`
turns that into a failing test rather than a confusing flight.

    ./.venv/bin/python -m pytest tests/test_config.py -q
"""
from __future__ import annotations

import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import apply_config  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return apply_config.load()


def test_generated_files_are_current(cfg):
    """Run scripts/apply_config.py if this fails. It is the whole point of the check."""
    for path, rendered in ((apply_config.AIRSIM_OUT, apply_config.render_airsim(cfg)),
                           (apply_config.PARAMS_OUT, apply_config.render_params(cfg))):
        assert os.path.exists(path), f"{path} has never been generated"
        with open(path) as fh:
            assert fh.read() == rendered, (
                f"{os.path.relpath(path, ROOT)} is stale - run scripts/apply_config.py")


def test_camera_buffers_all_share_one_aspect_ratio(cfg):
    """4:3 across RGB, depth and segmentation, or grounding reads the wrong pixel.

    `fov` is HORIZONTAL, so two buffers with the same fov and different aspect ratios cover
    different vertical fields. `frames.scale_to()` then maps an RGB pixel onto the wrong depth
    pixel - silently, on every waypoint.
    """
    ratios = {name: c["width"] / c["height"] for name, c in cfg["simulator"]["cameras"].items()}
    assert len(set(round(r, 6) for r in ratios.values())) == 1, (
        f"camera aspect ratios disagree: {ratios}")


def test_clock_speed_is_one(cfg):
    """ClockSpeed accelerates AirSim but NOT CARLA, so anything else desyncs the world."""
    assert cfg["simulator"]["clock_speed"] == 1.0


def test_altitude_clamp_is_above_the_ned_ground(cfg):
    """min_altitude_m is NED, and the origin sits 27.45 m above the street on Town10HD.

    A clamp at or below -27.45 would let the controller command a point underground while the
    number still looked positive.
    """
    ctrl = cfg["graph"]["offboard_control"]
    assert ctrl["min_altitude_m"] > -27.45
    assert ctrl["max_altitude_m"] > ctrl["min_altitude_m"]


def test_every_sensor_declares_a_known_source(cfg):
    for s in cfg["sensors"]:
        assert s.get("source") in ("airsim", "carla"), f"{s.get('name')}: bad source"
        assert "name" in s and "enabled" in s


def test_carla_sensors_declare_a_blueprint(cfg):
    """An airsim sensor is auto-created and named; a CARLA one must say what to spawn."""
    for s in apply_config.sensors_for(cfg, "carla"):
        assert s.get("blueprint", "").startswith("sensor."), (
            f"{s['name']}: CARLA sensors need a blueprint id")


def test_airsim_sensors_are_not_given_blueprints(cfg):
    """AirSim sensors have no CARLA blueprint. One would be silently ignored."""
    for s in apply_config.sensors_for(cfg, "airsim"):
        assert "blueprint" not in s, f"{s['name']}: airsim sensors take no blueprint"


def test_sensor_names_are_unique(cfg):
    names = [s["name"] for s in cfg["sensors"]]
    assert len(names) == len(set(names)), f"duplicate sensor names: {names}"


def test_gps_origin_is_all_or_nothing(cfg):
    """A half-set origin would be written as a real coordinate with a zero in it."""
    o = cfg["simulator"].get("gps_origin") or {}
    have = [o.get("lat") is not None, o.get("lon") is not None]
    assert all(have) or not any(have), "set both gps_origin lat and lon, or neither"


def test_unset_gps_origin_is_omitted_not_zeroed(cfg):
    """AirSim treats a present OriginGeopoint as authoritative, so {0,0} is the Atlantic."""
    o = dict(cfg["simulator"])
    o["gps_origin"] = {"lat": None, "lon": None, "alt": 0.0}
    rendered = json.loads(apply_config.render_airsim({**cfg, "simulator": o}))
    assert "OriginGeopoint" not in rendered


def test_gps_origin_is_rendered_when_set(cfg):
    o = dict(cfg["simulator"])
    o["gps_origin"] = {"lat": 51.5074, "lon": -0.1278, "alt": 11.0}
    rendered = json.loads(apply_config.render_airsim({**cfg, "simulator": o}))
    assert rendered["OriginGeopoint"]["Latitude"] == pytest.approx(51.5074)


def test_ros_domain_is_not_a_ros_parameter(cfg):
    """It is an environment variable. Left in, rclpy would reject the file."""
    import yaml
    doc = yaml.safe_load(apply_config.render_params(cfg))
    for node, block in doc.items():
        assert "ros_domain_id" not in block["ros__parameters"], node


def test_params_render_in_the_schema_rclpy_expects(cfg):
    import yaml
    doc = yaml.safe_load(apply_config.render_params(cfg))
    assert "carla_air_bridge" in doc
    for node, block in doc.items():
        assert set(block) == {"ros__parameters"}, f"{node} has stray keys"


def test_the_guide_embeds_the_real_config_file():
    """docs/guide.html pastes the whole of configs/testbed.yaml. Catch it drifting.

    A doc that quotes a config is a copy, and copies rot silently - which is the same
    failure mode `test_generated_files_are_current` exists for, pointed at a reader
    instead of a machine. Regenerate with scripts/embed_config_in_guide.py.
    """
    import html
    import re

    guide = os.path.join(ROOT, "docs", "guide.html")
    with open(guide, encoding="utf-8") as fh:
        page = fh.read()

    block = re.search(r'<details class="full">.*?<pre>(.*?)</pre>', page, re.S)
    assert block, "the full-file listing is gone from docs/guide.html"

    embedded = html.unescape(re.sub(r"</?(?:span|b)[^>]*>", "", block.group(1)))
    with open(apply_config.SOURCE, encoding="utf-8") as fh:
        source = fh.read()

    assert embedded.strip("\n") == source.strip("\n"), (
        "docs/guide.html no longer matches configs/testbed.yaml - "
        "run scripts/embed_config_in_guide.py")
