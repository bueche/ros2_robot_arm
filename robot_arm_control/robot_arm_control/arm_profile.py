"""Arm profiles: the per-arm facts shared by robot_arm_control nodes.

A profile is a YAML file in robot_arm_description/share/config/arm_profiles/.
Nodes take an 'arm_profile' parameter that is either a profile name
('koch_v1_1', 'so_arm101') or a path to a profile file.

The profile's 'launch' section is read by robot_arm_description's
robot_arm.launch.py; this module only takes launch.urdf from it (urdf_path).
"""

import os
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional

import yaml

PROFILE_PACKAGE = 'robot_arm_description'
PROFILE_SUBDIR = os.path.join('config', 'arm_profiles')
BACKENDS = ('dynamixel', 'feetech')
# Per-servo telemetry fields of DynamixelState / FeetechState that a servo may or
# may not provide. A joint's 'reports' list names the ones its servo provides.
TELEMETRY_FIELDS = ('present_temperature', 'present_input_voltage', 'present_current', 'present_load')


class ProfileError(ValueError):
    """Raised when a profile file is missing or malformed."""


@dataclass(frozen=True)
class JointSpec:
    name: str
    id: int
    model: str
    rail: str
    reports: FrozenSet[str]  # subset of TELEMETRY_FIELDS


@dataclass(frozen=True)
class Thresholds:
    temperature_warning_c: Optional[float]
    temperature_critical_c: Optional[float]
    load_warning_pct: Optional[float]
    current_warning_ma: Optional[float]


@dataclass(frozen=True)
class ArmProfile:
    name: str
    path: str
    controller: str                       # JointTrajectoryController name
    telemetry_backend: str
    telemetry_topic: str
    joints: List[JointSpec]               # command order
    rail_min_voltage: Dict[str, Optional[float]]
    safe_position: Dict[str, float]
    thresholds: Thresholds
    programs: Dict[str, List[dict]]       # name -> [{'name': str, 'positions': {joint: rad}}]
    urdf_path: Optional[str]              # launch.urdf, resolved in the robot_arm_description share dir

    @property
    def trajectory_topic(self) -> str:
        return f'/{self.controller}/joint_trajectory'

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def joint_by_id(self) -> Dict[int, JointSpec]:
        return {j.id: j for j in self.joints}

    @property
    def joint_by_name(self) -> Dict[str, JointSpec]:
        return {j.name: j for j in self.joints}

    def joints_reporting(self, field: str) -> List[JointSpec]:
        """Joints whose servo provides telemetry field 'field', in command order."""
        if field not in TELEMETRY_FIELDS:
            raise ValueError(f"unknown telemetry field '{field}' (expected one of {TELEMETRY_FIELDS})")
        return [j for j in self.joints if field in j.reports]


def resolve_profile_path(name_or_path: str) -> str:
    if not name_or_path:
        raise ProfileError("arm_profile parameter is empty")
    if os.path.isfile(name_or_path):
        return name_or_path
    if os.sep in name_or_path or name_or_path.endswith('.yaml'):
        raise ProfileError(f"arm profile file not found: '{name_or_path}'")
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory(PROFILE_PACKAGE), PROFILE_SUBDIR, f'{name_or_path}.yaml')
    if not os.path.isfile(path):
        raise ProfileError(f"no arm profile named '{name_or_path}' (looked for {path})")
    return path


def _opt_float(value, what):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ProfileError(f"{what}: expected a number or null, got {value!r}")


def load_arm_profile(name_or_path: str) -> ArmProfile:
    path = resolve_profile_path(name_or_path)
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        raise ProfileError(f"could not read arm profile {path}: {e}")

    def need(d, key, where):
        if key not in d:
            raise ProfileError(f"{path}: missing '{key}' in {where}")
        return d[key]

    telemetry = need(data, 'telemetry', 'profile')
    backend = need(telemetry, 'backend', 'telemetry')
    if backend not in BACKENDS:
        raise ProfileError(f"{path}: telemetry.backend '{backend}' not one of {BACKENDS}")

    rails = need(data, 'rails', 'profile') or {}
    rail_min_voltage = {str(name): _opt_float((cfg or {}).get('min_voltage'), f"rails.{name}.min_voltage")
                        for name, cfg in rails.items()}

    joints = []
    seen_ids = set()
    for name, cfg in (need(data, 'joints', 'profile') or {}).items():
        where = f"joints.{name}"
        sid = int(need(cfg, 'id', where))
        if sid in seen_ids:
            raise ProfileError(f"{path}: duplicate servo id {sid} ({where})")
        seen_ids.add(sid)
        rail = str(need(cfg, 'rail', where))
        if rail not in rail_min_voltage:
            raise ProfileError(f"{path}: {where}.rail '{rail}' is not defined under 'rails'")
        reports = frozenset(cfg.get('reports', []))
        unknown = reports - set(TELEMETRY_FIELDS)
        if unknown:
            raise ProfileError(f"{path}: {where}.reports has unknown fields {sorted(unknown)} "
                               f"(expected any of {TELEMETRY_FIELDS})")
        joints.append(JointSpec(name=name, id=sid, model=str(cfg.get('model', '')), rail=rail, reports=reports))
    if not joints:
        raise ProfileError(f"{path}: no joints defined")

    names = {j.name for j in joints}
    safe_position = {k: float(v) for k, v in (data.get('safe_position') or {}).items()}
    missing = names - set(safe_position)
    extra = set(safe_position) - names
    if missing or extra:
        raise ProfileError(f"{path}: safe_position must list exactly the profile joints "
                           f"(missing {sorted(missing)}, unknown {sorted(extra)})")

    t = data.get('thresholds') or {}
    thresholds = Thresholds(
        temperature_warning_c=_opt_float(t.get('temperature_warning_c'), 'thresholds.temperature_warning_c'),
        temperature_critical_c=_opt_float(t.get('temperature_critical_c'), 'thresholds.temperature_critical_c'),
        load_warning_pct=_opt_float(t.get('load_warning_pct'), 'thresholds.load_warning_pct'),
        current_warning_ma=_opt_float(t.get('current_warning_ma'), 'thresholds.current_warning_ma'),
    )

    programs = {}
    for prog_name, poses in (data.get('programs') or {}).items():
        if not isinstance(poses, list) or not poses:
            raise ProfileError(f"{path}: programs.{prog_name} must be a non-empty list of poses")
        for k, pose in enumerate(poses):
            positions = (pose or {}).get('positions') or {}
            unknown = set(positions) - names
            if unknown:
                raise ProfileError(f"{path}: programs.{prog_name}[{k}] has unknown joints {sorted(unknown)}")
        programs[str(prog_name)] = [
            {'name': str(pose.get('name', f'{prog_name}_{k}')),
             'positions': {j: float(v) for j, v in pose['positions'].items()}}
            for k, pose in enumerate(poses)]

    urdf_path = None
    urdf_rel = (data.get('launch') or {}).get('urdf')
    if urdf_rel:
        from ament_index_python.packages import get_package_share_directory
        urdf_path = os.path.join(get_package_share_directory(PROFILE_PACKAGE), urdf_rel)

    return ArmProfile(
        name=str(data.get('name', os.path.splitext(os.path.basename(path))[0])),
        path=path,
        controller=str(need(data, 'controller', 'profile')),
        telemetry_backend=backend,
        telemetry_topic=str(need(telemetry, 'topic', 'telemetry')),
        joints=joints,
        rail_min_voltage=rail_min_voltage,
        safe_position=safe_position,
        thresholds=thresholds,
        programs=programs,
        urdf_path=urdf_path,
    )


def load_urdf_joint_limits(urdf_path: str) -> Dict[str, tuple]:
    """Revolute/prismatic joint limits from a URDF as {joint: (min, max)} in rad.

    URDF lower/upper are taken as an unordered pair (some URDFs in this
    workspace list them in physical-direction order, e.g. lower > upper).
    """
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as e:
        raise ProfileError(f"could not read URDF {urdf_path}: {e}")
    limits = {}
    for joint in root.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic'):
            continue
        lim = joint.find('limit')
        if lim is None or lim.get('lower') is None or lim.get('upper') is None:
            continue
        lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        limits[joint.get('name')] = (min(lo, hi), max(lo, hi))
    return limits
