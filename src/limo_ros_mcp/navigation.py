"""Operator-owned map policy and static Navigation Contract v2 validation."""

from __future__ import annotations

import hashlib
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from limo_ros_mcp.evidence import sha256_document


@dataclass(frozen=True)
class PatrolMapPolicy:
    route_policy_id: str
    map_id: str
    frame_id: str
    map_yaml_uri: str
    map_yaml_sha256: str
    map_image_uri: str
    map_image_sha256: str
    resolution_m: float
    width_cells: int
    height_cells: int
    origin_x: float
    origin_y: float
    occupied_threshold: float
    free_threshold: float
    allow_unknown: bool
    geofence: tuple[tuple[float, float], ...]
    no_go_zones: tuple[tuple[tuple[float, float], ...], ...]
    default_xy_m: float
    default_yaw_rad: float
    min_xy_m: float
    max_xy_m: float
    min_yaw_rad: float
    max_yaw_rad: float
    minimum_obstacle_clearance_m: float
    expected_motion_mode: int
    policy_hash: str


def _default_policy_path() -> Path:
    installed = Path(__file__).resolve().parent / "data" / "configs" / "patrol_lab.example.yaml"
    if installed.exists():
        return installed
    return Path(__file__).resolve().parents[2] / "configs" / "patrol_lab.example.yaml"


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _polygon(name: str, value: Any) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{name} must contain at least three points")
    points: list[tuple[float, float]] = []
    for index, raw in enumerate(value):
        if not isinstance(raw, list) or len(raw) != 2:
            raise ValueError(f"{name}[{index}] must be [x, y]")
        points.append(
            (_finite(f"{name}[{index}].x", raw[0]), _finite(f"{name}[{index}].y", raw[1]))
        )
    return tuple(points)


def load_patrol_map_policy(path: str | Path | None = None) -> PatrolMapPolicy:
    configured = path or os.environ.get("ROSCLAW_LIMO_PATROL_MAP")
    policy_path = Path(configured).expanduser() if configured else _default_policy_path()
    raw = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != "limo.patrol-map.v1":
        raise ValueError("unsupported patrol map policy")
    map_config = raw.get("map")
    geofence_config = raw.get("geofence")
    tolerance = raw.get("goal_tolerance")
    if not isinstance(map_config, dict) or not isinstance(geofence_config, dict):
        raise ValueError("map and geofence must be objects")
    if not isinstance(tolerance, dict):
        raise ValueError("goal_tolerance must be an object")
    origin = map_config.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("map.origin must be [x, y, yaw]")
    width = map_config.get("width_cells")
    height = map_config.get("height_cells")
    if not isinstance(width, int) or isinstance(width, bool) or width <= 0:
        raise ValueError("map.width_cells must be a positive integer")
    if not isinstance(height, int) or isinstance(height, bool) or height <= 0:
        raise ValueError("map.height_cells must be a positive integer")
    resolution = _finite("map.resolution_m", map_config.get("resolution_m"))
    if not 0.001 <= resolution <= 1.0:
        raise ValueError("map.resolution_m must be within [0.001, 1.0]")
    occupied = _finite("map.occupied_threshold", map_config.get("occupied_threshold"))
    free = _finite("map.free_threshold", map_config.get("free_threshold"))
    if not 0.0 <= free < occupied <= 1.0:
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")
    no_go_raw = raw.get("no_go_zones", [])
    if not isinstance(no_go_raw, list):
        raise ValueError("no_go_zones must be a list")

    values = {
        key: _finite(f"goal_tolerance.{key}", tolerance.get(key))
        for key in (
            "default_xy_m",
            "default_yaw_rad",
            "min_xy_m",
            "max_xy_m",
            "min_yaw_rad",
            "max_yaw_rad",
        )
    }
    if not 0.01 <= values["min_xy_m"] <= values["default_xy_m"] <= values["max_xy_m"] <= 1.0:
        raise ValueError("xy goal tolerance bounds are invalid")
    if (
        not 0.01
        <= values["min_yaw_rad"]
        <= values["default_yaw_rad"]
        <= values["max_yaw_rad"]
        <= math.pi
    ):
        raise ValueError("yaw goal tolerance bounds are invalid")
    clearance = _finite("minimum_obstacle_clearance_m", raw.get("minimum_obstacle_clearance_m"))
    if not 0.05 <= clearance <= 2.0:
        raise ValueError("minimum_obstacle_clearance_m must be within [0.05, 2.0]")
    motion_mode = raw.get("expected_motion_mode")
    if isinstance(motion_mode, bool) or motion_mode not in {0, 1, 2}:
        raise ValueError("expected_motion_mode must be 0, 1, or 2")
    allow_unknown = map_config.get("allow_unknown", False)
    if not isinstance(allow_unknown, bool):
        raise ValueError("map.allow_unknown must be a boolean")
    for key in (
        "route_policy_id",
        "map_id",
        "frame_id",
        "map_yaml_uri",
        "map_yaml_sha256",
        "map_image_uri",
        "map_image_sha256",
    ):
        if not isinstance(raw.get(key), str) or not str(raw[key]).strip():
            raise ValueError(f"{key} must be a non-empty string")
    if raw["frame_id"] != "map":
        raise ValueError("frame_id must be map")
    if _finite("map.origin.yaw", origin[2]) != 0.0:
        raise ValueError("rotated map origins are not supported")
    for key in ("map_yaml_sha256", "map_image_sha256"):
        if re.fullmatch(r"sha256:[0-9a-f]{64}", str(raw[key])) is None:
            raise ValueError(f"{key} must be a tagged SHA-256 digest")

    return PatrolMapPolicy(
        route_policy_id=str(raw.get("route_policy_id", "")),
        map_id=str(raw.get("map_id", "")),
        frame_id=str(raw.get("frame_id", "")),
        map_yaml_uri=str(raw.get("map_yaml_uri", "")),
        map_yaml_sha256=str(raw.get("map_yaml_sha256", "")),
        map_image_uri=str(raw.get("map_image_uri", "")),
        map_image_sha256=str(raw.get("map_image_sha256", "")),
        resolution_m=resolution,
        width_cells=width,
        height_cells=height,
        origin_x=_finite("map.origin.x", origin[0]),
        origin_y=_finite("map.origin.y", origin[1]),
        occupied_threshold=occupied,
        free_threshold=free,
        allow_unknown=allow_unknown,
        geofence=_polygon("geofence.polygon", geofence_config.get("polygon")),
        no_go_zones=tuple(
            _polygon(f"no_go_zones[{index}]", item) for index, item in enumerate(no_go_raw)
        ),
        default_xy_m=values["default_xy_m"],
        default_yaw_rad=values["default_yaw_rad"],
        min_xy_m=values["min_xy_m"],
        max_xy_m=values["max_xy_m"],
        min_yaw_rad=values["min_yaw_rad"],
        max_yaw_rad=values["max_yaw_rad"],
        minimum_obstacle_clearance_m=clearance,
        expected_motion_mode=motion_mode,
        policy_hash=sha256_document(raw),
    )


def resolve_package_uri(uri: str) -> Path:
    prefix = "package://"
    if not uri.startswith(prefix):
        return Path(uri).expanduser()
    package_path = uri[len(prefix) :]
    package, separator, relative = package_path.partition("/")
    if (
        not package
        or not separator
        or not relative
        or re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", package) is None
        or ".." in Path(relative).parts
    ):
        raise ValueError("invalid package URI")
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            root_path = Path(root)
            direct = root_path / package / relative
            if direct.exists():
                return direct
            for candidate in root_path.glob(f"**/{package}/{relative}"):
                if candidate.exists():
                    return candidate
    raise FileNotFoundError(f"unable to resolve operator map URI {uri}")


def _verify_file(path: Path, expected: str) -> None:
    digest = f"sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}"
    if digest != expected:
        raise ValueError(f"map artifact hash mismatch for {path}")


def _read_pgm(path: Path, expected_width: int, expected_height: int) -> bytes:
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index : index + 1].isspace():
            index += 1
        if index < len(data) and data[index : index + 1] == b"#":
            while index < len(data) and data[index : index + 1] not in {b"\n", b"\r"}:
                index += 1
            continue
        start = index
        while index < len(data) and not data[index : index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    if tokens[0] != b"P5":
        raise ValueError("only binary P5 PGM maps are supported")
    width, height, maximum = (int(tokens[1]), int(tokens[2]), int(tokens[3]))
    if (width, height) != (expected_width, expected_height) or maximum != 255:
        raise ValueError("PGM metadata does not match patrol policy")
    if index >= len(data) or not data[index : index + 1].isspace():
        raise ValueError("PGM header is missing its payload delimiter")
    if data[index : index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = data[index:]
    if len(pixels) != width * height:
        raise ValueError("PGM payload size does not match dimensions")
    return pixels


def _point_in_polygon(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if (
            abs(cross) <= 1e-9
            and min(x1, x2) <= x <= max(x1, x2)
            and min(y1, y2) <= y <= max(y1, y2)
        ):
            return True
        if (y1 > y) != (y2 > y):
            intersection = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection:
                inside = not inside
        previous = current
    return inside


class NavigationGoalValidator:
    def __init__(self, policy: PatrolMapPolicy | None = None) -> None:
        self.policy = policy or load_patrol_map_policy()
        self._pixels: bytes | None = None

    def _map_pixels(self) -> bytes:
        if self._pixels is None:
            yaml_path = resolve_package_uri(self.policy.map_yaml_uri)
            image_path = resolve_package_uri(self.policy.map_image_uri)
            _verify_file(yaml_path, self.policy.map_yaml_sha256)
            _verify_file(image_path, self.policy.map_image_sha256)
            self._pixels = _read_pgm(image_path, self.policy.width_cells, self.policy.height_cells)
        return self._pixels

    def validate(
        self,
        *,
        x: Any,
        y: Any,
        yaw: Any,
        frame_id: str = "map",
        route_policy_id: str = "lab-default",
        goal_tolerance_xy_m: Any | None = None,
        goal_tolerance_yaw_rad: Any | None = None,
    ) -> dict[str, Any]:
        try:
            x_value = _finite("x", x)
            y_value = _finite("y", y)
            yaw_value = _finite("yaw", yaw)
            xy_tolerance = _finite(
                "goal_tolerance_xy_m",
                self.policy.default_xy_m if goal_tolerance_xy_m is None else goal_tolerance_xy_m,
            )
            yaw_tolerance = _finite(
                "goal_tolerance_yaw_rad",
                self.policy.default_yaw_rad
                if goal_tolerance_yaw_rad is None
                else goal_tolerance_yaw_rad,
            )
        except ValueError as exc:
            return self._blocked("LIMO_GOAL_INVALID", str(exc))
        if frame_id != self.policy.frame_id or frame_id != "map":
            return self._blocked(
                "LIMO_GOAL_FRAME_INVALID", "Navigation Contract v2 only accepts map frame"
            )
        if route_policy_id != self.policy.route_policy_id:
            return self._blocked(
                "LIMO_ROUTE_POLICY_MISMATCH", "route_policy_id is not operator-approved"
            )
        if not -math.pi <= yaw_value <= math.pi:
            return self._blocked("LIMO_GOAL_INVALID", "yaw must be within [-pi, pi]")
        if not self.policy.min_xy_m <= xy_tolerance <= self.policy.max_xy_m:
            return self._blocked(
                "LIMO_GOAL_TOLERANCE_INVALID", "xy tolerance is outside operator bounds"
            )
        if not self.policy.min_yaw_rad <= yaw_tolerance <= self.policy.max_yaw_rad:
            return self._blocked(
                "LIMO_GOAL_TOLERANCE_INVALID", "yaw tolerance is outside operator bounds"
            )

        grid_x = math.floor((x_value - self.policy.origin_x) / self.policy.resolution_m)
        grid_y = math.floor((y_value - self.policy.origin_y) / self.policy.resolution_m)
        if not 0 <= grid_x < self.policy.width_cells or not 0 <= grid_y < self.policy.height_cells:
            return self._blocked("LIMO_GOAL_OUTSIDE_MAP", "goal is outside map bounds")
        if not _point_in_polygon(x_value, y_value, self.policy.geofence):
            return self._blocked("LIMO_GOAL_OUTSIDE_GEOFENCE", "goal is outside operator geofence")
        if any(_point_in_polygon(x_value, y_value, polygon) for polygon in self.policy.no_go_zones):
            return self._blocked("LIMO_GOAL_OUTSIDE_GEOFENCE", "goal is inside a no-go zone")

        try:
            pixels = self._map_pixels()
        except (FileNotFoundError, OSError, ValueError) as exc:
            return self._blocked("LIMO_MAP_ARTIFACT_INVALID", str(exc))
        radius = math.ceil(self.policy.minimum_obstacle_clearance_m / self.policy.resolution_m)
        unknown_seen = False
        occupied_seen = False
        for candidate_y in range(
            max(0, grid_y - radius), min(self.policy.height_cells, grid_y + radius + 1)
        ):
            for candidate_x in range(
                max(0, grid_x - radius), min(self.policy.width_cells, grid_x + radius + 1)
            ):
                if math.hypot(candidate_x - grid_x, candidate_y - grid_y) > radius:
                    continue
                row = self.policy.height_cells - 1 - candidate_y
                pixel = pixels[row * self.policy.width_cells + candidate_x]
                occupancy = (255 - pixel) / 255.0
                occupied_seen = occupied_seen or occupancy > self.policy.occupied_threshold
                unknown_seen = (
                    unknown_seen
                    or self.policy.free_threshold <= occupancy <= self.policy.occupied_threshold
                )
        if occupied_seen:
            return self._blocked("LIMO_GOAL_OCCUPIED", "goal violates obstacle clearance")
        if unknown_seen and not self.policy.allow_unknown:
            return self._blocked(
                "LIMO_GOAL_OCCUPIED", "goal or clearance region contains unknown cells"
            )

        return {
            "ok": True,
            "decision": "ALLOW",
            "schema_version": "limo.navigation.v2",
            "normalized_goal": {"frame_id": frame_id, "x": x_value, "y": y_value, "yaw": yaw_value},
            "goal_tolerance": {"xy_m": xy_tolerance, "yaw_rad": yaw_tolerance},
            "route_policy_id": self.policy.route_policy_id,
            "route_policy_hash": self.policy.policy_hash,
            "map_id": self.policy.map_id,
            "map_image_hash": self.policy.map_image_sha256,
            "minimum_obstacle_clearance_m": self.policy.minimum_obstacle_clearance_m,
            "expected_motion_mode": self.policy.expected_motion_mode,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    @staticmethod
    def _blocked(error_code: str, message: str) -> dict[str, Any]:
        return {
            "ok": False,
            "decision": "BLOCK",
            "schema_version": "limo.navigation.v2",
            "error_code": error_code,
            "message": message,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }
