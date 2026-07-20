#!/usr/bin/env python3

import math
import json
import os
from collections import deque
from dataclasses import dataclass

try:
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from geometry_msgs.msg import Point, PointStamped, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    rospy = None
    pc2 = None
    Point = None
    PointStamped = None
    PoseStamped = None
    Odometry = None
    Path = None
    PointCloud2 = None
    Marker = None
    MarkerArray = None


DEFAULT_WALL_LENGTH = 2.5
DEFAULT_WALL_WIDTH = 0.2
DEFAULT_CYLINDER_DIAMETER = 0.4
DEFAULT_OBSTACLE_HEIGHT = 2.0
DEFAULT_MIN_STRUCTURE_HEIGHT = 1.3


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ClusterModel:
    kind: str
    center: Point3
    length: float
    width: float
    height: float
    min_lateral: float
    max_lateral: float
    point_count: int
    fit_error: float = 999.0
    min_forward: float = 0.0
    max_forward: float = 0.0
    yaw: float = 0.0
    face_center: Point3 = None
    face_thickness: float = 0.0
    face_count: int = 1
    min_z: float = None
    max_z: float = None


@dataclass(frozen=True)
class GapCandidate:
    label: str
    point: Point3
    width: float
    wall: ClusterModel
    cylinder: ClusterModel
    left_bound: float
    right_bound: float
    passable: bool


@dataclass(frozen=True)
class GapDetection:
    found: bool
    point: Point3
    width: float
    label: str
    wall: ClusterModel = None
    cylinder: ClusterModel = None
    models: tuple = ()
    cylinders: tuple = ()
    legal_gaps: tuple = ()
    selected_gap: GapCandidate = None
    complete_match: bool = False
    raw_points: int = 0
    filtered_points: int = 0
    downsampled_points: int = 0
    cluster_count: int = 0
    wall_count: int = 0
    cylinder_count: int = 0


def _result_quality_score(result):
    if result is None or not result.found or not result.complete_match:
        return float("inf")

    score = 0.0
    if result.wall is not None:
        score += result.wall.fit_error
        score -= min(result.wall.point_count, 300) * 0.0005
    for cylinder in result.cylinders:
        score += cylinder.fit_error
        score -= min(cylinder.point_count, 200) * 0.0005
    score += abs(result.width - 1.0) * 0.1
    return score


class TaskAStructureRefiner:
    def __init__(
        self,
        refine_duration=2.0,
        min_refine_matches=8,
        stable_center_threshold=0.12,
        stable_gap_threshold=0.10,
        max_refine_duration=4.0,
    ):
        self.refine_duration = float(refine_duration)
        self.min_refine_matches = int(min_refine_matches)
        self.stable_center_threshold = float(stable_center_threshold)
        self.stable_gap_threshold = float(stable_gap_threshold)
        self.max_refine_duration = float(max_refine_duration)
        self.observations = []
        self.best_result = None
        self.first_match_time = None
        self.final_result = None
        self.publish_consumed = False

    def update(self, result, stamp):
        if self.final_result is not None:
            return self.final_result

        if result is None or not result.found or not result.complete_match:
            return self.best_result or result

        stamp = float(stamp)
        if self.first_match_time is None:
            self.first_match_time = stamp

        self.observations.append((stamp, result))
        candidate = min(
            (observation for _, observation in self.observations),
            key=_result_quality_score,
        )
        if self.best_result is None or _result_quality_score(candidate) <= _result_quality_score(self.best_result):
            self.best_result = candidate

        elapsed = stamp - self.first_match_time
        if (
            len(self.observations) >= self.min_refine_matches
            and elapsed >= self.refine_duration
            and (self._is_stable() or elapsed >= self.max_refine_duration)
        ):
            self.final_result = self.best_result

        return self.final_result or self.best_result

    def consume_publish_result(self):
        if self.final_result is None or self.publish_consumed:
            return None
        self.publish_consumed = True
        return self.final_result

    def _is_stable(self):
        if len(self.observations) < self.min_refine_matches:
            return False
        recent = [result for _, result in self.observations[-self.min_refine_matches:]]
        xs = [result.point.x for result in recent]
        ys = [result.point.y for result in recent]
        widths = [result.width for result in recent]
        center_range = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        width_range = max(widths) - min(widths)
        return (
            center_range <= self.stable_center_threshold
            and width_range <= self.stable_gap_threshold
        )


def _dot_xy(point, axis):
    return point[0] * axis[0] + point[1] * axis[1]


def _distance_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point_distance_xy(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _yaw_distance_pi_periodic(a, b):
    diff = abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)
    return min(diff, abs(math.pi - diff))


def _normalize_xy(axis):
    norm = math.hypot(axis[0], axis[1])
    if norm == 0.0:
        return (1.0, 0.0)
    return (axis[0] / norm, axis[1] / norm)


def _left_lateral_axis(forward_axis):
    forward_axis = _normalize_xy(forward_axis)
    return (-forward_axis[1], forward_axis[0])


def _model_axes(model):
    major_axis = _normalize_xy((math.cos(model.yaw), math.sin(model.yaw)))
    minor_axis = (-major_axis[1], major_axis[0])
    return major_axis, minor_axis


def _axis_with_same_direction(axis, reference_axis):
    axis = _normalize_xy(axis)
    reference_axis = _normalize_xy(reference_axis)
    if _dot_xy(axis, reference_axis) < 0.0:
        return (-axis[0], -axis[1])
    return axis


def _project_model_xy(model, axis):
    axis = _normalize_xy(axis)
    major_axis, minor_axis = _model_axes(model)
    center = _dot_xy((model.center.x, model.center.y), axis)
    half_extent = 0.5 * (
        abs(_dot_xy(major_axis, axis)) * model.length
        + abs(_dot_xy(minor_axis, axis)) * model.width
    )
    return center - half_extent, center + half_extent


def _wall_face_point(model):
    return model.face_center if model.face_center is not None else model.center


def _wall_axes_from_observer(model, forward_axis, observer_origin_xy):
    side_axis, depth_axis = _model_axes(model)
    depth_axis = _normalize_xy(depth_axis)
    forward_axis = _normalize_xy(forward_axis)
    observer_origin_xy = (float(observer_origin_xy[0]), float(observer_origin_xy[1]))

    if model.face_center is not None:
        face_to_center = (
            model.center.x - model.face_center.x,
            model.center.y - model.face_center.y,
        )
        if math.hypot(face_to_center[0], face_to_center[1]) > 1e-6:
            if _dot_xy(depth_axis, face_to_center) < 0.0:
                depth_axis = (-depth_axis[0], -depth_axis[1])
            return side_axis, depth_axis

        observer_to_face = (
            model.face_center.x - observer_origin_xy[0],
            model.face_center.y - observer_origin_xy[1],
        )
        if math.hypot(observer_to_face[0], observer_to_face[1]) > 1e-6:
            if _dot_xy(depth_axis, observer_to_face) < 0.0:
                depth_axis = (-depth_axis[0], -depth_axis[1])
            return side_axis, depth_axis

    observer_to_center = (
        model.center.x - observer_origin_xy[0],
        model.center.y - observer_origin_xy[1],
    )
    if math.hypot(observer_to_center[0], observer_to_center[1]) > 1e-6:
        if _dot_xy(depth_axis, observer_to_center) < 0.0:
            depth_axis = (-depth_axis[0], -depth_axis[1])
    else:
        depth_axis = _axis_with_same_direction(depth_axis, forward_axis)
    return side_axis, depth_axis


def _wall_near_face_point(model, depth_axis, observer_origin_xy):
    if model.face_center is not None:
        return model.face_center

    depth_axis = _normalize_xy(depth_axis)
    half_width = max(model.width, 0.0) * 0.5
    candidates = (
        Point3(
            model.center.x - depth_axis[0] * half_width,
            model.center.y - depth_axis[1] * half_width,
            model.center.z,
        ),
        Point3(
            model.center.x + depth_axis[0] * half_width,
            model.center.y + depth_axis[1] * half_width,
            model.center.z,
        ),
    )
    return min(
        candidates,
        key=lambda point: _distance_xy((point.x, point.y), observer_origin_xy),
    )


def _wall_near_face_depth(model, depth_axis, observer_origin_xy):
    face = _wall_near_face_point(model, depth_axis, observer_origin_xy)
    return _dot_xy((face.x, face.y), depth_axis)


def _model_center_along(model, axis):
    return _dot_xy((model.center.x, model.center.y), _normalize_xy(axis))


def _result_with_label(result, label):
    if result is None:
        return None

    return GapDetection(
        found=result.found,
        point=result.point,
        width=result.width,
        label=label,
        wall=result.wall,
        cylinder=result.cylinder,
        models=result.models,
        cylinders=result.cylinders,
        legal_gaps=result.legal_gaps,
        selected_gap=result.selected_gap,
        complete_match=result.complete_match,
        raw_points=result.raw_points,
        filtered_points=result.filtered_points,
        downsampled_points=result.downsampled_points,
        cluster_count=result.cluster_count,
        wall_count=result.wall_count,
        cylinder_count=result.cylinder_count,
    )


def _rejected_result(result, label):
    if result is None:
        return None

    return GapDetection(
        found=False,
        point=result.point,
        width=result.width,
        label=label,
        wall=result.wall,
        cylinder=result.cylinder,
        models=result.models,
        cylinders=result.cylinders,
        legal_gaps=result.legal_gaps,
        selected_gap=result.selected_gap,
        complete_match=result.complete_match,
        raw_points=result.raw_points,
        filtered_points=result.filtered_points,
        downsampled_points=result.downsampled_points,
        cluster_count=result.cluster_count,
        wall_count=result.wall_count,
        cylinder_count=result.cylinder_count,
    )


def _point_from_orthogonal_axes(primary_value, secondary_value, primary_axis, secondary_axis, z):
    return Point3(
        primary_axis[0] * primary_value + secondary_axis[0] * secondary_value,
        primary_axis[1] * primary_value + secondary_axis[1] * secondary_value,
        z,
    )


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _point_from_axes(lateral, forward, lateral_axis, forward_axis, z):
    return Point3(
        lateral_axis[0] * lateral + forward_axis[0] * forward,
        lateral_axis[1] * lateral + forward_axis[1] * forward,
        z,
    )


def _solve_3x3(matrix, vector):
    rows = [list(matrix[index]) + [float(vector[index])] for index in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) < 1e-9:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        pivot_value = rows[column][column]
        rows[column] = [value / pivot_value for value in rows[column]]
        for row in range(3):
            if row == column:
                continue
            factor = rows[row][column]
            rows[row] = [
                rows[row][index] - factor * rows[column][index]
                for index in range(4)
            ]
    return (rows[0][3], rows[1][3], rows[2][3])


def _fit_circle_xy(points):
    if len(points) < 3:
        return None

    sx = sy = sx2 = sy2 = sxy = 0.0
    sxr = syr = sr = 0.0
    for point in points:
        x, y = point[0], point[1]
        r2 = x * x + y * y
        sx += x
        sy += y
        sx2 += x * x
        sy2 += y * y
        sxy += x * y
        sxr += x * r2
        syr += y * r2
        sr += r2

    solution = _solve_3x3(
        (
            (sx2, sxy, sx),
            (sxy, sy2, sy),
            (sx, sy, float(len(points))),
        ),
        (-sxr, -syr, -sr),
    )
    if solution is None:
        return None

    a, b, c = solution
    center_x = -a / 2.0
    center_y = -b / 2.0
    radius_sq = center_x * center_x + center_y * center_y - c
    if radius_sq <= 0.0:
        return None

    radius = math.sqrt(radius_sq)
    error = math.sqrt(
        sum(
            (math.hypot(point[0] - center_x, point[1] - center_y) - radius) ** 2
            for point in points
        )
        / len(points)
    )
    return center_x, center_y, radius, error


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _expand_path(path):
    return os.path.expandvars(os.path.expanduser(str(path)))


def _model_to_dict(model):
    data = {
        "kind": model.kind,
        "center": {"x": model.center.x, "y": model.center.y, "z": model.center.z},
        "length": model.length,
        "width": model.width,
        "height": model.height,
        "min_lateral": model.min_lateral,
        "max_lateral": model.max_lateral,
        "point_count": model.point_count,
        "fit_error": model.fit_error,
        "min_forward": model.min_forward,
        "max_forward": model.max_forward,
        "yaw": model.yaw,
        "face_thickness": model.face_thickness,
        "face_count": model.face_count,
    }
    if model.min_z is not None and model.max_z is not None:
        data["min_z"] = model.min_z
        data["max_z"] = model.max_z
    if model.face_center is not None:
        data["face_center"] = {
            "x": model.face_center.x,
            "y": model.face_center.y,
            "z": model.face_center.z,
        }
    return data


def _model_from_dict(data):
    center = data.get("center", {})
    center_point = Point3(
        float(center.get("x", 0.0)),
        float(center.get("y", 0.0)),
        float(center.get("z", 0.0)),
    )
    height = float(data.get("height", 0.0))
    face_center = data.get("face_center")
    if face_center is not None:
        face_center = Point3(
            float(face_center.get("x", 0.0)),
            float(face_center.get("y", 0.0)),
            float(face_center.get("z", 0.0)),
        )
    min_z = data.get("min_z")
    max_z = data.get("max_z")
    if min_z is None or max_z is None:
        min_z = center_point.z - height * 0.5
        max_z = center_point.z + height * 0.5
    return ClusterModel(
        kind=str(data.get("kind", "unknown")),
        center=center_point,
        length=float(data.get("length", 0.0)),
        width=float(data.get("width", 0.0)),
        height=height,
        min_lateral=float(data.get("min_lateral", 0.0)),
        max_lateral=float(data.get("max_lateral", 0.0)),
        point_count=int(data.get("point_count", 0)),
        fit_error=float(data.get("fit_error", 999.0)),
        min_forward=float(data.get("min_forward", 0.0)),
        max_forward=float(data.get("max_forward", 0.0)),
        yaw=float(data.get("yaw", 0.0)),
        face_center=face_center,
        face_thickness=float(data.get("face_thickness", 0.0)),
        face_count=int(data.get("face_count", 1)),
        min_z=float(min_z),
        max_z=float(max_z),
    )


def _model_z_bounds(model):
    if model is not None and model.min_z is not None and model.max_z is not None:
        return float(model.min_z), float(model.max_z)
    if model is None:
        return 0.0, 0.0
    half_height = max(model.height, 0.0) * 0.5
    return model.center.z - half_height, model.center.z + half_height


def _model_summary(models, limit=4):
    if not models:
        return "none"
    parts = []
    for model in list(models)[:limit]:
        parts.append(
            "%s(l=%.2f,w=%.2f,h=%.2f,n=%d,f=%.2f..%.2f,lat=%.2f..%.2f,faces=%d)"
            % (
                model.kind,
                model.length,
                model.width,
                model.height,
                model.point_count,
                model.min_forward,
                model.max_forward,
                model.min_lateral,
                model.max_lateral,
                model.face_count,
            )
        )
    if len(models) > limit:
        parts.append("...")
    return "; ".join(parts)


def _cluster_center(cluster):
    return (
        sum(point[0] for point in cluster) / len(cluster),
        sum(point[1] for point in cluster) / len(cluster),
        sum(point[2] for point in cluster) / len(cluster),
    )


def _principal_axes_xy(points):
    if not points:
        return (1.0, 0.0), (0.0, 1.0), 0.0, 0.0

    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    cov_xx = sum((point[0] - center_x) * (point[0] - center_x) for point in points) / len(points)
    cov_yy = sum((point[1] - center_y) * (point[1] - center_y) for point in points) / len(points)
    cov_xy = sum((point[0] - center_x) * (point[1] - center_y) for point in points) / len(points)

    yaw = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    major_axis = _normalize_xy((math.cos(yaw), math.sin(yaw)))
    minor_axis = (-major_axis[1], major_axis[0])

    major_values = [_dot_xy((point[0], point[1]), major_axis) for point in points]
    minor_values = [_dot_xy((point[0], point[1]), minor_axis) for point in points]
    major_extent = max(major_values) - min(major_values)
    minor_extent = max(minor_values) - min(minor_values)

    if minor_extent > major_extent:
        major_axis, minor_axis = minor_axis, major_axis
        major_extent, minor_extent = minor_extent, major_extent

    return major_axis, minor_axis, major_extent, minor_extent


def _oriented_extent_on_axis(center, length, width, yaw, axis):
    axis = _normalize_xy(axis)
    major_axis = _normalize_xy((math.cos(yaw), math.sin(yaw)))
    minor_axis = (-major_axis[1], major_axis[0])
    center_value = _dot_xy((center.x, center.y), axis)
    half_extent = 0.5 * (
        abs(_dot_xy(major_axis, axis)) * length
        + abs(_dot_xy(minor_axis, axis)) * width
    )
    return center_value - half_extent, center_value + half_extent


def _wall_face_geometry(points, lateral_axis, forward_axis):
    if not points:
        return None

    major_axis, minor_axis, length, thickness = _principal_axes_xy(points)
    if _dot_xy(minor_axis, forward_axis) < 0.0:
        minor_axis = (-minor_axis[0], -minor_axis[1])

    major_values = [_dot_xy((point[0], point[1]), major_axis) for point in points]
    minor_values = [_dot_xy((point[0], point[1]), minor_axis) for point in points]
    z_values = [point[2] for point in points]
    center_major = (min(major_values) + max(major_values)) / 2.0
    center_minor = sum(minor_values) / len(minor_values)
    center = Point3(
        major_axis[0] * center_major + minor_axis[0] * center_minor,
        major_axis[1] * center_major + minor_axis[1] * center_minor,
        (min(z_values) + max(z_values)) / 2.0,
    )
    yaw = math.atan2(major_axis[1], major_axis[0])
    fit_error = math.sqrt(
        sum((value - center_minor) ** 2 for value in minor_values) / len(minor_values)
    )
    min_lateral, max_lateral = _oriented_extent_on_axis(
        center,
        length,
        max(thickness, 0.02),
        yaw,
        lateral_axis,
    )
    min_forward, max_forward = _oriented_extent_on_axis(
        center,
        length,
        max(thickness, 0.02),
        yaw,
        forward_axis,
    )
    return ClusterModel(
        "wall_face",
        center,
        length,
        max(thickness, 0.02),
        max(z_values) - min(z_values),
        min_lateral,
        max_lateral,
        len(points),
        fit_error,
        min_forward,
        max_forward,
        yaw,
        face_center=center,
        face_thickness=max(thickness, 0.02),
        face_count=1,
        min_z=min(z_values),
        max_z=max(z_values),
    )


def _fit_wall_face_ransac(points, lateral_axis, forward_axis, inlier_threshold, min_points):
    if len(points) < min_points:
        return None, ()

    sample_count = min(36, len(points))
    if sample_count <= 1:
        return None, ()
    sample_indices = sorted(
        set(int(round(index * (len(points) - 1) / float(sample_count - 1))) for index in range(sample_count))
    )

    best_inliers = ()
    best_score = None
    threshold_sq = inlier_threshold * inlier_threshold
    for outer_pos, first_index in enumerate(sample_indices):
        p1 = points[first_index]
        for second_index in sample_indices[outer_pos + 1:]:
            p2 = points[second_index]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            norm = math.hypot(dx, dy)
            if norm < max(0.05, inlier_threshold * 2.0):
                continue
            normal = (-dy / norm, dx / norm)
            inliers = []
            for point_index, point in enumerate(points):
                distance = (point[0] - p1[0]) * normal[0] + (point[1] - p1[1]) * normal[1]
                if distance * distance <= threshold_sq:
                    inliers.append(point_index)
            if len(inliers) < min_points:
                continue
            inlier_points = [points[index] for index in inliers]
            model = _wall_face_geometry(inlier_points, lateral_axis, forward_axis)
            if model is None:
                continue
            score = (len(inliers), model.length, -model.fit_error)
            if best_score is None or score > best_score:
                best_score = score
                best_inliers = tuple(inliers)

    if best_inliers:
        inlier_points = [points[index] for index in best_inliers]
        return _wall_face_geometry(inlier_points, lateral_axis, forward_axis), best_inliers

    model = _wall_face_geometry(points, lateral_axis, forward_axis)
    if model is None:
        return None, ()
    return model, tuple(range(len(points)))


def _refit_wall_face_from_cloud(
    model,
    points,
    lateral_axis,
    forward_axis,
    inlier_threshold,
    min_points,
):
    if model is None or model.face_center is None or not points:
        return model

    major_axis, minor_axis = _model_axes(model)
    face_center = model.face_center
    face_minor = _dot_xy((face_center.x, face_center.y), minor_axis)
    plane_tolerance = max(
        float(inlier_threshold),
        min(max(model.face_thickness, model.width, 0.0) + 0.03, 0.20),
        0.05,
    )

    face_points = [
        point
        for point in points
        if abs(_dot_xy((point[0], point[1]), minor_axis) - face_minor) <= plane_tolerance
    ]
    if len(face_points) < max(3, int(min_points)):
        return model

    refit = _wall_face_geometry(face_points, lateral_axis, forward_axis)
    if refit is None or refit.point_count < max(3, int(min_points)):
        return model

    if model.face_count != 1:
        refit = _model_with_face_count(refit, model.face_count)
    return refit


def _model_with_face_count(model, face_count):
    return ClusterModel(
        model.kind,
        model.center,
        model.length,
        model.width,
        model.height,
        model.min_lateral,
        model.max_lateral,
        model.point_count,
        model.fit_error,
        model.min_forward,
        model.max_forward,
        model.yaw,
        model.face_center,
        model.face_thickness,
        face_count,
        model.min_z,
        model.max_z,
    )


def extract_wall_face_models(
    cluster,
    lateral_axis=(1.0, 0.0),
    forward_axis=(0.0, -1.0),
    inlier_threshold=0.10,
    min_points=8,
    min_length=0.35,
    max_thickness=0.25,
    max_faces=2,
):
    remaining = list(cluster)
    faces = []
    max_faces = max(1, int(max_faces))
    min_points = max(3, int(min_points))

    for _ in range(max_faces):
        if len(remaining) < min_points:
            break
        face, inlier_indices = _fit_wall_face_ransac(
            remaining,
            lateral_axis,
            forward_axis,
            inlier_threshold,
            min_points,
        )
        if face is None:
            break
        if (
            face.point_count < min_points
            or face.length < min_length
            or face.width > max_thickness
            or face.fit_error > max(inlier_threshold, 1e-3)
        ):
            break
        faces.append(
            _refit_wall_face_from_cloud(
                face,
                remaining,
                lateral_axis,
                forward_axis,
                inlier_threshold,
                min_points,
            )
        )
        inlier_set = set(inlier_indices)
        remaining = [point for index, point in enumerate(remaining) if index not in inlier_set]

    if len(faces) <= 1:
        return faces
    return [_model_with_face_count(face, len(faces)) for face in faces]


def _model_with_kind(model, kind, fit_error=999.0):
    return ClusterModel(
        kind,
        model.center,
        model.length,
        model.width,
        model.height,
        model.min_lateral,
        model.max_lateral,
        model.point_count,
        fit_error,
        model.min_forward,
        model.max_forward,
        model.yaw,
        model.face_center,
        model.face_thickness,
        model.face_count,
        model.min_z,
        model.max_z,
    )


def filter_points(
    points,
    z_min=0.2,
    z_max=2.2,
    forward_axis=(0.0, -1.0),
    lateral_axis=(1.0, 0.0),
    origin_xy=(0.0, 0.0),
    forward_min=0.0,
    forward_max=8.0,
    lateral_min=-4.0,
    lateral_max=6.5,
):
    forward_axis = _normalize_xy(forward_axis)
    lateral_axis = _normalize_xy(lateral_axis)
    filtered = []
    for point in points:
        x, y, z = point[:3]
        if z < z_min or z > z_max:
            continue
        rel_xy = (x - origin_xy[0], y - origin_xy[1])
        forward = _dot_xy(rel_xy, forward_axis)
        lateral = _dot_xy(rel_xy, lateral_axis)
        if forward_min <= forward <= forward_max and lateral_min <= lateral <= lateral_max:
            filtered.append((float(x), float(y), float(z)))
    return filtered


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0.0:
        return list(points)

    voxels = {}
    for point in points:
        key = (
            int(math.floor(point[0] / voxel_size)),
            int(math.floor(point[1] / voxel_size)),
            int(math.floor(point[2] / voxel_size)),
        )
        if key not in voxels:
            voxels[key] = [0.0, 0.0, 0.0, 0]
        voxel = voxels[key]
        voxel[0] += point[0]
        voxel[1] += point[1]
        voxel[2] += point[2]
        voxel[3] += 1

    return [
        (value[0] / value[3], value[1] / value[3], value[2] / value[3])
        for value in voxels.values()
    ]


def euclidean_clusters(points, tolerance=0.45, min_points=8, max_points=100000):
    if not points:
        return []

    cell_size = tolerance
    grid = {}
    for index, point in enumerate(points):
        key = (
            int(math.floor(point[0] / cell_size)),
            int(math.floor(point[1] / cell_size)),
            int(math.floor(point[2] / cell_size)),
        )
        grid.setdefault(key, []).append(index)

    visited = [False] * len(points)
    clusters = []
    tolerance_sq = tolerance * tolerance

    def nearby_indices(point):
        key = (
            int(math.floor(point[0] / cell_size)),
            int(math.floor(point[1] / cell_size)),
            int(math.floor(point[2] / cell_size)),
        )
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    yield from grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ())

    for start_index in range(len(points)):
        if visited[start_index]:
            continue

        visited[start_index] = True
        queue = deque([start_index])
        cluster_indices = []

        while queue:
            current = queue.popleft()
            cluster_indices.append(current)
            current_point = points[current]

            for index in nearby_indices(current_point):
                if visited[index]:
                    continue
                candidate = points[index]
                dx = current_point[0] - candidate[0]
                dy = current_point[1] - candidate[1]
                dz = current_point[2] - candidate[2]
                if dx * dx + dy * dy + dz * dz <= tolerance_sq:
                    visited[index] = True
                    queue.append(index)

        if min_points <= len(cluster_indices) <= max_points:
            clusters.append([points[index] for index in cluster_indices])

    return clusters


def classify_cluster(cluster, lateral_axis=(1.0, 0.0), forward_axis=(0.0, -1.0)):
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    zs = [point[2] for point in cluster]
    min_z, max_z = min(zs), max(zs)

    height = max_z - min_z

    lateral_axis = _normalize_xy(lateral_axis)
    forward_axis = _normalize_xy(forward_axis)
    laterals = [_dot_xy((point[0], point[1]), lateral_axis) for point in cluster]
    forwards = [_dot_xy((point[0], point[1]), forward_axis) for point in cluster]
    min_lateral = min(laterals)
    max_lateral = max(laterals)
    min_forward = min(forwards)
    max_forward = max(forwards)
    lateral_extent = max_lateral - min_lateral
    forward_extent = max_forward - min_forward
    major_axis, minor_axis, footprint_long, footprint_short = _principal_axes_xy(cluster)
    major_values = [_dot_xy((point[0], point[1]), major_axis) for point in cluster]
    minor_values = [_dot_xy((point[0], point[1]), minor_axis) for point in cluster]
    center_major = (min(major_values) + max(major_values)) / 2.0
    center_minor = (min(minor_values) + max(minor_values)) / 2.0
    yaw = math.atan2(major_axis[1], major_axis[0])

    center = Point3(
        major_axis[0] * center_major + minor_axis[0] * center_minor,
        major_axis[1] * center_major + minor_axis[1] * center_minor,
        (min_z + max_z) / 2.0,
    )

    model = ClusterModel(
        kind="unknown",
        center=center,
        length=footprint_long,
        width=footprint_short,
        height=height,
        min_lateral=min_lateral,
        max_lateral=max_lateral,
        point_count=len(cluster),
        min_forward=min_forward,
        max_forward=max_forward,
        yaw=yaw,
        min_z=min_z,
        max_z=max_z,
    )

    circle = _fit_circle_xy(cluster)
    if circle is not None:
        circle_x, circle_y, radius, circle_error = circle
        if (
            0.10 <= radius <= 0.35
            and lateral_extent <= 0.90
            and forward_extent <= 0.90
            and height >= 1.3
        ):
            center_lateral = _dot_xy((circle_x, circle_y), lateral_axis)
            center_forward = _dot_xy((circle_x, circle_y), forward_axis)
            circle_center = Point3(circle_x, circle_y, (min_z + max_z) / 2.0)
            diameter = 2.0 * radius
            return ClusterModel(
                "cylinder",
                circle_center,
                diameter,
                diameter,
                height,
                center_lateral - radius,
                center_lateral + radius,
                len(cluster),
                circle_error,
                center_forward - radius,
                center_forward + radius,
                yaw,
                min_z=min_z,
                max_z=max_z,
            )

    if (
        0.10 <= footprint_short <= 0.75
        and 0.25 <= footprint_long <= 0.85
        and height >= 1.3
    ):
        return ClusterModel(
            "cylinder",
            center,
            footprint_long,
            footprint_short,
            height,
            min_lateral,
            max_lateral,
            len(cluster),
            999.0,
            min_forward,
            max_forward,
            yaw,
            min_z=min_z,
            max_z=max_z,
        )

    return model


def classify_cluster_models(
    cluster,
    lateral_axis=(1.0, 0.0),
    forward_axis=(0.0, -1.0),
    wall_face_enabled=True,
    wall_face_inlier_threshold=0.10,
    wall_face_min_points=8,
    wall_face_min_length=0.35,
    wall_face_max_thickness=0.25,
    wall_max_faces=2,
):
    base_model = classify_cluster(
        cluster,
        lateral_axis=lateral_axis,
        forward_axis=forward_axis,
    )
    if base_model.kind == "cylinder" or not wall_face_enabled:
        return [base_model]

    face_models = extract_wall_face_models(
        cluster,
        lateral_axis=lateral_axis,
        forward_axis=forward_axis,
        inlier_threshold=wall_face_inlier_threshold,
        min_points=wall_face_min_points,
        min_length=wall_face_min_length,
        max_thickness=wall_face_max_thickness,
        max_faces=wall_max_faces,
    )
    if face_models:
        return face_models

    return [base_model]


def _dimension_error(value, expected):
    if expected <= 0.0:
        return abs(value)
    return abs(value - expected) / expected


def _wall_candidate_score(model, wall_length, wall_width, obstacle_height, min_structure_height):
    return (
        _dimension_error(model.length, wall_length)
        + _dimension_error(model.width, wall_width)
        + 0.4 * _dimension_error(max(model.height, min_structure_height), obstacle_height)
        - min(model.point_count, 300) * 0.0005
    )


def _cylinder_candidate_score(model, cylinder_diameter, obstacle_height, min_structure_height):
    diameter = (model.length + model.width) / 2.0
    roundness_error = abs(model.length - model.width) / max(diameter, 1e-3)
    return (
        _dimension_error(diameter, cylinder_diameter)
        + 0.5 * roundness_error
        + 0.4 * _dimension_error(max(model.height, min_structure_height), obstacle_height)
        - min(model.point_count, 200) * 0.0005
    )


def _cylinder_wall_priority_score(cylinder, wall, expected_gap, forward_window):
    cylinder_forward = (cylinder.min_forward + cylinder.max_forward) / 2.0
    wall_forward = (wall.min_forward + wall.max_forward) / 2.0
    forward_error = abs(cylinder_forward - wall_forward)

    if cylinder.max_lateral <= wall.min_lateral:
        lateral_gap = wall.min_lateral - cylinder.max_lateral
    elif cylinder.min_lateral >= wall.max_lateral:
        lateral_gap = cylinder.min_lateral - wall.max_lateral
    else:
        lateral_gap = 0.0

    forward_penalty = max(0.0, forward_error - forward_window) * 20.0
    return (
        forward_error * 4.0
        + abs(lateral_gap - expected_gap)
        + forward_penalty
        + cylinder.fit_error * 0.25
    )


def _gap_width_is_reasonable(width, safe_radius, predicted_gap_width, gap_width_min, gap_width_max):
    min_width = max(float(gap_width_min), min(2.0 * safe_radius, predicted_gap_width * 0.45))
    if gap_width_max > 0.0:
        max_width = max(float(gap_width_max), min_width)
    else:
        max_width = max(predicted_gap_width * 2.0, min_width + 0.20)
    return min_width <= width <= max_width


def _effective_gap_width(raw_width, overlap_depth, predicted_gap_width):
    if raw_width > 0.0:
        return raw_width
    return max(0.0, float(predicted_gap_width) - max(0.0, overlap_depth))


def _inferred_wall_candidate(
    model,
    forward_axis,
    lateral_axis,
    observer_origin_xy,
    wall_length,
    wall_width,
    obstacle_height,
    min_structure_height,
    wall_length_min,
    wall_length_max,
    wall_width_min,
    wall_width_max,
    wall_aspect_min,
    wall_face_min_length,
    wall_face_max_thickness,
    wall_infer_length_margin,
    wall_infer_height_margin,
    wall_infer_depth_from_face,
    wall_size_from_cloud,
):
    height_ok = model.height >= min_structure_height
    if not height_ok:
        return None

    aspect = model.length / max(model.width, 1e-3)
    box_wall_shape = (
        wall_length_min <= model.length <= wall_length_max
        and wall_width_min <= model.width <= wall_width_max
        and aspect >= wall_aspect_min
    )
    face_like = (
        model.kind in ("wall_face", "wall")
        and model.face_center is not None
        and model.length >= wall_face_min_length
        and model.face_thickness <= wall_face_max_thickness
    )

    if not box_wall_shape and not face_like:
        return None

    if face_like:
        major_axis, depth_axis = _wall_axes_from_observer(model, forward_axis, observer_origin_xy)
        face_center = model.face_center
        if wall_size_from_cloud:
            length_margin = min(
                max(wall_infer_length_margin, 0.0),
                max(0.03, model.length * 0.10),
            )
            inferred_length = max(0.05, model.length + length_margin)
        else:
            inferred_length = min(
                wall_length_max,
                max(wall_length_min, wall_length, model.length + wall_infer_length_margin),
            )
        inferred_width = min(
            wall_width_max,
            max(wall_width_min, wall_width),
        )
        inferred_yaw = math.atan2(major_axis[1], major_axis[0])
        min_z, max_z = _model_z_bounds(model)
        if wall_size_from_cloud:
            height_margin = min(
                max(wall_infer_height_margin, 0.0),
                max(0.02, model.height * 0.10),
            )
            min_z -= height_margin
            max_z += height_margin
            height = max(0.05, max_z - min_z)
        else:
            height = max(model.height, min_structure_height)
            center_half_height = height * 0.5
            min_z = face_center.z - center_half_height
            max_z = face_center.z + center_half_height
        center_z = (min_z + max_z) / 2.0
        center = Point3(face_center.x, face_center.y, center_z)
        if wall_infer_depth_from_face:
            center = Point3(
                face_center.x + depth_axis[0] * inferred_width * 0.5,
                face_center.y + depth_axis[1] * inferred_width * 0.5,
                center_z,
            )
        min_lateral, max_lateral = _oriented_extent_on_axis(
            center,
            inferred_length,
            inferred_width,
            inferred_yaw,
            lateral_axis,
        )
        min_forward, max_forward = _oriented_extent_on_axis(
            center,
            inferred_length,
            inferred_width,
            inferred_yaw,
            forward_axis,
        )
        face_penalty = (
            model.fit_error * 2.0
            + max(0.0, model.face_thickness - wall_face_max_thickness * 0.5)
            + max(0.0, wall_face_min_length - model.length)
        )
        near_face_distance = _distance_xy((face_center.x, face_center.y), observer_origin_xy)
        score_wall_length = inferred_length if wall_size_from_cloud else wall_length
        score_obstacle_height = height if wall_size_from_cloud else obstacle_height
        score = (
            _wall_candidate_score(
                ClusterModel(
                    "wall",
                    center,
                    inferred_length,
                    inferred_width,
                    height,
                    min_lateral,
                    max_lateral,
                    model.point_count,
                    0.0,
                    min_forward,
                    max_forward,
                    inferred_yaw,
                    face_center,
                    model.face_thickness,
                    model.face_count,
                    min_z,
                    max_z,
                ),
                score_wall_length,
                wall_width,
                score_obstacle_height,
                min_structure_height,
            )
            + face_penalty
            + near_face_distance * 0.02
            - min(model.point_count, 300) * 0.0005
        )
        return ClusterModel(
            "wall",
            center,
            inferred_length,
            inferred_width,
            height,
            min_lateral,
            max_lateral,
            model.point_count,
            score,
            min_forward,
            max_forward,
            inferred_yaw,
            face_center,
            model.face_thickness,
            model.face_count,
            min_z,
            max_z,
        )

    error = _wall_candidate_score(
        model,
        wall_length,
        wall_width,
        obstacle_height,
        min_structure_height,
    )
    return ClusterModel(
        "wall",
        model.center,
        model.length,
        model.width,
        model.height,
        model.min_lateral,
        model.max_lateral,
        model.point_count,
        error,
        model.min_forward,
        model.max_forward,
        model.yaw,
        model.face_center,
        model.face_thickness,
        model.face_count,
        model.min_z,
        model.max_z,
    )


def match_task_a_structure(
    models,
    safe_radius=0.20,
    forward_axis=(0.0, -1.0),
    lateral_axis=(1.0, 0.0),
    observer_origin_xy=(0.0, 0.0),
    wall_length=DEFAULT_WALL_LENGTH,
    wall_width=DEFAULT_WALL_WIDTH,
    cylinder_diameter=DEFAULT_CYLINDER_DIAMETER,
    obstacle_height=DEFAULT_OBSTACLE_HEIGHT,
    min_structure_height=DEFAULT_MIN_STRUCTURE_HEIGHT,
    cylinder_forward_window=0.65,
    predicted_gap_width=1.0,
    gap_width_min=0.0,
    gap_width_max=0.0,
    require_opposite_cylinders=True,
    cylinder_wall_lateral_overlap=0.20,
    wall_length_min=1.6,
    wall_length_max=3.4,
    wall_width_min=0.06,
    wall_width_max=0.65,
    wall_aspect_min=3.5,
    cylinder_length_min=0.18,
    cylinder_length_max=0.90,
    cylinder_width_min=0.10,
    cylinder_width_max=0.80,
    cylinder_aspect_max=3.0,
    wall_face_min_length=0.35,
    wall_face_max_thickness=0.25,
    wall_infer_length_margin=0.20,
    wall_infer_height_margin=0.05,
    wall_infer_depth_from_face=True,
    wall_size_from_cloud=True,
    select_nearest_gap=True,
):
    forward_axis = _normalize_xy(forward_axis)
    lateral_axis = _normalize_xy(lateral_axis)
    observer_origin_xy = (float(observer_origin_xy[0]), float(observer_origin_xy[1]))

    wall_candidates = []
    cylinder_candidates = []
    for model in models:
        height_ok = model.height >= min_structure_height
        wall_candidate = _inferred_wall_candidate(
            model,
            forward_axis,
            lateral_axis,
            observer_origin_xy,
            wall_length,
            wall_width,
            obstacle_height,
            min_structure_height,
            wall_length_min,
            wall_length_max,
            wall_width_min,
            wall_width_max,
            wall_aspect_min,
            wall_face_min_length,
            wall_face_max_thickness,
            wall_infer_length_margin,
            wall_infer_height_margin,
            wall_infer_depth_from_face,
            wall_size_from_cloud,
        )
        cylinder_shape = (
            cylinder_length_min <= model.length <= cylinder_length_max
            and cylinder_width_min <= model.width <= cylinder_width_max
            and model.length / max(model.width, 1e-3) <= cylinder_aspect_max
            and height_ok
        )

        if wall_candidate is not None:
            wall_candidates.append(wall_candidate)

        if cylinder_shape:
            error = _cylinder_candidate_score(
                model,
                cylinder_diameter,
                obstacle_height,
                min_structure_height,
            )
            cylinder_candidates.append(
                ClusterModel(
                    "cylinder",
                    model.center,
                    model.length,
                    model.width,
                    model.height,
                    model.min_lateral,
                    model.max_lateral,
                    model.point_count,
                    error,
                    model.min_forward,
                    model.max_forward,
                    model.yaw,
                )
            )

    if not wall_candidates or len(cylinder_candidates) < 2:
        return None, tuple(wall_candidates), tuple(cylinder_candidates), ()

    def wall_candidate_priority(model):
        _, depth_axis = _wall_axes_from_observer(model, forward_axis, observer_origin_xy)
        near_face = _wall_near_face_point(model, depth_axis, observer_origin_xy)
        return (
            _distance_xy((near_face.x, near_face.y), observer_origin_xy),
            model.fit_error,
            -model.point_count,
        )

    wall_candidates = sorted(wall_candidates, key=wall_candidate_priority)
    best_complete = None
    best_complete_score = None
    best_partial = None
    best_partial_score = None

    for wall in wall_candidates:
        wall_side_axis, wall_depth_axis = _wall_axes_from_observer(
            wall,
            forward_axis,
            observer_origin_xy,
        )
        wall_side_min, wall_side_max = _project_model_xy(wall, wall_side_axis)
        wall_match_depth = _wall_near_face_depth(wall, wall_depth_axis, observer_origin_xy)

        cylinder_geometry = {}
        left_candidates = []
        right_candidates = []
        for cylinder in cylinder_candidates:
            side_min, side_max = _project_model_xy(cylinder, wall_side_axis)
            depth_min, depth_max = _project_model_xy(cylinder, wall_depth_axis)
            side_center = (side_min + side_max) / 2.0
            depth_center = (depth_min + depth_max) / 2.0
            depth_error = abs(depth_center - wall_match_depth)
            cylinder_geometry[cylinder] = (
                side_min,
                side_max,
                side_center,
                depth_min,
                depth_max,
                depth_center,
                depth_error,
            )
            if side_center < (wall_side_min + wall_side_max) / 2.0 and side_max <= wall_side_min + cylinder_wall_lateral_overlap:
                left_candidates.append(cylinder)
            elif side_center >= (wall_side_min + wall_side_max) / 2.0 and side_min >= wall_side_max - cylinder_wall_lateral_overlap:
                right_candidates.append(cylinder)

        left_wall_side_candidates = [
            c for c in left_candidates
            if cylinder_geometry[c][6] <= cylinder_forward_window
        ]
        right_wall_side_candidates = [
            c for c in right_candidates
            if cylinder_geometry[c][6] <= cylinder_forward_window
        ]
        if require_opposite_cylinders:
            left_candidates = left_wall_side_candidates
            right_candidates = right_wall_side_candidates
        elif left_wall_side_candidates:
            left_candidates = left_wall_side_candidates
        if not require_opposite_cylinders and right_wall_side_candidates:
            right_candidates = right_wall_side_candidates

        selected_cylinders = []
        legal_gaps = []

        def gap_z_for(cylinder):
            return (wall.center.z + cylinder.center.z) / 2.0

        def candidate_priority(cylinder, gap_width):
            return (
                cylinder_geometry[cylinder][6] * 4.0
                + abs(gap_width - predicted_gap_width)
                + max(0.0, cylinder_geometry[cylinder][6] - cylinder_forward_window) * 20.0
                + cylinder.fit_error * 0.25
                - min(cylinder.point_count, 200) * 0.0005
            )

        if left_candidates:
            cylinder = min(
                left_candidates,
                key=lambda model: candidate_priority(
                    model,
                    wall_side_min - cylinder_geometry[model][1],
                ),
            )
            raw_width = wall_side_min - cylinder_geometry[cylinder][1]
            width = _effective_gap_width(
                raw_width,
                cylinder_geometry[cylinder][1] - wall_side_min,
                predicted_gap_width,
            )
            left_bound = cylinder_geometry[cylinder][1]
            right_bound = wall_side_min
            point = _point_from_orthogonal_axes(
                (left_bound + right_bound) / 2.0,
                wall_match_depth,
                wall_side_axis,
                wall_depth_axis,
                gap_z_for(cylinder),
            )
            legal_gaps.append(
                GapCandidate(
                    "left_wall_pillar_gap",
                    point,
                    width,
                    wall,
                    cylinder,
                    left_bound,
                    right_bound,
                    _gap_width_is_reasonable(
                        width,
                        safe_radius,
                        predicted_gap_width,
                        gap_width_min,
                        gap_width_max,
                    ),
                )
            )
            selected_cylinders.append(cylinder)

        if right_candidates:
            cylinder = min(
                right_candidates,
                key=lambda model: candidate_priority(
                    model,
                    cylinder_geometry[model][0] - wall_side_max,
                ),
            )
            raw_width = cylinder_geometry[cylinder][0] - wall_side_max
            width = _effective_gap_width(
                raw_width,
                wall_side_max - cylinder_geometry[cylinder][0],
                predicted_gap_width,
            )
            left_bound = wall_side_max
            right_bound = cylinder_geometry[cylinder][0]
            point = _point_from_orthogonal_axes(
                (left_bound + right_bound) / 2.0,
                wall_match_depth,
                wall_side_axis,
                wall_depth_axis,
                gap_z_for(cylinder),
            )
            legal_gaps.append(
                GapCandidate(
                    "right_wall_pillar_gap",
                    point,
                    width,
                    wall,
                    cylinder,
                    left_bound,
                    right_bound,
                    _gap_width_is_reasonable(
                        width,
                        safe_radius,
                        predicted_gap_width,
                        gap_width_min,
                        gap_width_max,
                    ),
                )
            )
            selected_cylinders.append(cylinder)

        if len(selected_cylinders) < 2:
            selected_cylinders = sorted(
                cylinder_candidates,
                key=lambda model: _cylinder_wall_priority_score(
                    model,
                    wall,
                    predicted_gap_width,
                    cylinder_forward_window,
                ),
            )[:2]

        partial_score = (
            wall.fit_error
            + sum(cylinder_geometry[c][6] for c in selected_cylinders if c in cylinder_geometry)
            + abs(len(legal_gaps) - 2) * 5.0
        )
        if best_partial_score is None or partial_score < best_partial_score:
            best_partial_score = partial_score
            best_partial = (None, (wall,), tuple(selected_cylinders), tuple(legal_gaps))

        passable_gaps = [gap for gap in legal_gaps if gap.passable]
        complete_match = (
            len(selected_cylinders) >= 2
            and len(legal_gaps) >= 2
            and len(passable_gaps) >= 2
        )
        if not complete_match:
            continue

        if not passable_gaps:
            continue

        if select_nearest_gap:
            passable_gaps.sort(
                key=lambda gap: (
                    _distance_xy((gap.point.x, gap.point.y), observer_origin_xy),
                    -gap.width,
                )
            )
        else:
            passable_gaps.sort(key=lambda gap: (0 if gap.label.startswith("left") else 1, -gap.width))
        selected_gap = passable_gaps[0]
        complete_score = (
            wall.fit_error
            + abs(selected_gap.width - predicted_gap_width)
            + (
                _distance_xy((selected_gap.point.x, selected_gap.point.y), observer_origin_xy) * 0.05
                if select_nearest_gap
                else 0.0
            )
            + sum(abs(gap.width - predicted_gap_width) for gap in legal_gaps) * 0.2
            + sum(cylinder_geometry[c][6] for c in selected_cylinders if c in cylinder_geometry)
            - min(wall.point_count, 300) * 0.0005
        )
        if best_complete_score is None or complete_score < best_complete_score:
            best_complete_score = complete_score
            best_complete = (selected_gap, (wall,), tuple(selected_cylinders), tuple(legal_gaps))

    if best_complete is not None:
        return best_complete
    return best_partial


def detect_task_a_gap(
    points,
    forward_axis=(0.0, -1.0),
    lateral_axis=(1.0, 0.0),
    roi_forward_axis=None,
    roi_lateral_axis=None,
    roi_origin=(0.0, 0.0),
    observer_origin_xy=None,
    safe_radius=0.20,
    z_min=0.2,
    z_max=2.2,
    forward_min=0.0,
    forward_max=8.0,
    lateral_min=-4.0,
    lateral_max=6.5,
    voxel_size=0.08,
    cluster_tolerance=0.5,
    min_cluster_points=8,
    cylinder_forward_window=0.65,
    predicted_gap_width=1.0,
    gap_width_min=0.0,
    gap_width_max=0.0,
    require_opposite_cylinders=True,
    cylinder_wall_lateral_overlap=0.20,
    wall_length=DEFAULT_WALL_LENGTH,
    wall_width=DEFAULT_WALL_WIDTH,
    cylinder_diameter=DEFAULT_CYLINDER_DIAMETER,
    obstacle_height=DEFAULT_OBSTACLE_HEIGHT,
    min_structure_height=DEFAULT_MIN_STRUCTURE_HEIGHT,
    wall_length_min=1.6,
    wall_length_max=3.4,
    wall_width_min=0.06,
    wall_width_max=0.65,
    wall_aspect_min=3.5,
    cylinder_length_min=0.18,
    cylinder_length_max=0.90,
    cylinder_width_min=0.10,
    cylinder_width_max=0.80,
    cylinder_aspect_max=3.0,
    wall_face_enabled=True,
    wall_face_inlier_threshold=0.10,
    wall_face_min_points=8,
    wall_face_min_length=0.35,
    wall_face_max_thickness=0.25,
    wall_max_faces=2,
    wall_infer_length_margin=0.20,
    wall_infer_height_margin=0.05,
    wall_infer_depth_from_face=True,
    wall_size_from_cloud=True,
    select_nearest_gap=True,
):
    if roi_forward_axis is None:
        roi_forward_axis = forward_axis
    if roi_lateral_axis is None:
        roi_lateral_axis = lateral_axis
    if observer_origin_xy is None:
        observer_origin_xy = roi_origin

    filtered = filter_points(
        points,
        z_min=z_min,
        z_max=z_max,
        forward_axis=roi_forward_axis,
        lateral_axis=roi_lateral_axis,
        origin_xy=roi_origin,
        forward_min=forward_min,
        forward_max=forward_max,
        lateral_min=lateral_min,
        lateral_max=lateral_max,
    )
    downsampled = voxel_downsample(filtered, voxel_size)
    clusters = euclidean_clusters(
        downsampled,
        tolerance=cluster_tolerance,
        min_points=min_cluster_points,
    )
    models = []
    for cluster in clusters:
        models.extend(classify_cluster_models(
            cluster,
            lateral_axis=lateral_axis,
            forward_axis=forward_axis,
            wall_face_enabled=wall_face_enabled,
            wall_face_inlier_threshold=wall_face_inlier_threshold,
            wall_face_min_points=wall_face_min_points,
            wall_face_min_length=wall_face_min_length,
            wall_face_max_thickness=wall_face_max_thickness,
            wall_max_faces=wall_max_faces,
        ))
    selected_gap, walls, cylinders, legal_gaps = match_task_a_structure(
        models,
        safe_radius=safe_radius,
        forward_axis=forward_axis,
        lateral_axis=lateral_axis,
        observer_origin_xy=observer_origin_xy,
        wall_length=wall_length,
        wall_width=wall_width,
        cylinder_diameter=cylinder_diameter,
        obstacle_height=obstacle_height,
        min_structure_height=min_structure_height,
        cylinder_forward_window=cylinder_forward_window,
        predicted_gap_width=predicted_gap_width,
        gap_width_min=gap_width_min,
        gap_width_max=gap_width_max,
        require_opposite_cylinders=require_opposite_cylinders,
        cylinder_wall_lateral_overlap=cylinder_wall_lateral_overlap,
        wall_length_min=wall_length_min,
        wall_length_max=wall_length_max,
        wall_width_min=wall_width_min,
        wall_width_max=wall_width_max,
        wall_aspect_min=wall_aspect_min,
        cylinder_length_min=cylinder_length_min,
        cylinder_length_max=cylinder_length_max,
        cylinder_width_min=cylinder_width_min,
        cylinder_width_max=cylinder_width_max,
        cylinder_aspect_max=cylinder_aspect_max,
        wall_face_min_length=wall_face_min_length,
        wall_face_max_thickness=wall_face_max_thickness,
        wall_infer_length_margin=wall_infer_length_margin,
        wall_infer_height_margin=wall_infer_height_margin,
        wall_infer_depth_from_face=wall_infer_depth_from_face,
        wall_size_from_cloud=wall_size_from_cloud,
        select_nearest_gap=select_nearest_gap,
    )
    stats = {
        "raw_points": len(points),
        "filtered_points": len(filtered),
        "downsampled_points": len(downsampled),
        "cluster_count": len(clusters),
        "wall_count": len(walls),
        "cylinder_count": len(cylinders),
    }
    if not walls or len(cylinders) < 2:
        wall = walls[0] if walls else None
        label = (
            "wall_found_need_two_cylinders_match"
            if wall is not None
            else "need_complete_wall_and_two_cylinders_match"
        )
        return GapDetection(
            False,
            Point3(0.0, 0.0, 0.0),
            0.0,
            label,
            wall,
            models=tuple(models),
            cylinders=tuple(cylinders),
            legal_gaps=tuple(legal_gaps),
            **stats,
        )

    wall = walls[0]
    if selected_gap is None:
        return GapDetection(
            False,
            Point3(0.0, 0.0, 0.0),
            0.0,
            "no_safe_complete_wall_pillar_gap",
            wall,
            models=tuple(models),
            cylinders=tuple(cylinders),
            legal_gaps=tuple(legal_gaps),
            complete_match=len(cylinders) >= 2 and len(legal_gaps) >= 2,
            **stats,
        )

    return GapDetection(
        True,
        selected_gap.point,
        selected_gap.width,
        selected_gap.label,
        wall,
        selected_gap.cylinder,
        tuple(models),
        tuple(cylinders),
        tuple(legal_gaps),
        selected_gap,
        True,
        **stats,
    )


def _set_color(marker, rgba):
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def _marker_point(point):
    msg = Point()
    msg.x = point.x
    msg.y = point.y
    msg.z = point.z
    return msg


def _set_marker_yaw(marker, yaw):
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = math.sin(yaw / 2.0)
    marker.pose.orientation.w = math.cos(yaw / 2.0)


def build_task_a_guide_path(
    result,
    final_goal,
    frame_id="map",
    stamp=None,
    forward_axis=(0.0, -1.0),
    corridor_margin=0.7,
    append_final_goal=False,
):
    if Path is None or result is None or not result.found:
        return None

    forward_axis = _normalize_xy(forward_axis)
    if result.wall is not None:
        _, wall_depth_axis = _wall_axes_from_observer(result.wall, forward_axis, (0.0, 0.0))
        forward_axis = wall_depth_axis
    gap = result.point
    entry = Point3(
        gap.x - forward_axis[0] * corridor_margin,
        gap.y - forward_axis[1] * corridor_margin,
        gap.z,
    )
    exit_point = Point3(
        gap.x + forward_axis[0] * corridor_margin,
        gap.y + forward_axis[1] * corridor_margin,
        gap.z,
    )

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp if stamp is not None else rospy.Time.now()
    path_points = [entry, gap, exit_point]
    if append_final_goal:
        path_points.append(Point3(*final_goal))
    for point in path_points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _model_marker(model, marker_id, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "task_a_clusters"
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.pose.position.x = model.center.x
    marker.pose.position.y = model.center.y
    marker.pose.position.z = model.center.z
    _set_marker_yaw(marker, model.yaw)

    if model.kind == "wall":
        marker.ns = "task_a_structure"
        marker.type = Marker.CUBE
        marker.scale.x = max(model.length, 0.05)
        marker.scale.y = max(model.width, 0.05)
        marker.scale.z = max(model.height, 0.05)
        _set_color(marker, (0.90, 0.55, 0.10, 0.70))
    elif model.kind == "cylinder":
        marker.ns = "task_a_structure"
        marker.type = Marker.CYLINDER
        marker.scale.x = max(model.length, 0.05)
        marker.scale.y = max(model.width, 0.05)
        marker.scale.z = max(model.height, 0.05)
        _set_color(marker, (0.95, 0.05, 0.05, 0.80))
    else:
        marker.type = Marker.CUBE
        marker.scale.x = max(model.length, 0.05)
        marker.scale.y = max(model.width, 0.05)
        marker.scale.z = max(model.height, 0.05)
        _set_color(marker, (0.45, 0.45, 0.45, 0.25))

    return marker


def _wall_face_marker(model, marker_id, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "task_a_wall_face"
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.type = Marker.CUBE
    face_center = model.face_center if model.face_center is not None else model.center
    marker.pose.position.x = face_center.x
    marker.pose.position.y = face_center.y
    marker.pose.position.z = face_center.z
    _set_marker_yaw(marker, model.yaw)
    marker.scale.x = max(model.length, 0.05)
    marker.scale.y = max(min(model.face_thickness, 0.08), 0.03)
    marker.scale.z = max(model.height, 0.05)
    _set_color(marker, (0.0, 0.95, 1.0, 0.65))
    return marker


def make_task_a_markers(result, frame_id="world", stamp=None):
    if MarkerArray is None:
        return None

    if stamp is None:
        stamp = rospy.Time.now()

    markers = MarkerArray()
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    marker_id = 1
    matched_models = []
    if result.wall is not None:
        matched_models.append(result.wall)
    matched_models.extend(result.cylinders)
    if matched_models:
        display_models = matched_models
    else:
        display_models = result.models

    for model in display_models:
        markers.markers.append(_model_marker(model, marker_id, frame_id, stamp))
        marker_id += 1
        if model.kind == "wall":
            markers.markers.append(_wall_face_marker(model, marker_id, frame_id, stamp))
            marker_id += 1

    for gap_candidate in result.legal_gaps:
        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "task_a_channels"
        line.id = marker_id
        marker_id += 1
        line.action = Marker.ADD
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.04
        if gap_candidate.passable:
            _set_color(line, (0.0, 0.75, 1.0, 0.85))
        else:
            _set_color(line, (1.0, 0.0, 0.0, 0.75))
        line.points.append(_marker_point(gap_candidate.cylinder.center))
        line.points.append(_marker_point(gap_candidate.point))
        line.points.append(_marker_point(_wall_face_point(gap_candidate.wall)))
        markers.markers.append(line)

        gap_label = Marker()
        gap_label.header.frame_id = frame_id
        gap_label.header.stamp = stamp
        gap_label.ns = "task_a_channels"
        gap_label.id = marker_id
        marker_id += 1
        gap_label.action = Marker.ADD
        gap_label.type = Marker.TEXT_VIEW_FACING
        gap_label.pose.position.x = gap_candidate.point.x
        gap_label.pose.position.y = gap_candidate.point.y
        gap_label.pose.position.z = gap_candidate.point.z + 0.45
        gap_label.pose.orientation.w = 1.0
        gap_label.scale.z = 0.25
        gap_label.text = "%s %.2fm" % (gap_candidate.label, gap_candidate.width)
        if gap_candidate == result.selected_gap:
            _set_color(gap_label, (0.0, 1.0, 0.20, 1.0))
        else:
            _set_color(gap_label, (0.0, 0.75, 1.0, 0.85))
        markers.markers.append(gap_label)

    if result.found:
        gap = Marker()
        gap.header.frame_id = frame_id
        gap.header.stamp = stamp
        gap.ns = "task_a_gap"
        gap.id = marker_id
        gap.action = Marker.ADD
        gap.type = Marker.SPHERE
        gap.pose.position.x = result.point.x
        gap.pose.position.y = result.point.y
        gap.pose.position.z = result.point.z
        gap.pose.orientation.w = 1.0
        gap.scale.x = 0.28
        gap.scale.y = 0.28
        gap.scale.z = 0.28
        _set_color(gap, (0.0, 1.0, 0.20, 1.0))
        markers.markers.append(gap)
        marker_id += 1

        if result.wall is not None and result.cylinder is not None:
            line = Marker()
            line.header.frame_id = frame_id
            line.header.stamp = stamp
            line.ns = "task_a_selected_channel"
            line.id = marker_id
            line.action = Marker.ADD
            line.type = Marker.LINE_STRIP
            line.scale.x = 0.08
            _set_color(line, (0.0, 1.0, 0.20, 0.9))
            line.points.append(_marker_point(result.cylinder.center))
            line.points.append(_marker_point(result.point))
            line.points.append(_marker_point(_wall_face_point(result.wall)))
            markers.markers.append(line)

    return markers


class TaskAGapDetectorNode:
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/ego/livox_cloud_world")
        self.guide_path_topic = rospy.get_param("~guide_path_topic", "/task_a/guide_path")
        self.candidate_guide_path_topic = rospy.get_param(
            "~candidate_guide_path_topic", "/task_a/candidate_guide_path"
        )
        self.marker_topic = rospy.get_param("~marker_topic", "/task_a/markers")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.safe_radius = float(rospy.get_param("~safe_radius", 0.20))
        self.final_goal = (
            float(rospy.get_param("~goal_x", 3.120975971221924)),
            float(rospy.get_param("~goal_y", -5.956602573394775)),
            float(rospy.get_param("~goal_z", 1.0)),
        )
        self.default_goal_z = self.final_goal[2]
        self.target_ready = _as_bool(rospy.get_param("~target_ready", True))
        self.goal_input_topic = rospy.get_param("~goal_input_topic", "/task_a/final_goal")
        self.goal_input_use_pose_z = _as_bool(rospy.get_param("~goal_input_use_pose_z", False))
        self.corridor_margin = float(rospy.get_param("~corridor_margin", 0.7))
        self.lock_detection = _as_bool(rospy.get_param("~lock_detection", True))
        self.refine_duration = float(rospy.get_param("~refine_duration", 2.0))
        self.min_refine_matches = int(rospy.get_param("~min_refine_matches", 8))
        self.stable_center_threshold = float(rospy.get_param("~stable_center_threshold", 0.12))
        self.stable_gap_threshold = float(rospy.get_param("~stable_gap_threshold", 0.10))
        self.max_refine_duration = float(rospy.get_param("~max_refine_duration", 4.0))
        self.cylinder_forward_window = float(rospy.get_param("~cylinder_forward_window", 0.65))
        self.predicted_gap_width = float(rospy.get_param("~predicted_gap_width", 1.0))
        self.gap_width_min = float(rospy.get_param("~gap_width_min", 0.0))
        self.gap_width_max = float(rospy.get_param("~gap_width_max", 0.0))
        self.require_opposite_cylinders = _as_bool(rospy.get_param("~require_opposite_cylinders", True))
        self.cylinder_wall_lateral_overlap = float(rospy.get_param("~cylinder_wall_lateral_overlap", 0.20))
        self.z_min = float(rospy.get_param("~z_min", 0.2))
        self.z_max = float(rospy.get_param("~z_max", 2.2))
        self.cloud_xy_filter_enabled = _as_bool(rospy.get_param("~cloud_xy_filter_enabled", False))
        self.cloud_x_min = float(rospy.get_param("~cloud_x_min", -1.0))
        self.cloud_x_max = float(rospy.get_param("~cloud_x_max", 15.0))
        self.cloud_y_min = float(rospy.get_param("~cloud_y_min", -1.0))
        self.cloud_y_max = float(rospy.get_param("~cloud_y_max", 15.0))
        self.forward_min = float(rospy.get_param("~forward_min", 0.0))
        self.forward_max = float(rospy.get_param("~forward_max", 8.0))
        self.lateral_min = float(rospy.get_param("~lateral_min", -4.0))
        self.lateral_max = float(rospy.get_param("~lateral_max", 6.5))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.08))
        self.cluster_tolerance = float(rospy.get_param("~cluster_tolerance", 0.5))
        self.min_cluster_points = int(rospy.get_param("~min_cluster_points", 8))
        self.wall_length = float(rospy.get_param("~wall_length", DEFAULT_WALL_LENGTH))
        self.wall_width = float(rospy.get_param("~wall_width", DEFAULT_WALL_WIDTH))
        self.cylinder_diameter = float(rospy.get_param("~cylinder_diameter", DEFAULT_CYLINDER_DIAMETER))
        self.obstacle_height = float(rospy.get_param("~obstacle_height", DEFAULT_OBSTACLE_HEIGHT))
        self.min_structure_height = float(rospy.get_param("~min_structure_height", DEFAULT_MIN_STRUCTURE_HEIGHT))
        self.wall_length_min = float(rospy.get_param("~wall_length_min", 1.6))
        self.wall_length_max = float(rospy.get_param("~wall_length_max", 3.4))
        self.wall_width_min = float(rospy.get_param("~wall_width_min", 0.06))
        self.wall_width_max = float(rospy.get_param("~wall_width_max", 0.65))
        self.wall_aspect_min = float(rospy.get_param("~wall_aspect_min", 3.5))
        self.wall_face_enabled = _as_bool(rospy.get_param("~wall_face_enabled", True))
        self.wall_face_inlier_threshold = float(rospy.get_param("~wall_face_inlier_threshold", 0.10))
        self.wall_face_min_points = int(rospy.get_param("~wall_face_min_points", 8))
        self.wall_face_min_length = float(rospy.get_param("~wall_face_min_length", 0.35))
        self.wall_face_max_thickness = float(rospy.get_param("~wall_face_max_thickness", 0.25))
        self.wall_max_faces = int(rospy.get_param("~wall_max_faces", 2))
        self.wall_infer_length_margin = float(rospy.get_param("~wall_infer_length_margin", 0.20))
        self.wall_infer_height_margin = float(rospy.get_param("~wall_infer_height_margin", 0.05))
        self.wall_infer_depth_from_face = _as_bool(rospy.get_param("~wall_infer_depth_from_face", True))
        self.wall_size_from_cloud = _as_bool(rospy.get_param("~wall_size_from_cloud", True))
        self.select_nearest_gap = _as_bool(rospy.get_param("~select_nearest_gap", True))
        self.append_final_goal = _as_bool(rospy.get_param("~append_final_goal", False))
        self.wall_observer_uses_odom = _as_bool(rospy.get_param("~wall_observer_uses_odom", True))
        self.cylinder_length_min = float(rospy.get_param("~cylinder_length_min", 0.18))
        self.cylinder_length_max = float(rospy.get_param("~cylinder_length_max", 0.90))
        self.cylinder_width_min = float(rospy.get_param("~cylinder_width_min", 0.10))
        self.cylinder_width_max = float(rospy.get_param("~cylinder_width_max", 0.80))
        self.cylinder_aspect_max = float(rospy.get_param("~cylinder_aspect_max", 3.0))
        self.forward_axis = (
            float(rospy.get_param("~forward_axis_x", 0.0)),
            float(rospy.get_param("~forward_axis_y", -1.0)),
        )
        self.lateral_axis = (
            float(rospy.get_param("~lateral_axis_x", 1.0)),
            float(rospy.get_param("~lateral_axis_y", 0.0)),
        )
        self.auto_detect = _as_bool(rospy.get_param("~auto_detect", True))
        self.manual_click_enabled = _as_bool(rospy.get_param("~manual_click_enabled", True))
        self.manual_click_topic = rospy.get_param("~manual_click_topic", "/clicked_point")
        self.manual_click_radius = float(rospy.get_param("~manual_click_radius", 0.80))
        self.manual_cluster_tolerance = float(rospy.get_param("~manual_cluster_tolerance", 0.25))
        self.manual_min_cluster_points = int(rospy.get_param("~manual_min_cluster_points", 4))
        self.manual_model_kind = rospy.get_param("~manual_model_kind", "auto")
        self.manual_click_sequence = _as_bool(rospy.get_param("~manual_click_sequence", True))
        self.manual_wall_expand_enabled = _as_bool(rospy.get_param("~manual_wall_expand_enabled", True))
        self.manual_wall_expand_length = float(rospy.get_param("~manual_wall_expand_length", 3.6))
        self.manual_wall_expand_width = float(rospy.get_param("~manual_wall_expand_width", 1.0))
        self.manual_wall_expand_tolerance = float(rospy.get_param("~manual_wall_expand_tolerance", 0.35))
        self.manual_publish_guide_path = _as_bool(rospy.get_param("~manual_publish_guide_path", False))
        self.manual_dimension_margin = float(rospy.get_param("~manual_dimension_margin", 0.25))
        self.manual_height_margin = float(rospy.get_param("~manual_height_margin", 0.35))
        self.manual_cylinder_dimension_margin = float(
            rospy.get_param("~manual_cylinder_dimension_margin", 0.35)
        )
        self.manual_cylinder_aspect_margin = float(
            rospy.get_param("~manual_cylinder_aspect_margin", 1.0)
        )
        self.manual_priority_window_margin = float(
            rospy.get_param("~manual_priority_window_margin", 0.35)
        )
        self.manual_cluster_point_ratio = float(rospy.get_param("~manual_cluster_point_ratio", 0.60))
        self.calibration_file = _expand_path(
            rospy.get_param("~calibration_file", "~/.ros/task_a_gap_detector_calibration.yaml")
        )
        self.use_saved_manual_calibration = _as_bool(rospy.get_param("~use_saved_manual_calibration", True))
        self.save_manual_calibration = _as_bool(rospy.get_param("~save_manual_calibration", True))
        self.saved_prior_enabled = _as_bool(rospy.get_param("~saved_prior_enabled", True))
        self.saved_prior_fast_lock = _as_bool(rospy.get_param("~saved_prior_fast_lock", False))
        self.saved_prior_prefer_roi = _as_bool(rospy.get_param("~saved_prior_prefer_roi", True))
        self.saved_prior_margin = float(rospy.get_param("~saved_prior_margin", 0.45))
        self.saved_prior_z_margin = float(rospy.get_param("~saved_prior_z_margin", 0.25))
        self.saved_prior_gate_required = _as_bool(rospy.get_param("~saved_prior_gate_required", True))
        self.saved_prior_max_center_shift = float(rospy.get_param("~saved_prior_max_center_shift", 0.45))
        self.saved_prior_max_model_shift = float(rospy.get_param("~saved_prior_max_model_shift", 0.65))
        self.saved_prior_max_wall_yaw_shift = float(rospy.get_param("~saved_prior_max_wall_yaw_shift", 0.35))
        self.publish_saved_calibration_on_startup = _as_bool(
            rospy.get_param("~publish_saved_calibration_on_startup", False)
        )
        self.use_odom_front_roi = _as_bool(rospy.get_param("~use_odom_front_roi", False))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.front_yaw_offset = float(rospy.get_param("~front_yaw_offset", 0.0))
        self.publish_debug = _as_bool(rospy.get_param("~publish_debug", True))
        self.publish_candidate_guide_path = _as_bool(rospy.get_param("~publish_candidate_guide_path", True))
        self.locked_result = None
        self.latest_points = None
        self.latest_raw_points_count = 0
        self.latest_xy_filtered_points_count = 0
        self.latest_cloud_stamp = None
        self.manual_models = []
        self.manual_result = None
        self.saved_manual_models = []
        self.saved_result = None
        self.launch_calibration_params = self.calibration_params()
        self.odom_position = None
        self.odom_forward_axis = None
        self.odom_lateral_axis = None
        self.odom_stamp = None
        self.refiner = TaskAStructureRefiner(
            refine_duration=self.refine_duration,
            min_refine_matches=self.min_refine_matches,
            stable_center_threshold=self.stable_center_threshold,
            stable_gap_threshold=self.stable_gap_threshold,
            max_refine_duration=self.max_refine_duration,
        )
        if self.use_saved_manual_calibration and self.auto_detect:
            self.load_manual_calibration()

        self.guide_path_publisher = rospy.Publisher(self.guide_path_topic, Path, queue_size=1, latch=True)
        self.candidate_guide_path_publisher = rospy.Publisher(
            self.candidate_guide_path_topic, Path, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.goal_input_subscriber = None
        if PoseStamped is not None:
            self.goal_input_subscriber = rospy.Subscriber(
                self.goal_input_topic,
                PoseStamped,
                self.goal_input_callback,
                queue_size=1,
            )
        self.odom_subscriber = None
        if (
            (self.use_odom_front_roi or self.wall_observer_uses_odom or self.select_nearest_gap)
            and Odometry is not None
        ):
            self.odom_subscriber = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.odom_callback,
                queue_size=1,
            )
        self.subscriber = rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
            buff_size=16777216,
        )
        self.click_subscriber = None
        if self.manual_click_enabled and PointStamped is not None:
            self.click_subscriber = rospy.Subscriber(
                self.manual_click_topic,
                PointStamped,
                self.clicked_point_callback,
                queue_size=1,
            )

        rospy.loginfo(
            "task_a_gap_detector: cloud=%s odom=%s front_roi=%s guide_path=%s candidate_guide_path=%s markers=%s goal_input=%s target_ready=%s safe_radius=%.2f lock_detection=%s refine=%.2fs/%d auto_detect=%s manual_click=%s radius=%.2f wall_face=%s line_thr=%.2f max_faces=%d cloud_size=%s nearest_gap=%s xy_filter=%s x=[%.1f, %.1f] y=[%.1f, %.1f] wall=%.2fx%.2fx%.2f min_h=%.2f",
            self.cloud_topic,
            self.odom_topic,
            self.use_odom_front_roi,
            self.guide_path_topic,
            self.candidate_guide_path_topic,
            self.marker_topic,
            self.goal_input_topic,
            self.target_ready,
            self.safe_radius,
            self.lock_detection,
            self.refine_duration,
            self.min_refine_matches,
            self.auto_detect,
            self.manual_click_topic if self.manual_click_enabled else "disabled",
            self.manual_click_radius,
            self.wall_face_enabled,
            self.wall_face_inlier_threshold,
            self.wall_max_faces,
            self.wall_size_from_cloud,
            self.select_nearest_gap,
            self.cloud_xy_filter_enabled,
            self.cloud_x_min,
            self.cloud_x_max,
            self.cloud_y_min,
            self.cloud_y_max,
            self.wall_length,
            self.wall_width,
            self.obstacle_height,
            self.min_structure_height,
        )
        if self.auto_detect and self.publish_saved_calibration_on_startup and self.saved_result is not None:
            self.publish_result(
                self.saved_result,
                rospy.Time.now(),
                publish_guide_path=False,
            )

    def calibration_param_names(self):
        return (
            "wall_length",
            "wall_width",
            "obstacle_height",
            "min_structure_height",
            "wall_length_min",
            "wall_length_max",
            "wall_width_min",
            "wall_width_max",
            "wall_aspect_min",
            "wall_face_inlier_threshold",
            "wall_face_min_points",
            "wall_face_min_length",
            "wall_face_max_thickness",
            "wall_max_faces",
            "wall_infer_length_margin",
            "wall_infer_height_margin",
            "cylinder_diameter",
            "cylinder_length_min",
            "cylinder_length_max",
            "cylinder_width_min",
            "cylinder_width_max",
            "cylinder_aspect_max",
            "cylinder_forward_window",
            "predicted_gap_width",
            "gap_width_min",
            "gap_width_max",
            "cylinder_wall_lateral_overlap",
            "cluster_tolerance",
            "min_cluster_points",
        )

    def integer_calibration_param_names(self):
        return ("min_cluster_points", "wall_face_min_points", "wall_max_faces")

    def calibration_params(self):
        values = {}
        for name in self.calibration_param_names():
            if not hasattr(self, name):
                continue
            value = getattr(self, name)
            values[name] = int(value) if name in self.integer_calibration_param_names() else float(value)
        return values

    def apply_calibration_params(self, values, set_ros_params=True):
        changed = {}
        for name in self.calibration_param_names():
            if name not in values:
                continue
            try:
                value = int(values[name]) if name in self.integer_calibration_param_names() else float(values[name])
            except (TypeError, ValueError):
                continue
            setattr(self, name, value)
            changed[name] = value

        if changed and set_ros_params:
            self.set_private_params(changed)
        return changed

    def model_passes_current_params(self, model, kind):
        if model is None or model.height < self.min_structure_height:
            return False

        aspect = model.length / max(model.width, 1e-3)
        if kind == "wall":
            if model.face_center is not None:
                return (
                    model.length >= self.wall_face_min_length
                    and model.face_thickness <= self.wall_face_max_thickness
                )
            return (
                self.wall_length_min <= model.length <= self.wall_length_max
                and self.wall_width_min <= model.width <= self.wall_width_max
                and aspect >= self.wall_aspect_min
            )

        if kind == "cylinder":
            return (
                self.cylinder_length_min <= model.length <= self.cylinder_length_max
                and self.cylinder_width_min <= model.width <= self.cylinder_width_max
                and aspect <= self.cylinder_aspect_max
            )

        return False

    def detection_result_from_models(self, models, label_prefix="manual"):
        models = tuple(models)
        selected_gap, walls, cylinders, legal_gaps = match_task_a_structure(
            models,
            safe_radius=self.safe_radius,
            forward_axis=self.forward_axis,
            lateral_axis=self.lateral_axis,
            observer_origin_xy=self.observer_origin_xy((0.0, 0.0), rospy.Time.now()),
            wall_length=self.wall_length,
            wall_width=self.wall_width,
            cylinder_diameter=self.cylinder_diameter,
            obstacle_height=self.obstacle_height,
            min_structure_height=self.min_structure_height,
            cylinder_forward_window=self.cylinder_forward_window,
            predicted_gap_width=self.predicted_gap_width,
            gap_width_min=self.gap_width_min,
            gap_width_max=self.gap_width_max,
            require_opposite_cylinders=self.require_opposite_cylinders,
            cylinder_wall_lateral_overlap=self.cylinder_wall_lateral_overlap,
            wall_length_min=self.wall_length_min,
            wall_length_max=self.wall_length_max,
            wall_width_min=self.wall_width_min,
            wall_width_max=self.wall_width_max,
            wall_aspect_min=self.wall_aspect_min,
            cylinder_length_min=self.cylinder_length_min,
            cylinder_length_max=self.cylinder_length_max,
            cylinder_width_min=self.cylinder_width_min,
            cylinder_width_max=self.cylinder_width_max,
            cylinder_aspect_max=self.cylinder_aspect_max,
            wall_face_min_length=self.wall_face_min_length,
            wall_face_max_thickness=self.wall_face_max_thickness,
            wall_infer_length_margin=self.wall_infer_length_margin,
            wall_infer_height_margin=self.wall_infer_height_margin,
            wall_infer_depth_from_face=self.wall_infer_depth_from_face,
            wall_size_from_cloud=self.wall_size_from_cloud,
            select_nearest_gap=self.select_nearest_gap,
        )
        prefix = (label_prefix + "_") if label_prefix else ""
        stats = {
            "raw_points": len(self.latest_points) if self.latest_points is not None else 0,
            "filtered_points": 0,
            "downsampled_points": 0,
            "cluster_count": len(models),
            "wall_count": len(walls),
            "cylinder_count": len(cylinders),
        }
        if not walls or len(cylinders) < 2:
            wall = walls[0] if walls else None
            label = (
                prefix + "wall_found_need_two_cylinders"
                if wall is not None
                else prefix + "need_wall_and_two_cylinders"
            )
            return GapDetection(
                False,
                Point3(0.0, 0.0, 0.0),
                0.0,
                label,
                wall,
                models=models,
                cylinders=tuple(cylinders),
                legal_gaps=tuple(legal_gaps),
                **stats,
            )

        wall = walls[0]
        if selected_gap is None:
            return GapDetection(
                False,
                Point3(0.0, 0.0, 0.0),
                0.0,
                prefix + "no_safe_complete_wall_pillar_gap",
                wall,
                models=models,
                cylinders=tuple(cylinders),
                legal_gaps=tuple(legal_gaps),
                complete_match=len(cylinders) >= 2 and len(legal_gaps) >= 2,
                **stats,
            )

        return GapDetection(
            True,
            selected_gap.point,
            selected_gap.width,
            prefix + selected_gap.label,
            wall,
            selected_gap.cylinder,
            models,
            tuple(cylinders),
            tuple(legal_gaps),
            selected_gap,
            True,
            **stats,
        )

    def load_manual_calibration(self, apply_params=True):
        if not self.calibration_file or not os.path.exists(self.calibration_file):
            return False

        try:
            with open(self.calibration_file, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, ValueError, TypeError) as exc:
            rospy.logwarn(
                "task_a_gap_detector: failed to load calibration file %s: %s",
                self.calibration_file,
                exc,
            )
            return False

        changed = {}
        if apply_params:
            changed = self.apply_calibration_params(payload.get("params", {}), set_ros_params=True)
        models = []
        upgraded_wall_faces = False
        for item in payload.get("models", []):
            try:
                model = _model_from_dict(item)
            except (TypeError, ValueError, KeyError):
                continue
            if model.kind in ("wall", "cylinder") and model.length > 0.0 and model.width > 0.0:
                if (
                    model.kind == "wall"
                    and model.face_center is None
                    and model.width <= max(self.wall_face_max_thickness, 0.30)
                    and model.length / max(model.width, 1e-3) >= 1.2
                ):
                    model = ClusterModel(
                        model.kind,
                        model.center,
                        model.length,
                        model.width,
                        model.height,
                        model.min_lateral,
                        model.max_lateral,
                        model.point_count,
                        model.fit_error,
                        model.min_forward,
                        model.max_forward,
                        model.yaw,
                        model.center,
                        model.width,
                        model.face_count,
                        model.min_z,
                        model.max_z,
                    )
                    upgraded_wall_faces = True
                models.append(model)

        if upgraded_wall_faces:
            launch_wall_width = float(self.launch_calibration_params.get("wall_width", self.wall_width))
            if self.wall_width < launch_wall_width:
                self.wall_width = launch_wall_width
                width_margin = max(0.08, self.manual_dimension_margin * 0.5)
                self.wall_width_min = max(0.02, self.wall_width - width_margin)
                self.wall_width_max = self.wall_width + width_margin
                self.set_private_params(
                    {
                        "wall_width": self.wall_width,
                        "wall_width_min": self.wall_width_min,
                        "wall_width_max": self.wall_width_max,
                    }
                )

        self.saved_manual_models = models
        self.saved_result = self.detection_result_from_models(models, label_prefix="saved") if models else None

        rospy.loginfo(
            "task_a_gap_detector: loaded saved manual calibration %s models=%d params=%d complete=%s",
            self.calibration_file,
            len(models),
            len(changed),
            self.saved_result.found if self.saved_result is not None else False,
        )
        return True

    def save_manual_calibration_file(self):
        if not self.save_manual_calibration or not self.calibration_file or not self.manual_models:
            return

        payload = {
            "version": 1,
            "updated_at": rospy.Time.now().to_sec(),
            "frame_id": self.frame_id,
            "params": self.calibration_params(),
            "models": [_model_to_dict(model) for model in self.manual_models],
        }
        try:
            directory = os.path.dirname(self.calibration_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            temp_path = self.calibration_file + ".tmp"
            with open(temp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temp_path, self.calibration_file)
        except OSError as exc:
            rospy.logwarn(
                "task_a_gap_detector: failed to save calibration file %s: %s",
                self.calibration_file,
                exc,
            )
            return

        self.saved_manual_models = list(self.manual_models)
        self.saved_result = self.detection_result_from_models(self.saved_manual_models, label_prefix="saved")
        rospy.loginfo(
            "task_a_gap_detector: saved manual calibration %s models=%d complete=%s",
            self.calibration_file,
            len(self.saved_manual_models),
            self.saved_result.found if self.saved_result is not None else False,
        )

    def current_auto_detect(self):
        auto_detect = _as_bool(rospy.get_param("~auto_detect", self.auto_detect))
        if auto_detect != self.auto_detect:
            self.auto_detect = auto_detect
            if auto_detect:
                if self.use_saved_manual_calibration:
                    self.load_manual_calibration()
                self.reset_refinement()
                rospy.loginfo(
                    "task_a_gap_detector: switched to auto_detect=true; using saved manual calibration as point-cloud prior when available"
                )
            else:
                self.reset_manual_session()
                rospy.loginfo(
                    "task_a_gap_detector: switched to fresh manual click mode; previous manual memory cleared; click wall first, then two cones/cylinders"
                )
        return self.auto_detect

    def reset_manual_session(self):
        self.manual_models = []
        self.manual_result = None
        self.reset_refinement()
        if hasattr(self, "launch_calibration_params"):
            self.apply_calibration_params(self.launch_calibration_params, set_ros_params=True)

    def reset_refinement(self):
        self.locked_result = None
        self.refiner = TaskAStructureRefiner(
            refine_duration=self.refine_duration,
            min_refine_matches=self.min_refine_matches,
            stable_center_threshold=self.stable_center_threshold,
            stable_gap_threshold=self.stable_gap_threshold,
            max_refine_duration=self.max_refine_duration,
        )

    def choose_manual_kind(self, model):
        requested_kind = str(rospy.get_param("~manual_model_kind", self.manual_model_kind)).strip().lower()
        if requested_kind in ("wall", "cylinder", "cone", "pillar"):
            return "cylinder" if requested_kind in ("cone", "pillar") else requested_kind

        if self.manual_click_sequence:
            if not any(manual_model.kind == "wall" for manual_model in self.manual_models):
                return "wall"
            return "cylinder"

        aspect = model.length / max(model.width, 1e-3)
        wall_score = _wall_candidate_score(
            model,
            self.wall_length,
            self.wall_width,
            self.obstacle_height,
            self.min_structure_height,
        )
        cylinder_score = _cylinder_candidate_score(
            model,
            self.cylinder_diameter,
            self.obstacle_height,
            self.min_structure_height,
        )
        if aspect >= 1.15 and wall_score <= cylinder_score * 1.6:
            return "wall"
        return "cylinder" if cylinder_score <= wall_score * 1.25 else "wall"

    def expand_wall_model_from_click(self, seed_model, clicked_point):
        if not self.manual_wall_expand_enabled or self.latest_points is None:
            return seed_model

        major_axis = _normalize_xy((math.cos(seed_model.yaw), math.sin(seed_model.yaw)))
        minor_axis = (-major_axis[1], major_axis[0])
        origin = (clicked_point.x, clicked_point.y)
        origin_major = _dot_xy(origin, major_axis)
        origin_minor = _dot_xy(origin, minor_axis)
        half_length = max(self.manual_wall_expand_length, seed_model.length, self.wall_length) / 2.0
        half_width = max(self.manual_wall_expand_width, seed_model.width, self.wall_width)

        candidates = []
        for point in self.latest_points:
            if point[2] < self.z_min or point[2] > self.z_max:
                continue
            point_xy = (point[0], point[1])
            along = _dot_xy(point_xy, major_axis) - origin_major
            across = _dot_xy(point_xy, minor_axis) - origin_minor
            if abs(along) <= half_length and abs(across) <= half_width:
                candidates.append(point)

        if len(candidates) < max(self.manual_min_cluster_points, seed_model.point_count):
            return seed_model

        downsampled = voxel_downsample(candidates, self.voxel_size)
        clusters = euclidean_clusters(
            downsampled,
            tolerance=max(self.manual_cluster_tolerance, self.manual_wall_expand_tolerance),
            min_points=self.manual_min_cluster_points,
        )
        if not clusters:
            return seed_model

        click_xy = (clicked_point.x, clicked_point.y)
        cluster = min(
            clusters,
            key=lambda candidate: _distance_xy(_cluster_center(candidate)[:2], click_xy),
        )
        if len(cluster) < seed_model.point_count:
            return seed_model

        expanded_models = classify_cluster_models(
            cluster,
            lateral_axis=self.lateral_axis,
            forward_axis=self.forward_axis,
            wall_face_enabled=self.wall_face_enabled,
            wall_face_inlier_threshold=self.wall_face_inlier_threshold,
            wall_face_min_points=self.wall_face_min_points,
            wall_face_min_length=self.wall_face_min_length,
            wall_face_max_thickness=self.wall_face_max_thickness,
            wall_max_faces=self.wall_max_faces,
        )
        expanded_model = min(
            expanded_models,
            key=lambda candidate: _distance_xy(
                ((candidate.face_center or candidate.center).x, (candidate.face_center or candidate.center).y),
                (clicked_point.x, clicked_point.y),
            ),
        )
        if expanded_model.length < seed_model.length:
            return seed_model
        return expanded_model

    def model_from_click(self, clicked_point):
        if self.latest_points is None:
            return None, 0, 0

        radius_sq = self.manual_click_radius * self.manual_click_radius
        local_points = []
        for point in self.latest_points:
            if point[2] < self.z_min or point[2] > self.z_max:
                continue
            dx = point[0] - clicked_point.x
            dy = point[1] - clicked_point.y
            if dx * dx + dy * dy <= radius_sq:
                local_points.append(point)

        if len(local_points) < self.manual_min_cluster_points:
            return None, len(local_points), 0

        downsampled = voxel_downsample(local_points, self.voxel_size)
        clusters = euclidean_clusters(
            downsampled,
            tolerance=self.manual_cluster_tolerance,
            min_points=self.manual_min_cluster_points,
        )
        if clusters:
            click_xy = (clicked_point.x, clicked_point.y)
            cluster = min(
                clusters,
                key=lambda candidate: _distance_xy(_cluster_center(candidate)[:2], click_xy),
            )
        else:
            cluster = downsampled

        raw_models = classify_cluster_models(
            cluster,
            lateral_axis=self.lateral_axis,
            forward_axis=self.forward_axis,
            wall_face_enabled=self.wall_face_enabled,
            wall_face_inlier_threshold=self.wall_face_inlier_threshold,
            wall_face_min_points=self.wall_face_min_points,
            wall_face_min_length=self.wall_face_min_length,
            wall_face_max_thickness=self.wall_face_max_thickness,
            wall_max_faces=self.wall_max_faces,
        )
        raw_model = min(
            raw_models,
            key=lambda candidate: _distance_xy(
                ((candidate.face_center or candidate.center).x, (candidate.face_center or candidate.center).y),
                (clicked_point.x, clicked_point.y),
            ),
        )
        kind = self.choose_manual_kind(raw_model)
        if kind == "wall":
            raw_model = self.expand_wall_model_from_click(raw_model, clicked_point)
            if raw_model.face_center is not None:
                fit_error = raw_model.fit_error
            else:
                fit_error = _wall_candidate_score(
                    raw_model,
                    self.wall_length,
                    self.wall_width,
                    self.obstacle_height,
                    self.min_structure_height,
                )
        else:
            fit_error = _cylinder_candidate_score(
                raw_model,
                self.cylinder_diameter,
                self.obstacle_height,
                self.min_structure_height,
            )
        return _model_with_kind(raw_model, kind, fit_error), len(local_points), len(clusters)

    def saved_prior_roi_window(self):
        if not self.saved_manual_models:
            return None

        walls = [model for model in self.saved_manual_models if model.kind == "wall"]
        if walls:
            observer_origin_xy = self.observer_origin_xy((0.0, 0.0), rospy.Time.now())
            wall_side_axis, wall_depth_axis = _wall_axes_from_observer(
                walls[-1],
                self.forward_axis,
                observer_origin_xy,
            )
            roi_forward_axis = wall_depth_axis
            roi_lateral_axis = _normalize_xy(wall_side_axis)
            if _dot_xy(roi_lateral_axis, self.lateral_axis) < 0.0:
                roi_lateral_axis = (-roi_lateral_axis[0], -roi_lateral_axis[1])
        else:
            roi_forward_axis = _normalize_xy(self.forward_axis)
            roi_lateral_axis = _normalize_xy(self.lateral_axis)

        forward_ranges = []
        lateral_ranges = []
        z_mins = []
        z_maxs = []
        for model in self.saved_manual_models:
            forward_ranges.extend(_project_model_xy(model, roi_forward_axis))
            lateral_ranges.extend(_project_model_xy(model, roi_lateral_axis))
            model_min_z, model_max_z = _model_z_bounds(model)
            if model_max_z <= model_min_z:
                half_height = max(model.height, self.min_structure_height) * 0.5
                model_min_z = model.center.z - half_height
                model_max_z = model.center.z + half_height
            z_mins.append(model_min_z)
            z_maxs.append(model_max_z)

        if not forward_ranges or not lateral_ranges or not z_mins:
            return None

        xy_margin = max(
            self.saved_prior_margin,
            self.cluster_tolerance,
            self.manual_cluster_tolerance,
            self.voxel_size,
        )
        z_min = max(self.z_min, min(z_mins) - self.saved_prior_z_margin)
        z_max = min(self.z_max, max(z_maxs) + self.saved_prior_z_margin)
        if z_max <= z_min:
            z_min = self.z_min
            z_max = self.z_max

        return {
            "roi_forward_axis": roi_forward_axis,
            "roi_lateral_axis": roi_lateral_axis,
            "roi_origin": (0.0, 0.0),
            "forward_min": min(forward_ranges) - xy_margin,
            "forward_max": max(forward_ranges) + xy_margin,
            "lateral_min": min(lateral_ranges) - xy_margin,
            "lateral_max": max(lateral_ranges) + xy_margin,
            "z_min": z_min,
            "z_max": z_max,
        }

    def saved_prior_detection_result(self, points):
        if not self.saved_prior_enabled or not self.saved_manual_models:
            return None

        roi = self.saved_prior_roi_window()
        if roi is None:
            return None

        result = detect_task_a_gap(
            points,
            forward_axis=self.forward_axis,
            lateral_axis=self.lateral_axis,
            roi_forward_axis=roi["roi_forward_axis"],
            roi_lateral_axis=roi["roi_lateral_axis"],
            roi_origin=roi["roi_origin"],
            observer_origin_xy=self.observer_origin_xy(roi["roi_origin"], rospy.Time.now()),
            safe_radius=self.safe_radius,
            z_min=roi["z_min"],
            z_max=roi["z_max"],
            forward_min=roi["forward_min"],
            forward_max=roi["forward_max"],
            lateral_min=roi["lateral_min"],
            lateral_max=roi["lateral_max"],
            voxel_size=self.voxel_size,
            cluster_tolerance=self.cluster_tolerance,
            min_cluster_points=self.min_cluster_points,
            cylinder_forward_window=self.cylinder_forward_window,
            predicted_gap_width=self.predicted_gap_width,
            gap_width_min=self.gap_width_min,
            gap_width_max=self.gap_width_max,
            require_opposite_cylinders=self.require_opposite_cylinders,
            cylinder_wall_lateral_overlap=self.cylinder_wall_lateral_overlap,
            wall_length=self.wall_length,
            wall_width=self.wall_width,
            cylinder_diameter=self.cylinder_diameter,
            obstacle_height=self.obstacle_height,
            min_structure_height=self.min_structure_height,
            wall_length_min=self.wall_length_min,
            wall_length_max=self.wall_length_max,
            wall_width_min=self.wall_width_min,
            wall_width_max=self.wall_width_max,
            wall_aspect_min=self.wall_aspect_min,
            cylinder_length_min=self.cylinder_length_min,
            cylinder_length_max=self.cylinder_length_max,
            cylinder_width_min=self.cylinder_width_min,
            cylinder_width_max=self.cylinder_width_max,
            cylinder_aspect_max=self.cylinder_aspect_max,
            wall_face_enabled=self.wall_face_enabled,
            wall_face_inlier_threshold=self.wall_face_inlier_threshold,
            wall_face_min_points=self.wall_face_min_points,
            wall_face_min_length=self.wall_face_min_length,
            wall_face_max_thickness=self.wall_face_max_thickness,
            wall_max_faces=self.wall_max_faces,
            wall_infer_length_margin=self.wall_infer_length_margin,
            wall_infer_height_margin=self.wall_infer_height_margin,
            wall_infer_depth_from_face=self.wall_infer_depth_from_face,
            wall_size_from_cloud=self.wall_size_from_cloud,
            select_nearest_gap=self.select_nearest_gap,
        )
        result = _result_with_label(result, "saved_prior_roi_" + result.label)
        rospy.loginfo_throttle(
            1.0,
            (
                "task_a_gap_detector: saved-prior ROI auto-detect with tuned params "
                "filtered=%d clusters=%d walls=%d cylinders=%d "
                "f=%.2f..%.2f lat=%.2f..%.2f z=%.2f..%.2f result=%s"
            ),
            result.filtered_points,
            result.cluster_count,
            result.wall_count,
            result.cylinder_count,
            roi["forward_min"],
            roi["forward_max"],
            roi["lateral_min"],
            roi["lateral_max"],
            roi["z_min"],
            roi["z_max"],
            result.label,
        )
        return result

    def result_matches_saved_prior(self, result):
        if result is None or not result.found:
            return False, "no_current_match"

        reference = self.saved_result
        if reference is None or not reference.found:
            return True, "no_saved_complete_reference"

        gap_shift = _point_distance_xy(result.point, reference.point)
        if gap_shift > self.saved_prior_max_center_shift:
            return False, "gap_shift_%.2f" % gap_shift

        if result.wall is None or reference.wall is None:
            return False, "missing_wall_match"

        wall_shift = _point_distance_xy(_wall_face_point(result.wall), _wall_face_point(reference.wall))
        if wall_shift > self.saved_prior_max_model_shift:
            return False, "wall_face_shift_%.2f" % wall_shift

        wall_yaw_shift = _yaw_distance_pi_periodic(result.wall.yaw, reference.wall.yaw)
        if wall_yaw_shift > self.saved_prior_max_wall_yaw_shift:
            return False, "wall_yaw_shift_%.2f" % wall_yaw_shift

        detected_cylinders = list(result.cylinders)
        for saved_cylinder in reference.cylinders:
            if not detected_cylinders:
                return False, "missing_cylinder_match"

            nearest_index, nearest = min(
                enumerate(detected_cylinders),
                key=lambda item: _point_distance_xy(item[1].center, saved_cylinder.center),
            )
            cylinder_shift = _point_distance_xy(nearest.center, saved_cylinder.center)
            if cylinder_shift > self.saved_prior_max_model_shift:
                return False, "cylinder_shift_%.2f" % cylinder_shift

            detected_cylinders.pop(nearest_index)

        return True, "matched_saved_prior"

    def set_private_params(self, values):
        for name, value in values.items():
            rospy.set_param("~" + name, value)

    def apply_manual_calibration(self):
        walls = [model for model in self.manual_models if model.kind == "wall"]
        cylinders = [model for model in self.manual_models if model.kind == "cylinder"]
        changed = {}

        if walls:
            wall = walls[-1]
            launch_wall_length = float(self.launch_calibration_params.get("wall_length", self.wall_length))
            launch_wall_width = float(self.launch_calibration_params.get("wall_width", self.wall_width))
            observed_face = wall.face_center is not None
            if self.wall_size_from_cloud:
                self.wall_length = max(wall.length, 0.05)
            else:
                self.wall_length = max(wall.length, launch_wall_length, 0.05)
            if observed_face:
                self.wall_width = max(self.wall_width, launch_wall_width, self.wall_width_min, 0.03)
                observed_thickness = max(wall.face_thickness, wall.width, 0.02)
                self.wall_face_min_length = max(0.05, min(self.wall_face_min_length, wall.length * 0.60))
                self.wall_face_max_thickness = min(
                    0.80,
                    max(self.wall_face_max_thickness, observed_thickness + self.manual_dimension_margin * 0.5),
                )
                self.wall_infer_length_margin = max(
                    self.wall_infer_length_margin,
                    min(self.manual_dimension_margin, self.wall_length * 0.25),
                )
            else:
                self.wall_width = max(wall.width, 0.03)
            self.obstacle_height = max(wall.height, self.min_structure_height)
            self.wall_length_min = max(
                0.05,
                min(wall.length, self.wall_length) - self.manual_dimension_margin,
            )
            self.wall_length_max = self.wall_length + self.manual_dimension_margin
            width_margin = max(0.08, self.manual_dimension_margin * 0.5)
            self.wall_width_min = max(0.02, self.wall_width - width_margin)
            self.wall_width_max = self.wall_width + width_margin
            measured_aspect = self.wall_length / max(self.wall_width, 1e-3)
            self.wall_aspect_min = max(1.05, min(self.wall_aspect_min, measured_aspect * 0.70))
            self.min_structure_height = max(
                0.15,
                min(self.min_structure_height, max(0.15, wall.height - self.manual_height_margin)),
            )
            changed.update(
                {
                    "wall_length": self.wall_length,
                    "wall_width": self.wall_width,
                    "obstacle_height": self.obstacle_height,
                    "wall_length_min": self.wall_length_min,
                    "wall_length_max": self.wall_length_max,
                    "wall_width_min": self.wall_width_min,
                    "wall_width_max": self.wall_width_max,
                    "wall_aspect_min": self.wall_aspect_min,
                    "wall_face_min_length": self.wall_face_min_length,
                    "wall_face_max_thickness": self.wall_face_max_thickness,
                    "wall_infer_length_margin": self.wall_infer_length_margin,
                    "min_structure_height": self.min_structure_height,
                }
            )

        if cylinders:
            lengths = [max(model.length, 0.05) for model in cylinders]
            widths = [max(model.width, 0.05) for model in cylinders]
            diameters = [(length + width) / 2.0 for length, width in zip(lengths, widths)]
            self.cylinder_diameter = sum(diameters) / len(diameters)
            cylinder_margin = max(self.manual_cylinder_dimension_margin, self.manual_dimension_margin)
            self.cylinder_length_min = max(0.03, min(lengths) - cylinder_margin)
            self.cylinder_length_max = max(lengths) + cylinder_margin
            self.cylinder_width_min = max(0.03, min(widths) - cylinder_margin)
            self.cylinder_width_max = max(widths) + cylinder_margin
            observed_aspect = max(length / max(width, 1e-3) for length, width in zip(lengths, widths))
            self.cylinder_aspect_max = max(
                self.cylinder_aspect_max,
                observed_aspect + self.manual_cylinder_aspect_margin,
            )
            min_manual_points = min(model.point_count for model in cylinders)
            tuned_min_points = max(3, int(math.floor(min_manual_points * self.manual_cluster_point_ratio)))
            self.min_cluster_points = min(self.min_cluster_points, tuned_min_points)
            self.min_structure_height = max(
                0.15,
                min(
                    self.min_structure_height,
                    max(0.15, min(model.height for model in cylinders) - self.manual_height_margin),
                ),
            )
            changed.update(
                {
                    "cylinder_diameter": self.cylinder_diameter,
                    "cylinder_length_min": self.cylinder_length_min,
                    "cylinder_length_max": self.cylinder_length_max,
                    "cylinder_width_min": self.cylinder_width_min,
                    "cylinder_width_max": self.cylinder_width_max,
                    "cylinder_aspect_max": self.cylinder_aspect_max,
                    "min_cluster_points": self.min_cluster_points,
                    "min_structure_height": self.min_structure_height,
                }
            )

        if walls and cylinders:
            wall = _inferred_wall_candidate(
                walls[-1],
                self.forward_axis,
                self.lateral_axis,
                self.observer_origin_xy((0.0, 0.0), rospy.Time.now()),
                self.wall_length,
                self.wall_width,
                self.obstacle_height,
                self.min_structure_height,
                self.wall_length_min,
                self.wall_length_max,
                self.wall_width_min,
                self.wall_width_max,
                self.wall_aspect_min,
                self.wall_face_min_length,
                self.wall_face_max_thickness,
                self.wall_infer_length_margin,
                self.wall_infer_height_margin,
                self.wall_infer_depth_from_face,
                self.wall_size_from_cloud,
            ) or walls[-1]
            observer_origin_xy = self.observer_origin_xy((0.0, 0.0), rospy.Time.now())
            wall_side_axis, wall_depth_axis = _wall_axes_from_observer(
                wall,
                self.forward_axis,
                observer_origin_xy,
            )
            wall_side_min, wall_side_max = _project_model_xy(wall, wall_side_axis)
            wall_depth = _wall_near_face_depth(wall, wall_depth_axis, observer_origin_xy)
            depth_errors = []
            gaps = []
            for cylinder in cylinders:
                side_min, side_max = _project_model_xy(cylinder, wall_side_axis)
                depth_min, depth_max = _project_model_xy(cylinder, wall_depth_axis)
                depth_errors.append(abs(((depth_min + depth_max) / 2.0) - wall_depth))
                if side_max <= wall_side_min:
                    gaps.append(wall_side_min - side_max)
                elif side_min >= wall_side_max:
                    gaps.append(side_min - wall_side_max)

            if depth_errors:
                self.cylinder_forward_window = max(
                    0.20,
                    min(2.0, max(depth_errors) + self.manual_priority_window_margin),
                )
                changed["cylinder_forward_window"] = self.cylinder_forward_window
            if gaps:
                self.predicted_gap_width = sum(gaps) / len(gaps)
                changed["predicted_gap_width"] = self.predicted_gap_width

        if changed:
            self.set_private_params(changed)
            self.reset_refinement()
            rospy.loginfo(
                "task_a_gap_detector: tuned auto params from manual clicks cyl_l=%.2f..%.2f cyl_w=%.2f..%.2f aspect<=%.2f depth_window=%.2f expected_gap=%.2f min_cluster=%d",
                self.cylinder_length_min,
                self.cylinder_length_max,
                self.cylinder_width_min,
                self.cylinder_width_max,
                self.cylinder_aspect_max,
                self.cylinder_forward_window,
                self.predicted_gap_width,
                self.min_cluster_points,
            )

    def manual_detection_result(self):
        return self.detection_result_from_models(self.manual_models, label_prefix="manual")

    def clicked_point_callback(self, clicked):
        if self.current_auto_detect():
            rospy.loginfo_throttle(
                2.0,
                "task_a_gap_detector: auto_detect is true; set /task_a_gap_detector/auto_detect false before manual clicking",
            )
            return

        model, local_count, cluster_count = self.model_from_click(clicked.point)
        if model is None:
            rospy.logwarn(
                "task_a_gap_detector: clicked point found too few points (%d) within %.2fm; wait for /world_cloud or increase manual_click_radius",
                local_count,
                self.manual_click_radius,
            )
            return

        self.manual_models.append(model)
        self.apply_manual_calibration()
        self.manual_result = self.manual_detection_result()
        self.save_manual_calibration_file()
        self.publish_result(
            self.manual_result,
            rospy.Time.now(),
            publish_guide_path=self.manual_publish_guide_path,
        )
        rospy.loginfo(
            "task_a_gap_detector: manual click -> %s l=%.2f w=%.2f h=%.2f points=%d clusters=%d; models wall=%d cone_or_cylinder=%d. Switch to auto with: rosparam set /task_a_gap_detector/auto_detect true",
            model.kind,
            model.length,
            model.width,
            model.height,
            model.point_count,
            cluster_count,
            len([m for m in self.manual_models if m.kind == "wall"]),
            len([m for m in self.manual_models if m.kind == "cylinder"]),
        )

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        yaw = _yaw_from_quaternion(orientation) + self.front_yaw_offset
        forward_axis = (math.cos(yaw), math.sin(yaw))
        self.odom_position = (float(position.x), float(position.y))
        self.odom_forward_axis = _normalize_xy(forward_axis)
        self.odom_lateral_axis = _left_lateral_axis(self.odom_forward_axis)
        self.odom_stamp = odom.header.stamp if odom.header.stamp.to_sec() > 0.0 else rospy.Time.now()

    def goal_input_callback(self, goal):
        position = goal.pose.position
        z = position.z if self.goal_input_use_pose_z else self.default_goal_z
        if not self.goal_input_use_pose_z and abs(position.z) > 1e-3:
            z = position.z
        self.final_goal = (float(position.x), float(position.y), float(z))
        self.target_ready = True
        rospy.set_param("~target_ready", True)
        rospy.loginfo(
            "task_a_gap_detector: received RViz target goal=(%.2f, %.2f, %.2f); guide path will append/use this target",
            self.final_goal[0],
            self.final_goal[1],
            self.final_goal[2],
        )
        if self.locked_result is not None:
            self.publish_result(self.locked_result, rospy.Time.now(), publish_guide_path=True)

    def front_roi_frame(self, stamp):
        if not self.use_odom_front_roi:
            return self.forward_axis, self.lateral_axis, (0.0, 0.0), "fixed"

        if self.odom_position is None or self.odom_forward_axis is None:
            rospy.logwarn_throttle(
                1.0,
                "task_a_gap_detector: waiting for odom %s, using fixed ROI axis temporarily",
                self.odom_topic,
            )
            return self.forward_axis, self.lateral_axis, (0.0, 0.0), "fixed_wait_odom"

        age = abs((stamp - self.odom_stamp).to_sec()) if self.odom_stamp is not None else 0.0
        if age > self.odom_timeout:
            rospy.logwarn_throttle(
                1.0,
                "task_a_gap_detector: odom stale %.2fs > %.2fs, using fixed ROI axis temporarily",
                age,
                self.odom_timeout,
            )
            return self.forward_axis, self.lateral_axis, (0.0, 0.0), "fixed_stale_odom"

        return self.odom_forward_axis, self.odom_lateral_axis, self.odom_position, "odom_front"

    def observer_origin_xy(self, fallback_origin, stamp):
        if self.odom_position is None or self.odom_stamp is None:
            return fallback_origin
        age = abs((stamp - self.odom_stamp).to_sec())
        if age > self.odom_timeout:
            return fallback_origin
        return self.odom_position

    def publish_result(self, result, stamp, publish_guide_path=True):
        if result is None:
            return

        markers = make_task_a_markers(result, frame_id=self.frame_id, stamp=stamp)
        if markers is not None:
            self.marker_publisher.publish(markers)

        if not result.found:
            return

        if not self.target_ready:
            rospy.loginfo_throttle(
                2.0,
                "task_a_gap_detector: structure found, waiting for RViz 2D Nav Goal on %s before publishing guide path",
                self.goal_input_topic,
            )
            return

        path = build_task_a_guide_path(
            result,
            self.final_goal,
            frame_id=self.frame_id,
            stamp=stamp,
            forward_axis=self.forward_axis,
            corridor_margin=self.corridor_margin,
            append_final_goal=self.append_final_goal,
        )
        if path is None:
            return

        if publish_guide_path:
            self.guide_path_publisher.publish(path)
        elif self.publish_candidate_guide_path:
            self.candidate_guide_path_publisher.publish(path)

    def apply_cloud_xy_filter(self, points):
        self.latest_raw_points_count = len(points)
        if not self.cloud_xy_filter_enabled:
            self.latest_xy_filtered_points_count = len(points)
            return points

        x_min = min(self.cloud_x_min, self.cloud_x_max)
        x_max = max(self.cloud_x_min, self.cloud_x_max)
        y_min = min(self.cloud_y_min, self.cloud_y_max)
        y_max = max(self.cloud_y_min, self.cloud_y_max)
        filtered = [
            point
            for point in points
            if x_min <= point[0] <= x_max and y_min <= point[1] <= y_max
        ]
        self.latest_xy_filtered_points_count = len(filtered)
        rospy.loginfo_throttle(
            1.0,
            "task_a_gap_detector: cloud XY prefilter kept %d/%d points x=[%.1f, %.1f] y=[%.1f, %.1f]",
            len(filtered),
            len(points),
            x_min,
            x_max,
            y_min,
            y_max,
        )
        return filtered

    def cloud_callback(self, cloud):
        raw_points = [(p[0], p[1], p[2]) for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)]
        points = self.apply_cloud_xy_filter(raw_points)
        self.latest_points = points
        self.latest_cloud_stamp = cloud.header.stamp if cloud.header.stamp.to_sec() > 0.0 else rospy.Time.now()

        if not self.current_auto_detect():
            if self.manual_result is not None:
                self.publish_result(
                    self.manual_result,
                    rospy.Time.now(),
                    publish_guide_path=self.manual_publish_guide_path,
                )
            else:
                rospy.loginfo_throttle(
                    2.0,
                    "task_a_gap_detector: manual mode cached %d points; click wall first, then two cones/cylinders in RViz on %s",
                    len(points),
                    self.manual_click_topic,
                )
            return

        if self.lock_detection and self.locked_result is not None:
            self.publish_result(self.locked_result, rospy.Time.now())
            rospy.loginfo_throttle(
                1.0,
                "task_a_gap_detector: locked complete Task A structure %s point=(%.2f, %.2f, %.2f) width=%.2f",
                self.locked_result.label,
                self.locked_result.point.x,
                self.locked_result.point.y,
                self.locked_result.point.z,
                self.locked_result.width,
            )
            return

        stamp = rospy.Time.now()
        prior_result = self.saved_prior_detection_result(points)
        if prior_result is not None and prior_result.found:
            prior_ok, prior_reason = self.result_matches_saved_prior(prior_result)
            if not prior_ok:
                rospy.logwarn_throttle(
                    1.0,
                    "task_a_gap_detector: saved-prior ROI cluster rejected by prior gate (%s); keeping it as candidate only",
                    prior_reason,
                )
                prior_result = None
            elif self.saved_prior_fast_lock:
                if self.lock_detection:
                    self.locked_result = prior_result
                self.publish_result(prior_result, stamp, publish_guide_path=True)
                rospy.loginfo(
                    "task_a_gap_detector: saved-prior fast lock confirmed by current cloud clustering and prior gate gap=(%.2f, %.2f, %.2f) width=%.2f",
                    prior_result.point.x,
                    prior_result.point.y,
                    prior_result.point.z,
                    prior_result.width,
                )
                return

        if prior_result is not None and prior_result.found and self.saved_prior_prefer_roi:
            result = prior_result
            roi_origin = (0.0, 0.0)
            roi_source = "saved_prior_preferred"
        else:
            roi_forward_axis, roi_lateral_axis, roi_origin, roi_source = self.front_roi_frame(stamp)
            result = detect_task_a_gap(
                points,
                forward_axis=self.forward_axis,
                lateral_axis=self.lateral_axis,
                roi_forward_axis=roi_forward_axis,
                roi_lateral_axis=roi_lateral_axis,
                roi_origin=roi_origin,
                observer_origin_xy=self.observer_origin_xy(roi_origin, stamp),
                safe_radius=self.safe_radius,
                z_min=self.z_min,
                z_max=self.z_max,
                forward_min=self.forward_min,
                forward_max=self.forward_max,
                lateral_min=self.lateral_min,
                lateral_max=self.lateral_max,
                voxel_size=self.voxel_size,
                cluster_tolerance=self.cluster_tolerance,
                min_cluster_points=self.min_cluster_points,
                cylinder_forward_window=self.cylinder_forward_window,
                predicted_gap_width=self.predicted_gap_width,
                gap_width_min=self.gap_width_min,
                gap_width_max=self.gap_width_max,
                require_opposite_cylinders=self.require_opposite_cylinders,
                cylinder_wall_lateral_overlap=self.cylinder_wall_lateral_overlap,
                wall_length=self.wall_length,
                wall_width=self.wall_width,
                cylinder_diameter=self.cylinder_diameter,
                obstacle_height=self.obstacle_height,
                min_structure_height=self.min_structure_height,
                wall_length_min=self.wall_length_min,
                wall_length_max=self.wall_length_max,
                wall_width_min=self.wall_width_min,
                wall_width_max=self.wall_width_max,
                wall_aspect_min=self.wall_aspect_min,
                cylinder_length_min=self.cylinder_length_min,
                cylinder_length_max=self.cylinder_length_max,
                cylinder_width_min=self.cylinder_width_min,
                cylinder_width_max=self.cylinder_width_max,
                cylinder_aspect_max=self.cylinder_aspect_max,
                wall_face_enabled=self.wall_face_enabled,
                wall_face_inlier_threshold=self.wall_face_inlier_threshold,
                wall_face_min_points=self.wall_face_min_points,
                wall_face_min_length=self.wall_face_min_length,
                wall_face_max_thickness=self.wall_face_max_thickness,
                wall_max_faces=self.wall_max_faces,
                wall_infer_length_margin=self.wall_infer_length_margin,
                wall_infer_height_margin=self.wall_infer_height_margin,
                wall_infer_depth_from_face=self.wall_infer_depth_from_face,
                wall_size_from_cloud=self.wall_size_from_cloud,
                select_nearest_gap=self.select_nearest_gap,
            )
            if (
                prior_result is not None
                and prior_result.found
                and (not result.found or _result_quality_score(prior_result) <= _result_quality_score(result))
            ):
                result = prior_result
                roi_source = "saved_prior"
        if (
            self.saved_prior_gate_required
            and self.saved_result is not None
            and self.saved_result.found
            and result.found
        ):
            prior_ok, prior_reason = self.result_matches_saved_prior(result)
            if not prior_ok:
                rospy.logwarn_throttle(
                    1.0,
                    "task_a_gap_detector: auto result rejected by saved-prior gate (%s); not publishing guide path",
                    prior_reason,
                )
                result = _rejected_result(result, "saved_prior_gate_rejected_" + prior_reason)
                roi_source = roi_source + "_rejected"
        rospy.loginfo_throttle(
            1.0,
            (
                "task_a_gap_detector: raw=%d filtered=%d downsampled=%d "
                "clusters=%d matched_walls=%d matched_cylinders=%d roi=%s origin=(%.2f, %.2f) result=%s"
            ),
            result.raw_points,
            result.filtered_points,
            result.downsampled_points,
            result.cluster_count,
            result.wall_count,
            result.cylinder_count,
            roi_source,
            roi_origin[0],
            roi_origin[1],
            result.label,
        )

        if self.lock_detection:
            display_result = self.refiner.update(result, stamp.to_sec())
            self.publish_result(display_result or result, stamp, publish_guide_path=False)

            final_result = self.refiner.consume_publish_result()
            if final_result is not None:
                self.locked_result = final_result
                self.publish_result(final_result, stamp, publish_guide_path=True)
                rospy.loginfo(
                    "task_a_gap_detector: refined and locked complete Task A structure guide_path via gap=(%.2f, %.2f, %.2f) width=%.2f matches=%d",
                    final_result.point.x,
                    final_result.point.y,
                    final_result.point.z,
                    final_result.width,
                    len(self.refiner.observations),
                )

            if not result.found:
                if self.publish_debug:
                    rospy.logwarn_throttle(
                        1.0,
                        "task_a_gap_detector: %s wall=%s cylinders=%s models=%s",
                        result.label,
                        _model_summary((result.wall,) if result.wall is not None else ()),
                        _model_summary(result.cylinders),
                        _model_summary(result.models),
                    )
                return

            rospy.loginfo_throttle(
                1.0,
                "task_a_gap_detector: refining complete structure best=%s observations=%d publish_ready=%s",
                display_result.label if display_result is not None else result.label,
                len(self.refiner.observations),
                self.locked_result is not None,
            )
            return

        self.publish_result(result, stamp, publish_guide_path=True)

        if not result.found:
            if self.publish_debug:
                rospy.logwarn_throttle(
                    1.0,
                    "task_a_gap_detector: %s wall=%s cylinders=%s models=%s",
                    result.label,
                    _model_summary((result.wall,) if result.wall is not None else ()),
                    _model_summary(result.cylinders),
                    _model_summary(result.models),
                )
            return

        rospy.loginfo_throttle(
            1.0,
            "task_a_gap_detector: %s guide_path via gap=(%.2f, %.2f, %.2f) width=%.2f",
            result.label,
            result.point.x,
            result.point.y,
            result.point.z,
            result.width,
        )


def main():
    rospy.init_node("task_a_gap_detector")
    TaskAGapDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
