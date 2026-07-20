#!/usr/bin/env python3

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class TrajectorySample:
    position: Point3
    velocity: Point3
    yaw: float


def _point(values: Sequence[float]) -> Point3:
    if len(values) != 3:
        raise ValueError("A trajectory point must contain exactly x, y, z.")
    return (float(values[0]), float(values[1]), float(values[2]))


def _distance(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _lerp(a: Point3, b: Point3, ratio: float) -> Point3:
    return (
        a[0] + (b[0] - a[0]) * ratio,
        a[1] + (b[1] - a[1]) * ratio,
        a[2] + (b[2] - a[2]) * ratio,
    )


def _normalize(vector: Point3) -> Point3:
    length = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if length < 1e-9:
        return (0.0, 0.0, 0.0)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _yaw_from_velocity(velocity: Point3) -> float:
    if abs(velocity[0]) < 1e-9 and abs(velocity[1]) < 1e-9:
        return 0.0
    return math.atan2(velocity[1], velocity[0])


class Segment:
    def __init__(self, points: Iterable[Point3], speed: float):
        if speed <= 0.0:
            raise ValueError("Segment speed must be positive.")

        self.points = self._deduplicate(list(points))
        if len(self.points) < 2:
            raise ValueError("A segment needs at least two distinct points.")

        self.speed = float(speed)
        self.arc_lengths = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            self.arc_lengths.append(self.arc_lengths[-1] + _distance(start, end))

        self.length = self.arc_lengths[-1]
        if self.length < 1e-9:
            raise ValueError("A segment must have non-zero arc length.")
        self.duration = self.length / self.speed

    @staticmethod
    def _deduplicate(points: List[Point3]) -> List[Point3]:
        result = []
        for point in points:
            if not result or _distance(result[-1], point) > 1e-9:
                result.append(point)
        return result

    def sample_distance(self, distance_along: float) -> TrajectorySample:
        if distance_along <= 0.0:
            return self._sample_between(0, 0.0)
        if distance_along >= self.length:
            return self._sample_between(len(self.points) - 2, 1.0)

        index = max(0, bisect.bisect_right(self.arc_lengths, distance_along) - 1)
        segment_start = self.arc_lengths[index]
        segment_length = self.arc_lengths[index + 1] - segment_start
        ratio = (distance_along - segment_start) / segment_length
        return self._sample_between(index, ratio)

    def _sample_between(self, index: int, ratio: float) -> TrajectorySample:
        start = self.points[index]
        end = self.points[index + 1]
        position = _lerp(start, end, ratio)
        direction = _normalize((end[0] - start[0], end[1] - start[1], end[2] - start[2]))
        velocity = (direction[0] * self.speed, direction[1] * self.speed, direction[2] * self.speed)
        return TrajectorySample(position=position, velocity=velocity, yaw=_yaw_from_velocity(velocity))

    @classmethod
    def from_config(cls, config: dict, default_samples: int):
        segment_type = config.get("type")
        speed = float(config["speed"])
        if segment_type == "waypoint":
            points = [_point(point) for point in config["points"]]
        elif segment_type == "sine":
            points = _sine_points(config, int(config.get("samples", default_samples)))
        elif segment_type == "lane_change":
            points = _lane_change_points(config, int(config.get("samples", default_samples)))
        else:
            raise ValueError("Unsupported trajectory segment type: {}".format(segment_type))
        return cls(points, speed)


def _sine_points(config: dict, samples: int) -> List[Point3]:
    start = _point(config["start"])
    end = _point(config["end"])
    amplitude = float(config.get("amplitude", 0.0))
    cycles = float(config.get("cycles", 1.0))
    samples = max(2, samples)

    delta_xy = (end[0] - start[0], end[1] - start[1])
    length_xy = math.hypot(delta_xy[0], delta_xy[1])
    if length_xy < 1e-9:
        lateral = (0.0, 1.0)
    else:
        lateral = (-delta_xy[1] / length_xy, delta_xy[0] / length_xy)

    points = []
    for index in range(samples + 1):
        u = index / float(samples)
        base = _lerp(start, end, u)
        offset = amplitude * math.sin(2.0 * math.pi * cycles * u)
        points.append((base[0] + lateral[0] * offset, base[1] + lateral[1] * offset, base[2]))
    return points


def _smoothstep(u: float, smoothness: float) -> float:
    base = u * u * (3.0 - 2.0 * u)
    if abs(smoothness - 1.0) < 1e-9:
        return base
    return (1.0 - smoothness) * u + smoothness * base


def _lane_change_points(config: dict, samples: int) -> List[Point3]:
    start = _point(config["start"])
    end = _point(config["end"])
    smoothness = max(0.0, min(1.0, float(config.get("smoothness", 1.0))))
    samples = max(2, samples)

    points = []
    for index in range(samples + 1):
        u = index / float(samples)
        lateral_ratio = _smoothstep(u, smoothness)
        points.append(
            (
                start[0] + (end[0] - start[0]) * u,
                start[1] + (end[1] - start[1]) * lateral_ratio,
                start[2] + (end[2] - start[2]) * u,
            )
        )
    return points


class Trajectory:
    def __init__(self, segments: Sequence[Segment], loop: bool = False):
        if not segments:
            raise ValueError("Trajectory needs at least one segment.")
        self.segments = list(segments)
        self.loop = bool(loop)
        self.duration = sum(segment.duration for segment in self.segments)

    @classmethod
    def from_config(cls, config: dict):
        default_samples = int(config.get("samples", 200))
        segments = [Segment.from_config(segment, default_samples) for segment in config["segments"]]
        return cls(segments, bool(config.get("loop", False)))

    def sample(self, elapsed_time: float) -> TrajectorySample:
        if self.loop and self.duration > 1e-9:
            elapsed_time = elapsed_time % self.duration
        else:
            elapsed_time = max(0.0, elapsed_time)

        time_cursor = 0.0
        for segment in self.segments:
            next_cursor = time_cursor + segment.duration
            if elapsed_time <= next_cursor:
                return segment.sample_distance((elapsed_time - time_cursor) * segment.speed)
            time_cursor = next_cursor

        final_position = self.segments[-1].points[-1]
        return TrajectorySample(position=final_position, velocity=(0.0, 0.0, 0.0), yaw=0.0)

    def sample_path(self, dt: float) -> List[TrajectorySample]:
        if dt <= 0.0:
            raise ValueError("Path sampling dt must be positive.")
        samples = []
        steps = int(math.ceil(self.duration / dt))
        for index in range(steps + 1):
            samples.append(self.sample(min(index * dt, self.duration)))
        return samples
