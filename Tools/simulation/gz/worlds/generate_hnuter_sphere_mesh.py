#!/usr/bin/env python3
"""Generate the high-resolution visual mesh used by hnuter_sphere.sdf."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def generate_obj(
    output: Path,
    radius: float,
    longitude_segments: int,
    latitude_segments: int,
) -> None:
    vertices = [(0.0, 0.0, radius)]
    normals = [(0.0, 0.0, 1.0)]

    for latitude in range(1, latitude_segments):
        theta = math.pi * latitude / latitude_segments
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)

        for longitude in range(longitude_segments):
            phi = 2.0 * math.pi * longitude / longitude_segments
            nx = sin_theta * math.cos(phi)
            ny = sin_theta * math.sin(phi)
            nz = cos_theta
            vertices.append((radius * nx, radius * ny, radius * nz))
            normals.append((nx, ny, nz))

    vertices.append((0.0, 0.0, -radius))
    normals.append((0.0, 0.0, -1.0))
    bottom_index = len(vertices)

    faces = []
    first_ring = 2
    for longitude in range(longitude_segments):
        current = first_ring + longitude
        following = first_ring + (longitude + 1) % longitude_segments
        faces.append((1, current, following))

    ring_count = latitude_segments - 1
    for ring in range(ring_count - 1):
        current_ring = first_ring + ring * longitude_segments
        next_ring = current_ring + longitude_segments

        for longitude in range(longitude_segments):
            following = (longitude + 1) % longitude_segments
            a = current_ring + longitude
            b = next_ring + longitude
            c = next_ring + following
            d = current_ring + following
            faces.append((a, b, c))
            faces.append((a, c, d))

    last_ring = first_ring + (ring_count - 1) * longitude_segments
    for longitude in range(longitude_segments):
        current = last_ring + longitude
        following = last_ring + (longitude + 1) % longitude_segments
        faces.append((bottom_index, following, current))

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# Hnuter sphere visual mesh with analytic smooth normals\n")
        stream.write("o hnuter_sphere_smooth\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
        for x, y, z in normals:
            stream.write(f"vn {x:.9f} {y:.9f} {z:.9f}\n")
        stream.write("s 1\n")
        for a, b, c in faces:
            stream.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")

    print(
        f"Wrote {output}: {len(vertices)} vertices, "
        f"{len(faces)} triangles"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("meshes") / "hnuter_sphere_smooth.obj",
    )
    parser.add_argument("--radius", type=float, default=10.0)
    parser.add_argument("--longitude-segments", type=int, default=64)
    parser.add_argument("--latitude-segments", type=int, default=32)
    args = parser.parse_args()
    generate_obj(
        args.output,
        args.radius,
        args.longitude_segments,
        args.latitude_segments,
    )


if __name__ == "__main__":
    main()
