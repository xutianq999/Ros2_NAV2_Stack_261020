#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


PCD_NUMPY_TYPES = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
}


def read_binary_pcd_xyz(path: Path):
    header = {}
    offset = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA entry")
            offset += len(line)
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break

    if header["DATA"][0].lower() != "binary":
        raise ValueError("Only binary PCD files are supported")

    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    dtype_fields = []
    for name, size, value_type, count in zip(fields, sizes, types, counts):
        numpy_type = PCD_NUMPY_TYPES.get((value_type.upper(), size))
        if numpy_type is None:
            raise ValueError(f"Unsupported PCD field type: {value_type}{size}")
        dtype_fields.append((name, numpy_type) if count == 1 else (name, numpy_type, (count,)))

    cloud = np.memmap(path, dtype=np.dtype(dtype_fields), mode="r", offset=offset)
    for field in ("x", "y", "z"):
        if field not in cloud.dtype.names:
            raise ValueError(f"PCD has no {field!r} field")
    return cloud


def histogram(cloud, mask, origin_x, origin_y, resolution, width, height):
    x_cells = np.floor((np.asarray(cloud["x"][mask]) - origin_x) / resolution).astype(np.int32)
    y_cells = np.floor((np.asarray(cloud["y"][mask]) - origin_y) / resolution).astype(np.int32)
    inside = (x_cells >= 0) & (x_cells < width) & (y_cells >= 0) & (y_cells < height)
    linear = y_cells[inside].astype(np.int64) * width + x_cells[inside]
    return np.bincount(linear, minlength=width * height).reshape(height, width)


def main():
    parser = argparse.ArgumentParser(description="Project a binary PCD map into a Nav2 occupancy map")
    parser.add_argument("pcd", type=Path)
    parser.add_argument("output", type=Path, help="Output path without .pgm/.yaml suffix")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--ground-min", type=float, default=-0.35)
    parser.add_argument("--ground-max", type=float, default=0.15)
    parser.add_argument("--obstacle-min", type=float, default=0.20)
    parser.add_argument("--obstacle-max", type=float, default=1.50)
    parser.add_argument("--padding", type=float, default=0.50)
    parser.add_argument("--bounds-percentile", type=float, default=0.05)
    parser.add_argument("--min-ground-hits", type=int, default=3)
    parser.add_argument("--min-obstacle-hits", type=int, default=30)
    args = parser.parse_args()

    cloud = read_binary_pcd_xyz(args.pcd)
    x = np.asarray(cloud["x"])
    y = np.asarray(cloud["y"])
    z = np.asarray(cloud["z"])
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    useful_height = finite & (z >= args.ground_min) & (z <= args.obstacle_max)
    percentile = args.bounds_percentile
    low_x, high_x = np.percentile(x[useful_height], [percentile, 100.0 - percentile])
    low_y, high_y = np.percentile(y[useful_height], [percentile, 100.0 - percentile])
    origin_x = math.floor((low_x - args.padding) / args.resolution) * args.resolution
    origin_y = math.floor((low_y - args.padding) / args.resolution) * args.resolution
    max_x = math.ceil((high_x + args.padding) / args.resolution) * args.resolution
    max_y = math.ceil((high_y + args.padding) / args.resolution) * args.resolution
    width = int(round((max_x - origin_x) / args.resolution))
    height = int(round((max_y - origin_y) / args.resolution))

    ground_mask = finite & (z >= args.ground_min) & (z <= args.ground_max)
    obstacle_mask = finite & (z >= args.obstacle_min) & (z <= args.obstacle_max)
    ground_hits = histogram(
        cloud, ground_mask, origin_x, origin_y, args.resolution, width, height)
    obstacle_hits = histogram(
        cloud, obstacle_mask, origin_x, origin_y, args.resolution, width, height)

    free = (ground_hits >= args.min_ground_hits).astype(np.uint8)
    occupied = (obstacle_hits >= args.min_obstacle_hits).astype(np.uint8)

    # Fill tiny sampling gaps while preserving doors and narrow passages.
    free = ndimage.binary_closing(free, structure=np.ones((3, 3), bool))
    free = ndimage.binary_opening(free, structure=np.ones((2, 2), bool))
    occupied = ndimage.binary_closing(occupied, structure=np.ones((3, 3), bool))
    occupied = ndimage.binary_opening(occupied, structure=np.ones((2, 2), bool))

    image = np.full((height, width), 205, dtype=np.uint8)
    image[free != 0] = 254
    image[occupied != 0] = 0
    # PGM row zero is the map's maximum Y, while the histogram starts at minimum Y.
    image = np.flipud(image)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = args.output.with_suffix(".pgm")
    yaml_path = args.output.with_suffix(".yaml")
    Image.fromarray(image, mode="L").save(pgm_path)
    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="ascii",
    )

    print(f"Wrote {pgm_path} ({width}x{height}, resolution={args.resolution} m)")
    print(f"Wrote {yaml_path} (origin=[{origin_x:.3f}, {origin_y:.3f}, 0.0])")
    print(
        f"Cells: free={np.count_nonzero(image == 254)}, "
        f"occupied={np.count_nonzero(image == 0)}, unknown={np.count_nonzero(image == 205)}"
    )


if __name__ == "__main__":
    main()
