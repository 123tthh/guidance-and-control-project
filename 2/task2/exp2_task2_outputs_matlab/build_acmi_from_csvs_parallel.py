#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
build_acmi_from_csvs_parallel_8s.py

读取三种制导方法的详细 CSV，生成 Tacview ACMI。
并将三种工况沿 north 方向平移并列显示。

支持目录结构 1：
    当前目录/
    ├─三点法/exp2_task2_slx_three_point.csv
    ├─追踪法/exp2_task2_slx_pursuit.csv
    └─比例引导法/exp2_task2_slx_pn_K4.csv

支持目录结构 2：
    当前目录/
    ├─exp2_task2_matlab_three_point.csv
    ├─exp2_task2_matlab_pursuit.csv
    └─exp2_task2_matlab_pn_K4.csv

运行：
    python .\build_acmi_from_csvs_parallel_8s.py

可选：
    python .\build_acmi_from_csvs_parallel_8s.py --tmax 8 --out exp2_all_methods_parallel_8s.acmi
    python .\build_acmi_from_csvs_parallel_8s.py --spacing 30000
    python .\build_acmi_from_csvs_parallel_8s.py --step 2
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import List, Tuple


METHODS = [
    {
        "id": "three_point",
        "name": "三点法",
        "folders": ["三点法", "."],
        "patterns": [
            "exp2_task2_slx_three_point.csv",
            "exp2_task2_matlab_three_point.csv",
            "*three_point*.csv",
        ],
        "north_index": -1,
        "color": "Red",
        "station_id": 101,
        "missile_id": 102,
        "target_id": 103,
    },
    {
        "id": "pursuit",
        "name": "追踪法",
        "folders": ["追踪法", "."],
        "patterns": [
            "exp2_task2_slx_pursuit.csv",
            "exp2_task2_matlab_pursuit.csv",
            "*pursuit*.csv",
        ],
        "north_index": 0,
        "color": "Blue",
        "station_id": 201,
        "missile_id": 202,
        "target_id": 203,
    },
    {
        "id": "pn_K4",
        "name": "比例导引法 K=4",
        "folders": ["比例引导法", "比例导引法", "."],
        "patterns": [
            "exp2_task2_slx_pn_K4.csv",
            "exp2_task2_matlab_pn_K4.csv",
            "exp2_task2_matlab_pn_K4*.csv",
            "*pn_K4*.csv",
        ],
        "north_index": 1,
        "color": "Green",
        "station_id": 301,
        "missile_id": 302,
        "target_id": 303,
    },
]


REQUIRED_COLUMNS = [
    "time_s",
    "x_M_m",
    "y_M_m",
    "x_T_m",
    "y_T_m",
    "r_m",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate parallel Tacview ACMI from three method CSV files."
    )
    parser.add_argument(
        "--root",
        default=None,
        help="CSV 根目录。默认使用本 py 文件所在目录。",
    )
    parser.add_argument(
        "--out",
        default="exp2_all_methods_parallel_8s.acmi",
        help="输出 ACMI 文件名或路径。",
    )
    parser.add_argument(
        "--origin-lon",
        type=float,
        default=139.6917,
        help="Tacview 场景原点经度。",
    )
    parser.add_argument(
        "--origin-lat",
        type=float,
        default=35.6895,
        help="Tacview 场景原点纬度。",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=30000.0,
        help="三种工况沿 north 方向的间距，单位 m。",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="抽样步长。1 表示每行都写入 ACMI；2 表示隔一行取一点。",
    )
    parser.add_argument(
        "--tmax",
        type=float,
        default=8.0,
        help="只导出 time_s <= tmax 的数据，默认 8 秒。",
    )
    return parser.parse_args()


def script_dir() -> Path:
    return Path(__file__).resolve().parent


def f(row: dict, key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value is None:
        return default

    value = str(value).strip()
    if value == "":
        return default

    try:
        x = float(value)
        if math.isnan(x) or math.isinf(x):
            return default
        return x
    except Exception:
        return default


def find_csv(root: Path, method: dict) -> Path:
    candidates = []

    for folder in method["folders"]:
        folder_path = root / folder
        if not folder_path.exists():
            continue

        for pattern in method["patterns"]:
            candidates.extend(folder_path.glob(pattern))

    for pattern in method["patterns"]:
        candidates.extend(root.rglob(pattern))

    clean = []
    seen = set()

    for p in candidates:
        if not p.is_file():
            continue

        name = p.name.lower()

        if "summary" in name or "tacview_long" in name:
            continue

        rp = p.resolve()
        if rp in seen:
            continue

        seen.add(rp)
        clean.append(p)

    if not clean:
        raise FileNotFoundError(
            f"找不到 {method['name']} 的详细 CSV。\n"
            f"root = {root}\n"
            f"patterns = {method['patterns']}"
        )

    clean = sorted(clean, key=lambda x: (len(x.name), str(x)))
    return clean[0]


def read_rows(path: Path) -> List[dict]:
    with path.open("r", encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError(f"CSV 为空：{path}")

    missing = [c for c in REQUIRED_COLUMNS if c not in rows[0]]
    if missing:
        raise RuntimeError(
            f"CSV 缺少必要字段：{missing}\n"
            f"文件：{path}\n"
            f"当前字段：{list(rows[0].keys())}"
        )

    return rows


def trim_rows_by_time(rows: List[dict], tmax: float) -> List[dict]:
    trimmed = [row for row in rows if f(row, "time_s") <= tmax + 1e-9]

    if not trimmed:
        raise RuntimeError(f"该 CSV 在 0~{tmax} s 内没有数据。")

    return trimmed


def enu_to_llh(
    east_m: float,
    north_m: float,
    alt_m: float,
    origin_lon: float,
    origin_lat: float,
) -> Tuple[float, float, float]:
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = 111_320.0 * max(math.cos(math.radians(origin_lat)), 1e-6)

    lon = origin_lon + east_m / meters_per_deg_lon
    lat = origin_lat + north_m / meters_per_deg_lat
    alt = max(alt_m, 0.0)
    return lon, lat, alt


def attitude(rows: List[dict], i: int, x_key: str, y_key: str) -> Tuple[float, float]:
    if len(rows) < 2:
        return 90.0, 0.0

    i0 = max(i - 1, 0)
    i1 = min(i + 1, len(rows) - 1)

    dx = f(rows[i1], x_key) - f(rows[i0], x_key)
    dz = f(rows[i1], y_key) - f(rows[i0], y_key)

    if abs(dx) < 1e-9:
        heading = 0.0
    elif dx >= 0:
        heading = 90.0
    else:
        heading = 270.0

    pitch = math.degrees(math.atan2(dz, max(abs(dx), 1e-9)))
    return heading, pitch


def write_line(
    file,
    obj_id: int,
    lon: float,
    lat: float,
    alt: float,
    roll: float,
    pitch: float,
    heading: float,
    attrs: str = "",
) -> None:
    line = (
        f"{obj_id},"
        f"T={lon:.8f}|{lat:.8f}|{alt:.2f}|{roll:.2f}|{pitch:.2f}|{heading:.2f}"
    )

    if attrs:
        line += "," + attrs

    file.write(line + "\n")


def write_acmi(
    method_data: List[Tuple[dict, Path, List[dict]]],
    out_path: Path,
    origin_lon: float,
    origin_lat: float,
    spacing: float,
    step: int,
    tmax: float,
) -> None:
    events = []
    step = max(int(step), 1)

    for method, csv_path, rows in method_data:
        rows = trim_rows_by_time(rows, tmax)

        for i in range(0, len(rows), step):
            t = f(rows[i], "time_s")
            events.append((t, method, rows, i))

    events.sort(key=lambda x: (x[0], x[1]["id"]))

    declared = set()
    last_t = None

    with out_path.open("w", encoding="utf-8", newline="\n") as file:
        file.write("FileType=text/acmi/tacview\n")
        file.write("FileVersion=2.2\n")
        file.write("0,ReferenceTime=2026-01-01T00:00:00Z\n")
        file.write("0,RecordingTime=2026-01-01T00:00:00Z\n")
        file.write("0,Title=Exp2 Guidance Law Parallel Comparison 0-8s\n")
        file.write("0,DataSource=Detailed CSV files\n")
        file.write("0,Author=Guidance and Control Experiment\n")

        for t, method, rows, i in events:
            if last_t is None or abs(t - last_t) > 1e-9:
                file.write(f"#{t:.3f}\n")
                last_t = t

            row = rows[i]

            name = method["name"]
            color = method["color"]
            north = method["north_index"] * spacing

            station_id = method["station_id"]
            missile_id = method["missile_id"]
            target_id = method["target_id"]

            lon, lat, alt = enu_to_llh(0.0, north, 0.0, origin_lon, origin_lat)

            attrs = ""
            if station_id not in declared:
                attrs = f"Name={name} 制导站,Type=Ground+Static,Color=White"
                declared.add(station_id)

            write_line(file, station_id, lon, lat, alt, 0.0, 0.0, 0.0, attrs)

            xm = f(row, "x_M_m")
            ym = f(row, "y_M_m")
            heading_m, pitch_m = attitude(rows, i, "x_M_m", "y_M_m")
            lon, lat, alt = enu_to_llh(xm, north, ym, origin_lon, origin_lat)

            attrs = ""
            if missile_id not in declared:
                attrs = f"Name={name} 导弹,Type=Air+Missile,Color={color}"
                declared.add(missile_id)

            attrs_extra = (
                f"IAS=1000,"
                f"r_m={f(row, 'r_m'):.2f},"
                f"q_deg={f(row, 'q_deg'):.4f},"
                f"qdot_deg_s={f(row, 'qdot_deg_s'):.4f},"
                f"n_g={f(row, 'n_g'):.4f}"
            )

            attrs = attrs + "," + attrs_extra if attrs else attrs_extra
            write_line(file, missile_id, lon, lat, alt, 0.0, pitch_m, heading_m, attrs)

            xt = f(row, "x_T_m")
            yt = f(row, "y_T_m")
            heading_t, pitch_t = attitude(rows, i, "x_T_m", "y_T_m")
            lon, lat, alt = enu_to_llh(xt, north, yt, origin_lon, origin_lat)

            attrs = ""
            if target_id not in declared:
                attrs = f"Name={name} 目标,Type=Air+FixedWing,Color={color}"
                declared.add(target_id)

            attrs_extra = "IAS=500"
            attrs = attrs + "," + attrs_extra if attrs else attrs_extra
            write_line(file, target_id, lon, lat, alt, 0.0, pitch_t, heading_t, attrs)


def main() -> None:
    args = parse_args()

    root = Path(args.root).resolve() if args.root else script_dir()
    out_path = Path(args.out)

    if not out_path.is_absolute():
        out_path = root / out_path

    print(f"CSV 根目录: {root}")
    print(f"导出时间范围: 0 <= time_s <= {args.tmax:.3f} s")

    method_data = []

    for method in METHODS:
        csv_path = find_csv(root, method)
        rows = read_rows(csv_path)
        rows_0_tmax = trim_rows_by_time(rows, args.tmax)

        method_data.append((method, csv_path, rows))

        print(
            f"{method['name']}: {csv_path}  "
            f"rows={len(rows)}, rows_0_tmax={len(rows_0_tmax)}, "
            f"t_end_used={f(rows_0_tmax[-1], 'time_s'):.3f}s"
        )

    write_acmi(
        method_data=method_data,
        out_path=out_path,
        origin_lon=args.origin_lon,
        origin_lat=args.origin_lat,
        spacing=args.spacing,
        step=args.step,
        tmax=args.tmax,
    )

    print(f"ACMI written: {out_path}")


if __name__ == "__main__":
    main()