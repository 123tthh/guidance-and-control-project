"""
Build Tacview ACMI from Simulink logs for Experiment 1.

This version keeps the physical display altitude unchanged at 450 km,
uses the Tacview spacecraft/Soyuz visual carrier, and only applies:
    - attitude visualization gain = 4
    - ground/orbit longitude speed gain = 2

Input priority:
    1) exp1SimulinkForTacview.mat
    2) exp1SimulinkForTacview.csv
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Tuple, Optional

import numpy as np


INPUT_MAT = "exp1SimulinkForTacview.mat"
INPUT_CSV = "exp1SimulinkForTacview.csv"
OUTPUT_ACMI = "exp1_attitude_simulink_pid_soyuz_gain.acmi"
OUTPUT_SUMMARY = "exp1_acmi_soyuz_gain_summary.txt"

REFERENCE_TIME = "2026-06-20T00:00:00Z"

RE = 6378e3
MU = 3.986e14
H_ORBIT = 450e3
ALT_M = H_ORBIT
A_ORBIT = RE + H_ORBIT
W0 = math.sqrt(MU / A_ORBIT**3)
LON_RATE_DEG_PER_S = math.degrees(W0)

LAT_MAIN = 0.0
LON0 = 0.0

# Tacview visual carrier: the spacecraft item shown as "Soyuz".
# If your Tacview build does not render this, change Type to "Space+Satellite",
# but keep Name="Soyuz".
OBJECT_TYPE = "Space+Spacecraft"
OBJECT_NAME = "Soyuz"
OBJECT_COLOR = "Blue"
OBJECT_CALLSIGN = "SOYUZ-PID"

# The only visualization gains used in this version.
GROUND_MOTION_GAIN = 2.0
ATTITUDE_VISUAL_GAIN = 4.0

TMAX = 0.05
SATURATION_RATIO = 0.98
STEADY_RATE_EPS = 1.0e-4


def _as_1d(x) -> np.ndarray:
    return np.asarray(x).squeeze().reshape(-1)


def _as_time_by_channels(x, n_time: int, n_channels: Optional[int] = None) -> np.ndarray:
    y = np.asarray(x).squeeze()

    if y.ndim == 1:
        y = y.reshape(-1, 1)
    if y.ndim != 2:
        raise ValueError(f"Cannot convert shape {y.shape} to time-by-channel matrix.")

    if y.shape[1] == n_time and y.shape[0] != n_time:
        y = y.T
    if y.shape[0] != n_time and y.shape[1] == n_time:
        y = y.T
    if y.shape[0] != n_time:
        raise ValueError(f"Time dimension mismatch: data shape={y.shape}, time length={n_time}")

    if n_channels is not None:
        y = y[:, :n_channels]
    return y


def _load_mat(path: Path) -> Dict[str, np.ndarray]:
    try:
        from scipy.io import loadmat
        raw = loadmat(path)
    except NotImplementedError:
        raw = _load_mat_v73(path)

    if "t" in raw:
        t = _as_1d(raw["t"])
    elif "time" in raw:
        t = _as_1d(raw["time"])
    else:
        raise KeyError("MAT file must contain 't' or 'time'.")

    n = len(t)

    if "attitudeDeg" in raw:
        attitude_deg = _as_time_by_channels(raw["attitudeDeg"], n, 3)
    elif "attitudeRad" in raw:
        attitude_deg = np.rad2deg(_as_time_by_channels(raw["attitudeRad"], n, 3))
    elif all(k in raw for k in ("phiDeg", "thetaDeg", "psiDeg")):
        attitude_deg = np.column_stack([_as_1d(raw["phiDeg"]), _as_1d(raw["thetaDeg"]), _as_1d(raw["psiDeg"])])
    else:
        raise KeyError("MAT file must contain attitudeDeg, attitudeRad, or phiDeg/thetaDeg/psiDeg.")

    data = {"time": t, "attitudeDeg": attitude_deg}

    if "wheelTorque" in raw:
        data["wheelTorque"] = _as_time_by_channels(raw["wheelTorque"], n, 4)
    if "actualTorque" in raw:
        data["actualTorque"] = _as_time_by_channels(raw["actualTorque"], n, 3)
    if "rate" in raw:
        data["rate"] = _as_time_by_channels(raw["rate"], n, 3)

    return data


def _load_mat_v73(path: Path) -> Dict[str, np.ndarray]:
    import h5py
    raw = {}
    with h5py.File(path, "r") as f:
        for k in f.keys():
            raw[k] = np.array(f[k]).T
    return raw


def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    arr = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")

    def col(name: str) -> np.ndarray:
        if name not in arr.dtype.names:
            raise KeyError(f"CSV missing column: {name}")
        return np.asarray(arr[name]).reshape(-1)

    data = {
        "time": col("time"),
        "attitudeDeg": np.column_stack([col("phiDeg"), col("thetaDeg"), col("psiDeg")]),
    }

    if all(k in arr.dtype.names for k in ("wheel1", "wheel2", "wheel3", "wheel4")):
        data["wheelTorque"] = np.column_stack([col("wheel1"), col("wheel2"), col("wheel3"), col("wheel4")])
    if all(k in arr.dtype.names for k in ("uX", "uY", "uZ")):
        data["actualTorque"] = np.column_stack([col("uX"), col("uY"), col("uZ")])
    if all(k in arr.dtype.names for k in ("omegaX", "omegaY", "omegaZ")):
        data["rate"] = np.column_stack([col("omegaX"), col("omegaY"), col("omegaZ")])

    return data


def load_simulink_data() -> Tuple[Dict[str, np.ndarray], Path]:
    mat_path = Path(INPUT_MAT)
    csv_path = Path(INPUT_CSV)

    if mat_path.exists():
        return _load_mat(mat_path), mat_path
    if csv_path.exists():
        return _load_csv(csv_path), csv_path

    raise FileNotFoundError(f"Cannot find {INPUT_MAT} or {INPUT_CSV}.")


def settling_time(time: np.ndarray, attitude_deg: np.ndarray, band_deg: float) -> Optional[float]:
    inside = np.all(np.abs(attitude_deg) <= band_deg, axis=1)
    outside_idx = np.where(~inside)[0]
    if len(outside_idx) == 0:
        return float(time[0])
    last_outside = outside_idx[-1]
    if last_outside + 1 < len(time):
        return float(time[last_outside + 1])
    return None


def rate_settling_time(time: np.ndarray, rate: Optional[np.ndarray]) -> Optional[float]:
    if rate is None:
        return None
    mag = np.linalg.norm(rate, axis=1)
    outside_idx = np.where(mag > STEADY_RATE_EPS)[0]
    if len(outside_idx) == 0:
        return float(time[0])
    last_outside = outside_idx[-1]
    if last_outside + 1 < len(time):
        return float(time[last_outside + 1])
    return None


def wheel_saturation_interval(time: np.ndarray, wheel_torque: Optional[np.ndarray]):
    if wheel_torque is None:
        return None, None, float("nan")
    peak = float(np.max(np.abs(wheel_torque)))
    saturated = np.max(np.abs(wheel_torque), axis=1) >= SATURATION_RATIO * TMAX
    idx = np.where(saturated)[0]
    if len(idx) == 0:
        return None, None, peak
    return float(time[idx[0]]), float(time[idx[-1]]), peak


def attitude_to_tacview(phi_deg: float, theta_deg: float, psi_deg: float):
    # The physical Simulink attitude is kept in data; only Tacview display is amplified.
    roll = ATTITUDE_VISUAL_GAIN * phi_deg
    pitch = ATTITUDE_VISUAL_GAIN * theta_deg
    yaw = 90.0 - ATTITUDE_VISUAL_GAIN * psi_deg
    return roll, pitch, yaw


def lonlat_at(t_sec: float):
    lon = LON0 + GROUND_MOTION_GAIN * LON_RATE_DEG_PER_S * t_sec
    lat = LAT_MAIN
    return lon, lat


def hexid(v: int) -> str:
    return f"{v:X}"


def add_bookmark(lines, t_sec: float, text: str):
    lines.append(f"#{t_sec:.3f}")
    lines.append(f"0,Event=Bookmark|{text}")


def build_acmi(data: Dict[str, np.ndarray], source_file: Path) -> Dict[str, object]:
    time = np.asarray(data["time"]).reshape(-1)
    time = time - time[0]

    attitude = np.asarray(data["attitudeDeg"])
    wheel_torque = data.get("wheelTorque")
    rate = data.get("rate")

    n = len(time)
    duration = float(time[-1])

    ts_2 = settling_time(time, attitude, 2.0)
    ts_1 = settling_time(time, attitude, 1.0)
    ts_rate = rate_settling_time(time, rate)
    sat_start, sat_end, wheel_peak = wheel_saturation_interval(time, wheel_torque)

    object_id = 0xC101

    lines = []
    lines.append("FileType=text/acmi/tacview")
    lines.append("FileVersion=2.2")
    lines.append(f"0,ReferenceTime={REFERENCE_TIME}")
    lines.append("0,Title=Experiment 1 - Simulink PID Anti-windup Satellite Attitude Control")
    lines.append("0,Category=Education")
    lines.append(
        "0,Briefing=ACMI generated directly from Simulink logs. "
        "Visual carrier is Tacview Soyuz spacecraft. "
        "Altitude remains 450 km. Display-only gains: attitude x4, ground motion x2."
    )
    lines.append("0,ReferenceLongitude=0")
    lines.append("0,ReferenceLatitude=0")

    phi0, theta0, psi0 = attitude[0, :]
    roll0, pitch0, yaw0 = attitude_to_tacview(phi0, theta0, psi0)
    lon0, lat0 = lonlat_at(0.0)

    lines.append("#0")
    lines.append(
        f"{hexid(object_id)},"
        f"T={lon0:.7f}|{lat0:.7f}|{ALT_M:.1f}|{roll0:.3f}|{pitch0:.3f}|{yaw0:.3f},"
        f"Name={OBJECT_NAME},Type={OBJECT_TYPE},Color={OBJECT_COLOR},Coalition=Allies,CallSign={OBJECT_CALLSIGN}"
    )
    lines.append("0,Event=Bookmark|Simulink PID anti-windup simulation starts")

    for i in range(n):
        ti = float(time[i])
        phi, theta, psi = attitude[i, :]
        roll, pitch, yaw = attitude_to_tacview(phi, theta, psi)
        lon, lat = lonlat_at(ti)
        lines.append(f"#{ti:.3f}")
        lines.append(f"{hexid(object_id)},T={lon:.7f}|{lat:.7f}|{ALT_M:.1f}|{roll:.3f}|{pitch:.3f}|{yaw:.3f}")

    if ts_2 is not None:
        add_bookmark(lines, ts_2, f"Attitude enters and stays within +/-2 deg at t={ts_2:.1f}s")
    if ts_1 is not None:
        add_bookmark(lines, ts_1, f"Attitude enters and stays within +/-1 deg at t={ts_1:.1f}s")
    if ts_rate is not None:
        add_bookmark(lines, ts_rate, f"Relative angular rate norm below {STEADY_RATE_EPS:g} rad/s at t={ts_rate:.1f}s")

    if wheel_torque is not None:
        if sat_start is None:
            add_bookmark(lines, 0.0, f"No wheel saturation detected; peak wheel torque = {wheel_peak:.4f} N*m")
        else:
            add_bookmark(lines, sat_start, f"Wheel torque saturation starts near t={sat_start:.1f}s")
            add_bookmark(lines, sat_end, f"Wheel torque saturation ends near t={sat_end:.1f}s")
            add_bookmark(lines, 0.0, f"Peak wheel torque = {wheel_peak:.4f} N*m, limit = {TMAX:.4f} N*m")

    with open(OUTPUT_ACMI, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")

    summary = {
        "source": str(source_file),
        "output": OUTPUT_ACMI,
        "duration_s": duration,
        "frames": n,
        "final_attitude_deg": attitude[-1, :],
        "max_abs_attitude_deg": np.max(np.abs(attitude), axis=0),
        "settling_2deg_s": ts_2,
        "settling_1deg_s": ts_1,
        "rate_settling_s": ts_rate,
        "wheel_peak_Nm": wheel_peak,
        "wheel_saturation_start_s": sat_start,
        "wheel_saturation_end_s": sat_end,
        "object_type": OBJECT_TYPE,
        "object_name": OBJECT_NAME,
        "altitude_m": ALT_M,
        "ground_motion_gain": GROUND_MOTION_GAIN,
        "attitude_visual_gain": ATTITUDE_VISUAL_GAIN,
    }

    with open(OUTPUT_SUMMARY, "w", encoding="utf-8", newline="\n") as f:
        f.write("Experiment 1 Tacview ACMI generation summary\n")
        f.write("================================================\n\n")
        for key, value in summary.items():
            if isinstance(value, np.ndarray):
                value = np.array2string(value, precision=4)
            f.write(f"{key}: {value}\n")

    return summary


def main():
    data, source = load_simulink_data()
    summary = build_acmi(data, source)

    print("\nACMI generation completed.")
    print(f"Input:  {summary['source']}")
    print(f"Output: {summary['output']}")
    print(f"Object: {summary['object_type']} / {summary['object_name']}")
    print(f"Altitude: {summary['altitude_m']:.0f} m")
    print(f"Ground motion gain: {summary['ground_motion_gain']}")
    print(f"Attitude visual gain: {summary['attitude_visual_gain']}")
    print(f"Duration: {summary['duration_s']:.1f} s")
    print(f"Frames: {summary['frames']}")
    print(f"Final attitude deg: {np.round(summary['final_attitude_deg'], 4)}")
    print(f"Peak wheel torque: {summary['wheel_peak_Nm']:.5f} N*m")


if __name__ == "__main__":
    main()
