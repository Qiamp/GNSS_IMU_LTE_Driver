#!/usr/bin/env python3
"""Detect the IMU and u-blox F9P serial ports used by this workspace.

The script combines three signals:
  * stable /dev/serial/by-id links and udev metadata
  * UBX/NMEA/RTCM-like GNSS data from u-blox F9P
  * numeric CSV lines expected by src/imu_gnss_driver

It has no pyserial dependency; it uses the Linux termios API directly.
"""

from __future__ import annotations

import argparse
import dataclasses
import glob
import json
import os
import re
import select
import shutil
import subprocess
import sys
import termios
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


DEFAULT_PATTERNS = (
    "/dev/serial/by-id/*",
    "/dev/ttyACM*",
    "/dev/ttyUSB*",
)

DEFAULT_BAUDRATES = (921600,)
DEFAULT_SAMPLE_SECONDS = 2.5
MAX_CAPTURE_BYTES = 128 * 1024

IMU_CONFIG = "src/imu_gnss_driver/config/serial_params.yaml"
GNSS_CONFIG = "src/gnss_driver/config/driver_config.yaml"

NUMBER_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)
NMEA_START_RE = re.compile(rb"\$(?:GN|GP|GL|GA|GB|GQ)[A-Z]{3}[^\r\n]*")


@dataclasses.dataclass
class Candidate:
    real_path: str
    paths: List[str]
    properties: Dict[str, str]

    @property
    def stable_path(self) -> str:
        by_id = sorted(p for p in self.paths if p.startswith("/dev/serial/by-id/"))
        if by_id:
            return by_id[0]
        return self.real_path


@dataclasses.dataclass
class Observation:
    path: str
    baudrate: int
    bytes_read: int
    ublox_score: int
    imu_score: int
    evidence: List[str]
    error: Optional[str] = None

    @property
    def role(self) -> str:
        if self.ublox_score >= 70 and self.ublox_score > self.imu_score:
            return "ublox_f9p"
        if self.imu_score >= 70 and self.imu_score > self.ublox_score:
            return "imu"
        return "unknown"

    @property
    def score(self) -> int:
        if self.role == "ublox_f9p":
            return self.ublox_score
        if self.role == "imu":
            return self.imu_score
        return max(self.ublox_score, self.imu_score)

    @property
    def confidence(self) -> str:
        score = self.score
        if score >= 100:
            return "high"
        if score >= 70:
            return "medium"
        if score >= 40:
            return "low"
        return "none"


@dataclasses.dataclass
class PortResult:
    candidate: Candidate
    observations: List[Observation]

    @property
    def best(self) -> Observation:
        return max(self.observations, key=lambda obs: obs.score)


def baud_constant(baudrate: int) -> Optional[int]:
    return getattr(termios, "B{}".format(baudrate), None)


def expand_patterns(patterns: Sequence[str]) -> List[str]:
    paths: List[str] = []
    for pattern in patterns:
        if any(ch in pattern for ch in "*?["):
            paths.extend(sorted(glob.glob(pattern)))
        else:
            paths.append(pattern)
    return paths


def read_udev_properties(path: str) -> Dict[str, str]:
    if not shutil.which("udevadm"):
        return {}
    try:
        proc = subprocess.run(
            ["udevadm", "info", "--query=property", "--name", path],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1.5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {}

    properties: Dict[str, str] = {}
    for line in proc.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        properties[key] = value
    return properties


def discover_candidates(patterns: Sequence[str]) -> List[Candidate]:
    by_real: Dict[str, List[str]] = {}
    for path in expand_patterns(patterns):
        if not os.path.exists(path):
            continue
        real_path = os.path.realpath(path)
        if not os.path.exists(real_path):
            continue
        by_real.setdefault(real_path, []).append(path)
        by_real[real_path].append(real_path)

    candidates: List[Candidate] = []
    for real_path, paths in by_real.items():
        unique_paths = sorted(set(paths), key=path_sort_key)
        probe_path = next(
            (p for p in unique_paths if p.startswith("/dev/serial/by-id/")),
            real_path,
        )
        candidates.append(
            Candidate(
                real_path=real_path,
                paths=unique_paths,
                properties=read_udev_properties(probe_path),
            )
        )
    return sorted(candidates, key=lambda c: path_sort_key(c.stable_path))


def path_sort_key(path: str) -> Tuple[int, str]:
    if path.startswith("/dev/serial/by-id/"):
        return (0, path)
    if "/ttyACM" in path:
        return (1, path)
    if "/ttyUSB" in path:
        return (2, path)
    return (3, path)


def configure_serial(fd: int, baudrate: int) -> None:
    baud = baud_constant(baudrate)
    if baud is None:
        raise ValueError("baudrate {} is not supported by termios".format(baudrate))

    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (
        attrs[2]
        & ~(
            termios.PARENB
            | termios.CSTOPB
            | termios.CSIZE
            | getattr(termios, "CRTSCTS", 0)
        )
    )
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud
    attrs[5] = baud
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def sniff_serial(path: str, baudrate: int, seconds: float) -> Tuple[bytes, Optional[str]]:
    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        return b"", "{}".format(exc)

    old_attrs = None
    try:
        old_attrs = termios.tcgetattr(fd)
        configure_serial(fd, baudrate)
        chunks: List[bytes] = []
        total = 0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and total < MAX_CAPTURE_BYTES:
            timeout = min(0.1, max(0.0, deadline - time.monotonic()))
            readable, _, _ = select.select([fd], [], [], timeout)
            if not readable:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError as exc:
                return b"".join(chunks), "{}".format(exc)
            if not chunk:
                continue
            chunks.append(chunk)
            total += len(chunk)
        return b"".join(chunks), None
    except (OSError, ValueError) as exc:
        return b"", "{}".format(exc)
    finally:
        if old_attrs is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
            except OSError:
                pass
        os.close(fd)


def valid_ubx_messages(data: bytes) -> Tuple[int, List[str]]:
    count = 0
    names: List[str] = []
    index = 0
    while index + 8 <= len(data):
        start = data.find(b"\xb5\x62", index)
        if start < 0:
            break
        if start + 8 > len(data):
            break
        length = data[start + 4] | (data[start + 5] << 8)
        if length > 4096:
            index = start + 2
            continue
        end = start + 6 + length + 2
        if end > len(data):
            break

        ck_a = 0
        ck_b = 0
        for byte in data[start + 2 : start + 6 + length]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        if ck_a == data[end - 2] and ck_b == data[end - 1]:
            count += 1
            names.append("{:02x}:{:02x}".format(data[start + 2], data[start + 3]))
            index = end
        else:
            index = start + 2
    return count, names[:8]


def nmea_checksum_ok(sentence: bytes) -> bool:
    if not sentence.startswith(b"$") or b"*" not in sentence:
        return False
    body, checksum = sentence[1:].split(b"*", 1)
    if len(checksum) < 2:
        return False
    value = 0
    for byte in body:
        value ^= byte
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    return value == expected


def nmea_counts(data: bytes) -> Tuple[int, int]:
    candidates = NMEA_START_RE.findall(data)
    valid = sum(1 for item in candidates if nmea_checksum_ok(item))
    return valid, len(candidates)


def rtcm3_like_count(data: bytes) -> int:
    count = 0
    index = 0
    while index + 6 <= len(data):
        start = data.find(b"\xd3", index)
        if start < 0:
            break
        if start + 3 > len(data):
            break
        length = ((data[start + 1] & 0x03) << 8) | data[start + 2]
        if length == 0 or length > 1023:
            index = start + 1
            continue
        end = start + 3 + length + 3
        if end > len(data):
            break
        count += 1
        index = end
    return count


def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    printable = 0
    for byte in data:
        if byte in (9, 10, 13) or 32 <= byte < 127:
            printable += 1
    return printable / float(len(data))


def numeric_csv_lines(data: bytes) -> Tuple[int, int]:
    text = data.decode("ascii", errors="ignore")
    best_field_count = 0
    matching_lines = 0
    for raw_line in re.split(r"[\r\n]+", text):
        line = raw_line.strip()
        if not line or line.startswith("$"):
            continue
        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 18:
            continue
        numeric_count = sum(1 for part in parts if NUMBER_RE.match(part))
        if numeric_count >= 18 and numeric_count / float(len(parts)) >= 0.85:
            matching_lines += 1
            best_field_count = max(best_field_count, len(parts))
    return matching_lines, best_field_count


def metadata_scores(candidate: Candidate) -> Tuple[int, int, List[str]]:
    text_items = list(candidate.paths)
    text_items.extend("{}={}".format(k, v) for k, v in candidate.properties.items())
    text = " ".join(text_items).lower()

    ublox = 0
    imu = 0
    evidence: List[str] = []

    if "u-blox" in text or "ublox" in text:
        ublox += 75
        evidence.append("udev/by-id mentions u-blox")
    if "f9p" in text or "zed-f9p" in text:
        ublox += 30
        evidence.append("metadata mentions F9P")
    if candidate.properties.get("ID_VENDOR_ID", "").lower() == "1546":
        ublox += 65
        evidence.append("u-blox USB vendor id 1546")
    if "gnss" in text and ("receiver" in text or "gps" in text):
        ublox += 30
        evidence.append("metadata looks like a GNSS receiver")

    adapter_words = ("ch340", "wch", "cp210", "silicon labs", "ftdi", "arduino")
    if any(word in text for word in adapter_words) and ublox == 0:
        imu += 15
        evidence.append("generic USB serial adapter metadata")

    return ublox, imu, evidence


def classify(candidate: Candidate, baudrate: int, data: bytes, error: Optional[str]) -> Observation:
    ublox_score, imu_score, evidence = metadata_scores(candidate)

    if error:
        evidence.append("open/read error: {}".format(error))

    if data:
        ubx_count, ubx_names = valid_ubx_messages(data)
        if ubx_count:
            ublox_score += 90 + min(25, ubx_count * 3)
            evidence.append(
                "valid UBX frames: {} ({})".format(
                    ubx_count, ", ".join(ubx_names[:5])
                )
            )

        valid_nmea, possible_nmea = nmea_counts(data)
        if valid_nmea:
            ublox_score += 80 + min(25, valid_nmea * 3)
            evidence.append("valid NMEA sentences: {}".format(valid_nmea))
        elif possible_nmea:
            ublox_score += 35
            evidence.append("NMEA-like sentences: {}".format(possible_nmea))

        rtcm_count = rtcm3_like_count(data)
        if rtcm_count >= 2:
            ublox_score += 30
            evidence.append("RTCM3-like frames: {}".format(rtcm_count))

        csv_count, best_fields = numeric_csv_lines(data)
        if csv_count:
            imu_score += 90 + min(25, csv_count * 3)
            evidence.append(
                "numeric CSV lines: {}, best field count: {}".format(
                    csv_count, best_fields
                )
            )

        ratio = printable_ratio(data)
        comma_count = data.count(b",")
        if ratio > 0.80 and comma_count >= 20 and csv_count == 0:
            imu_score += 10
            evidence.append(
                "mostly printable data with many commas, but no full IMU CSV line"
            )
    else:
        evidence.append("no bytes captured")

    return Observation(
        path=candidate.stable_path,
        baudrate=baudrate,
        bytes_read=len(data),
        ublox_score=ublox_score,
        imu_score=imu_score,
        evidence=evidence,
        error=error,
    )


def select_best_role(results: Sequence[PortResult], role: str) -> Optional[PortResult]:
    matching = [result for result in results if result.best.role == role]
    if not matching:
        return None
    return max(matching, key=lambda result: result.best.score)


def config_path_for(candidate: Candidate, use_real_path: bool) -> str:
    return candidate.real_path if use_real_path else candidate.stable_path


def update_yaml_value(path: Path, key: str, value: str) -> None:
    text = path.read_text()
    pattern = re.compile(
        r"^(\s*{}\s*:\s*)(\"[^\"]*\"|'[^']*'|[^#\s]+)(.*)$".format(
            re.escape(key)
        ),
        re.MULTILINE,
    )

    def replace(match: re.Match[str]) -> str:
        return '{}"{}"{}'.format(match.group(1), value, match.group(3))

    updated, count = pattern.subn(replace, text, count=1)
    if count != 1:
        raise RuntimeError("key '{}' was not found in {}".format(key, path))
    path.write_text(updated)


def result_to_dict(result: PortResult, use_real_path: bool) -> Dict[str, object]:
    best = result.best
    return {
        "path": result.candidate.stable_path,
        "real_path": result.candidate.real_path,
        "config_path": config_path_for(result.candidate, use_real_path),
        "all_paths": result.candidate.paths,
        "role": best.role,
        "confidence": best.confidence,
        "score": best.score,
        "ublox_score": best.ublox_score,
        "imu_score": best.imu_score,
        "baudrate": best.baudrate,
        "bytes_read": best.bytes_read,
        "evidence": best.evidence,
    }


def print_results(results: Sequence[PortResult], use_real_path: bool) -> None:
    print("Serial port scan results:")
    for result in results:
        candidate = result.candidate
        best = result.best
        print("- {}".format(candidate.stable_path))
        if candidate.real_path != candidate.stable_path:
            print("  real path: {}".format(candidate.real_path))
        print(
            "  role: {}  confidence: {}  score: {}  baudrate: {}".format(
                best.role, best.confidence, best.score, best.baudrate
            )
        )
        print(
            "  scores: ublox_f9p={} imu={}  bytes_read={}".format(
                best.ublox_score, best.imu_score, best.bytes_read
            )
        )
        print("  config path: {}".format(config_path_for(candidate, use_real_path)))
        if best.evidence:
            print("  evidence: {}".format("; ".join(best.evidence[:5])))

    imu = select_best_role(results, "imu")
    ublox = select_best_role(results, "ublox_f9p")
    print("")
    print("Recommended config values:")
    if imu:
        print(
            '  {}: port: "{}"'.format(
                IMU_CONFIG, config_path_for(imu.candidate, use_real_path)
            )
        )
    else:
        print("  IMU: not identified")
    if ublox:
        print(
            '  {}: input_serial_port: "{}"'.format(
                GNSS_CONFIG, config_path_for(ublox.candidate, use_real_path)
            )
        )
    else:
        print("  u-blox F9P: not identified")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detect the IMU and u-blox F9P serial ports."
    )
    parser.add_argument(
        "ports",
        nargs="*",
        help="Optional explicit ports or glob patterns. Defaults to /dev/serial/by-id/*, /dev/ttyACM*, /dev/ttyUSB*.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        action="append",
        dest="baudrates",
        help="Baudrate to test. Can be repeated. Default: 921600.",
    )
    parser.add_argument(
        "--sample-seconds",
        type=float,
        default=DEFAULT_SAMPLE_SECONDS,
        help="Seconds to read each port at each baudrate. Default: 2.5.",
    )
    parser.add_argument(
        "--use-real-path",
        action="store_true",
        help="Recommend /dev/tty* paths instead of stable /dev/serial/by-id paths.",
    )
    parser.add_argument(
        "--write-config",
        action="store_true",
        help="Update the IMU and GNSS config files with the detected ports.",
    )
    parser.add_argument(
        "--imu-config",
        default=IMU_CONFIG,
        help="IMU YAML file to update with --write-config.",
    )
    parser.add_argument(
        "--gnss-config",
        default=GNSS_CONFIG,
        help="GNSS YAML file to update with --write-config.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON instead of text.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    patterns = args.ports if args.ports else list(DEFAULT_PATTERNS)
    baudrates = args.baudrates if args.baudrates else list(DEFAULT_BAUDRATES)
    unsupported = [baud for baud in baudrates if baud_constant(baud) is None]
    if unsupported:
        print(
            "Unsupported baudrate(s) on this system: {}".format(
                ", ".join(str(item) for item in unsupported)
            ),
            file=sys.stderr,
        )
        return 1

    candidates = discover_candidates(patterns)
    if not candidates:
        print("No serial ports found.", file=sys.stderr)
        return 1

    results: List[PortResult] = []
    for candidate in candidates:
        observations = []
        for baudrate in baudrates:
            data, error = sniff_serial(
                candidate.stable_path, baudrate, args.sample_seconds
            )
            observations.append(classify(candidate, baudrate, data, error))
        results.append(PortResult(candidate=candidate, observations=observations))

    use_real_path = bool(args.use_real_path)
    if args.json:
        print(
            json.dumps(
                [result_to_dict(result, use_real_path) for result in results],
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print_results(results, use_real_path)

    if args.write_config:
        imu = select_best_role(results, "imu")
        ublox = select_best_role(results, "ublox_f9p")
        if not imu or not ublox:
            print(
                "--write-config requires both IMU and u-blox F9P to be identified.",
                file=sys.stderr,
            )
            return 2
        update_yaml_value(
            Path(args.imu_config),
            "port",
            config_path_for(imu.candidate, use_real_path),
        )
        update_yaml_value(
            Path(args.gnss_config),
            "input_serial_port",
            config_path_for(ublox.candidate, use_real_path),
        )
        print("")
        print("Updated:")
        print("  {}".format(args.imu_config))
        print("  {}".format(args.gnss_config))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
