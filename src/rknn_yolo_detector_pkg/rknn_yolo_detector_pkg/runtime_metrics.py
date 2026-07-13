from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class SystemSnapshot:
    cpu_percent: float | None = None
    memory_used_bytes: int | None = None
    memory_total_bytes: int | None = None
    soc_temp_c: float | None = None
    npu_temp_c: float | None = None
    npu_load_percent: float | None = None
    npu_freq_hz: int | None = None


@dataclass(frozen=True)
class PerformanceSnapshot:
    in_fps: float | None = None
    out_fps: float | None = None
    drop_percent: float | None = None
    dropped_lifetime: int = 0
    e2e_ms: float | None = None
    queue_ms: float | None = None
    preprocess_ms: float | None = None
    inference_ms: float | None = None
    postprocess_ms: float | None = None


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return None


def parse_cpu_totals(text: str) -> tuple[int, int] | None:
    for line in text.splitlines():
        fields = line.split()
        if fields and fields[0] == "cpu" and len(fields) >= 5:
            try:
                values = [int(value) for value in fields[1:]]
            except ValueError:
                return None
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return total, idle
    return None


def parse_memory(text: str) -> tuple[int, int] | None:
    values: dict[str, int] = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0].endswith(":"):
            try:
                values[fields[0][:-1]] = int(fields[1]) * 1024
            except ValueError:
                continue
    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    if total is None or available is None or total <= 0 or available < 0:
        return None
    return total - available, total


def parse_npu_load(text: str) -> tuple[float | None, int | None]:
    value = text.strip()
    if not value:
        return None, None
    try:
        if "@" in value:
            load_text, freq_text = value.split("@", 1)
            load = float(load_text)
            freq = int(freq_text.removesuffix("Hz"))
            return (load if 0.0 <= load <= 100.0 else None), (freq if freq > 0 else None)
        load = float(value)
        return (load if 0.0 <= load <= 100.0 else None), None
    except ValueError:
        return None, None


class RuntimeMetrics:
    def __init__(self, window_sec: float) -> None:
        self._window_sec = window_sec
        self._events: deque[tuple[float, str, tuple[float, ...]]] = deque()
        self._dropped_lifetime = 0

    def _trim(self, now: float) -> None:
        while self._events and now - self._events[0][0] > self._window_sec:
            self._events.popleft()

    def record_input(self, now: float) -> None:
        self._events.append((now, "in", ()))
        self._trim(now)

    def record_drop(self, now: float) -> None:
        self._dropped_lifetime += 1
        self._events.append((now, "drop", ()))
        self._trim(now)

    def record_output(
        self, now: float, e2e_ms: float, queue_ms: float, preprocess_ms: float,
        inference_ms: float, postprocess_ms: float,
    ) -> None:
        self._events.append((now, "out", (e2e_ms, queue_ms, preprocess_ms, inference_ms, postprocess_ms)))
        self._trim(now)

    def snapshot(self, now: float) -> PerformanceSnapshot:
        self._trim(now)
        duration = self._window_sec
        inputs = [event for event in self._events if event[1] == "in"]
        drops = [event for event in self._events if event[1] == "drop"]
        outputs = [event[2] for event in self._events if event[1] == "out"]
        accepted_or_dropped = len(inputs) + len(drops)
        if not outputs:
            return PerformanceSnapshot(
                in_fps=len(inputs) / duration,
                out_fps=0.0,
                drop_percent=(100.0 * len(drops) / accepted_or_dropped) if accepted_or_dropped else 0.0,
                dropped_lifetime=self._dropped_lifetime,
            )
        averages = tuple(sum(row[index] for row in outputs) / len(outputs) for index in range(5))
        return PerformanceSnapshot(
            in_fps=len(inputs) / duration,
            out_fps=len(outputs) / duration,
            drop_percent=(100.0 * len(drops) / accepted_or_dropped) if accepted_or_dropped else 0.0,
            dropped_lifetime=self._dropped_lifetime,
            e2e_ms=averages[0], queue_ms=averages[1], preprocess_ms=averages[2],
            inference_ms=averages[3], postprocess_ms=averages[4],
        )


class SystemMetricsReader:
    def __init__(self, proc_root: Path = Path("/proc"), sys_root: Path = Path("/sys")) -> None:
        self._proc_root = proc_root
        self._sys_root = sys_root
        self._previous_cpu: tuple[int, int] | None = None

    def read(self) -> SystemSnapshot:
        cpu = self._read_cpu()
        memory = parse_memory(_read_text(self._proc_root / "meminfo") or "")
        soc_temp, npu_temp = self._read_temperatures()
        npu_load, npu_freq = self._read_npu()
        return SystemSnapshot(
            cpu_percent=cpu,
            memory_used_bytes=memory[0] if memory else None,
            memory_total_bytes=memory[1] if memory else None,
            soc_temp_c=soc_temp,
            npu_temp_c=npu_temp,
            npu_load_percent=npu_load,
            npu_freq_hz=npu_freq,
        )

    def _read_cpu(self) -> float | None:
        current = parse_cpu_totals(_read_text(self._proc_root / "stat") or "")
        previous = self._previous_cpu
        self._previous_cpu = current
        if current is None or previous is None:
            return None
        total_delta = current[0] - previous[0]
        idle_delta = current[1] - previous[1]
        if total_delta <= 0:
            return None
        return max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta)))

    def _read_temperatures(self) -> tuple[float | None, float | None]:
        soc = npu = None
        for zone in self._sys_root.glob("class/thermal/thermal_zone*"):
            zone_type = _read_text(zone / "type")
            raw = _read_text(zone / "temp")
            if zone_type not in {"soc-thermal", "npu-thermal"} or raw is None:
                continue
            try:
                value = float(raw)
            except ValueError:
                continue
            value = value / 1000.0 if value > 1000.0 else value
            if zone_type == "soc-thermal":
                soc = value
            else:
                npu = value
        return soc, npu

    def _read_npu(self) -> tuple[float | None, int | None]:
        base = self._sys_root / "class/devfreq/fdab0000.npu"
        load, load_freq = parse_npu_load(_read_text(base / "load") or "")
        raw_freq = _read_text(base / "cur_freq")
        try:
            current_freq = int(raw_freq) if raw_freq else None
        except ValueError:
            current_freq = None
        return load, current_freq or load_freq
