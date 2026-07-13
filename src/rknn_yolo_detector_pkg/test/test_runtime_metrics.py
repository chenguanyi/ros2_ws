from pathlib import Path

from rknn_yolo_detector_pkg.runtime_metrics import RuntimeMetrics, SystemMetricsReader, parse_cpu_totals, parse_memory, parse_npu_load


def test_parse_cpu_totals():
    assert parse_cpu_totals("cpu  100 20 30 40 10 0 0 0\n") == (200, 50)
    assert parse_cpu_totals("bad") is None


def test_parse_memory_uses_available():
    memory = parse_memory("MemTotal:       8192 kB\nMemAvailable:   2048 kB\n")
    assert memory == (6144 * 1024, 8192 * 1024)
    assert parse_memory("MemTotal: 100 kB\n") is None


def test_parse_npu_load_and_frequency():
    assert parse_npu_load("100@1000000000Hz") == (100.0, 1_000_000_000)
    assert parse_npu_load("invalid") == (None, None)
    assert parse_npu_load("101@100Hz") == (None, 100)


def test_runtime_metrics_counts_events_without_sequence_assumptions():
    metrics = RuntimeMetrics(1.0)
    metrics.record_input(0.0)
    metrics.record_input(0.2)
    metrics.record_drop(0.3)
    metrics.record_output(0.5, 50.0, 5.0, 2.0, 30.0, 8.0)
    metrics.record_output(0.6, 60.0, 6.0, 3.0, 31.0, 9.0)
    snapshot = metrics.snapshot(0.8)
    assert snapshot.in_fps == 2.0
    assert snapshot.out_fps == 2.0
    assert snapshot.drop_percent == 100 / 3
    assert snapshot.dropped_lifetime == 1
    assert snapshot.e2e_ms == 55.0
    assert snapshot.queue_ms == 5.5


def test_system_reader_handles_supported_files(tmp_path: Path):
    proc = tmp_path / "proc"
    sys = tmp_path / "sys"
    proc.mkdir()
    (proc / "stat").write_text("cpu  100 0 0 100 0 0 0 0\n")
    (proc / "meminfo").write_text("MemTotal: 8192 kB\nMemAvailable: 4096 kB\n")
    zone = sys / "class/thermal/thermal_zone4"
    zone.mkdir(parents=True)
    (zone / "type").write_text("soc-thermal")
    (zone / "temp").write_text("55000")
    npu_zone = sys / "class/thermal/thermal_zone6"
    npu_zone.mkdir(parents=True)
    (npu_zone / "type").write_text("npu-thermal")
    (npu_zone / "temp").write_text("60000")
    npu = sys / "class/devfreq/fdab0000.npu"
    npu.mkdir(parents=True)
    (npu / "load").write_text("87@1000000000Hz")
    (npu / "cur_freq").write_text("900000000")
    reader = SystemMetricsReader(proc, sys)
    first = reader.read()
    assert first.cpu_percent is None
    (proc / "stat").write_text("cpu  150 0 0 150 0 0 0 0\n")
    second = reader.read()
    assert second.cpu_percent == 50.0
    assert second.memory_used_bytes == 4096 * 1024
    assert second.soc_temp_c == 55.0
    assert second.npu_temp_c == 60.0
    assert second.npu_load_percent == 87.0
    assert second.npu_freq_hz == 900_000_000
