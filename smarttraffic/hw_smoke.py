"""hw_smoke -- live ESP32 integration check (Stage 9). Needs the real hardware.

    python -m smarttraffic.hw_smoke --list-ports
    python -m smarttraffic.hw_smoke --mode link   --port COM5 --seconds 30
    python -m smarttraffic.hw_smoke --mode light  --port COM5
    python -m smarttraffic.hw_smoke --mode hybrid --port COM5 --seconds 120 [--traffic S4]

Modes
  link    connect, show HELLO / link state / every real sensor edge.
  light   drive the mapped movement(s) through the full Signals V2 sequence twice
          and mirror it on the physical head(s).
  hybrid  the full Stage 9 stack: VehicleModel + VirtualDetectors (15 virtual)
          + real detector(s) via HybridDetectorSource -> SensorBank, SignalBank with a
          repeating TEST PATTERN on the mirrored movement (not a controller), and the
          physical head mirroring it. Prints every sensor-bank change of the real
          detector's movement, link changes and faults.

This is the only module that reads the wall clock: it paces simulation time to
real time. The library modules it uses stay deterministic.
"""
import argparse
import sys
import time
from typing import Callable, List, Optional

from smarttraffic.hardware import (HardwareConfig, build_hardware, load_hardware_config,
                                   DEFAULT_CONFIG_FILE, HardwareHub, HardwareDetectorSource,
                                   SignalMirror)
from smarttraffic.hybrid_sensors import format_sensor_id
from smarttraffic.sensors import SensorBank
from smarttraffic.signals import SignalBank, SignalState
from smarttraffic.vehicles import VehicleModel

DT_MS = 100


class Pacer:
    """Keeps simulation time in step with the wall clock (real-time mode only)."""
    def __init__(self, realtime: bool, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        self.realtime, self.clock, self.sleep = realtime, clock, sleep
        self.t0 = clock()

    def wait_until(self, sim_ms: int) -> None:
        if self.realtime:
            delay = self.t0 + sim_ms / 1000 - self.clock()
            if delay > 0:
                self.sleep(delay)


def _pattern_commands(bank: SignalBank, movements: List[str], phase_ms: int) -> None:
    """Repeating test pattern: RED 8 s -> start_green (RY 1 s) -> GREEN 8 s -> end_green
    (FG 3 s, Y, AR) -> RED ... Period 30 s. Only for the smoke test."""
    for m in movements:
        if phase_ms == 0 and bank.can_start_green(m):
            bank.start_green(m)
        elif phase_ms == 9000 and bank.state(m) is SignalState.GREEN:
            bank.end_green(m)


def run_link(cfg: HardwareConfig, seconds: float, out=print, transport_factory=None,
             pacer: Optional[Pacer] = None) -> dict:
    hub = HardwareHub(cfg, transport_factory)
    real = HardwareDetectorSource(hub, cfg.sensors, 0) if cfg.sensors else None
    pacer = pacer or Pacer(True)
    last_state = {}
    t = 0
    try:
        while t < seconds * 1000:
            t += DT_MS
            pacer.wait_until(t)
            edges = real.poll(t) if real else []
            hub.pump(t)
            for d, l in hub.links.items():
                if last_state.get(d) != l.state:
                    last_state[d] = l.state
                    out(f"[{t / 1000:7.1f}s] {d}: {l.state.name}  boot={l.boot_id}  offset={l.offset_ms}")
            for e in edges:
                out(f"[{t / 1000:7.1f}s] REAL {e.movement}:{e.detector} "
                    f"{'OCCUPIED' if e.occupied else 'CLEAR'} at sim {e.time_ms} ms")
    finally:
        status = hub.status()
        hub.close()
    return status


def run_light(cfg: HardwareConfig, out=print, transport_factory=None, pacer: Optional[Pacer] = None,
              cycles: int = 2) -> List[tuple]:
    hub = HardwareHub(cfg, transport_factory)
    mirror = SignalMirror(hub, cfg.signals)
    bank = SignalBank()
    pacer = pacer or Pacer(True)
    movements = list(cfg.signals)
    try:
        for t in range(0, cycles * 30000 + 3000, DT_MS):
            _pattern_commands(bank, movements, t % 30000)
            for m, st in mirror.update(bank):
                out(f"[{t / 1000:7.1f}s] physical {m} <- {st}")
            bank.advance(DT_MS / 1000)
            pacer.wait_until(t + DT_MS)
    finally:
        mirror.blackout()
        hub.close()
    return mirror.log


def run_hybrid(cfg: HardwareConfig, seconds: float, out=print, transport_factory=None,
               pacer: Optional[Pacer] = None, arrivals=()) -> dict:
    start = 0.0
    vm = VehicleModel(list(arrivals), start=start)
    hw = build_hardware(cfg, vm, transport_factory)
    sb = SensorBank(hw.detectors)
    bank = SignalBank(start_time=start)
    pacer = pacer or Pacer(True)
    watch = sorted({m for m, _ in cfg.sensors} | set(cfg.signals))
    mirrored = list(cfg.signals)
    out("Real detectors : " + (", ".join(format_sensor_id(s) for s in cfg.sensors) or "none"))
    out("Virtual        : " + str(16 - len(cfg.sensors)) + " detectors")
    out("Physical heads : " + (", ".join(f"{m}->{t.device}/{t.channel}" for m, t in cfg.signals.items()) or "none"))
    last = {}
    last_link = {}
    last_faults = None
    t = 0
    try:
        while t < seconds * 1000:
            _pattern_commands(bank, mirrored, t % 30000)
            vm.advance(DT_MS / 1000, bank)
            sb.advance(DT_MS / 1000, bank)
            bank.advance(DT_MS / 1000)
            t += DT_MS
            if hw.mirror is not None:
                for m, st in hw.mirror.update(bank):
                    if last.get(("sig", m)) != st:
                        last[("sig", m)] = st
                        out(f"[{t / 1000:7.1f}s] signal {m}: {st}")
            for d, l in hw.hub.links.items():
                if last_link.get(d) != l.state:
                    last_link[d] = l.state
                    out(f"[{t / 1000:7.1f}s] link {d}: {l.state.name}")
            faults = sb.faulted_detectors()
            if faults != last_faults:
                last_faults = faults
                out(f"[{t / 1000:7.1f}s] faulted detectors: "
                    + (", ".join(format_sensor_id(f) for f in faults) or "none"))
            for m in watch:
                s = sb.status(m)
                key = (s.d1, s.d2, s.has_call, int(s.demand_level), s.in_service, s.gap_out)
                if last.get(m) != key:
                    last[m] = key
                    out(f"[{t / 1000:7.1f}s] {m}: D1={'ON ' if s.d1 else 'off'} D2={'ON ' if s.d2 else 'off'} "
                        f"call={s.has_call} level={s.demand_level.name} service={s.in_service} "
                        f"gap_out={s.gap_out} gap_timer={s.gap_timer_ms}")
            pacer.wait_until(t)
    finally:
        status = {"links": hw.hub.status(), "real": hw.real.status() if hw.real else {},
                  "mirror": hw.mirror.status() if hw.mirror else {}, "faults": sb.faulted_detectors()}
        hw.close()
    return status


def _list_ports(out=print) -> None:
    try:
        from serial.tools import list_ports
    except ImportError:
        out("pyserial is not installed (pip install -r requirements.txt)")
        return
    ports = list(list_ports.comports())
    if not ports:
        out("No serial ports found. Is the ESP32 plugged in (and its USB driver installed)?")
    for p in ports:
        out(f"{p.device}\t{p.description}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m smarttraffic.hw_smoke", description=__doc__.split("\n")[0])
    ap.add_argument("--mode", choices=["link", "light", "hybrid"], default="hybrid")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG_FILE))
    ap.add_argument("--port", action="append", default=[],
                    help="COM port; 'COM5' for the only device, or 'esp32_a=COM5' (repeatable)")
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--traffic", default=None, help="add virtual traffic from a scenario, e.g. S4")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--list-ports", action="store_true")
    a = ap.parse_args(argv)
    if a.list_ports:
        _list_ports()
        return 0
    cfg = load_hardware_config(a.config)
    for spec in a.port:
        if "=" in spec:
            dev, port = spec.split("=", 1)
        elif len(cfg.devices) == 1:
            dev, port = next(iter(cfg.devices)), spec
        else:
            ap.error("several devices configured: use --port DEVICE=COMx")
        cfg = cfg.with_port(dev, port)
    try:
        if a.mode == "link":
            status = run_link(cfg, a.seconds)
        elif a.mode == "light":
            run_light(cfg)
            status = None
        else:
            arrivals = ()
            if a.traffic:
                from smarttraffic.traffic_gen import generate_arrivals
                arrivals = generate_arrivals(a.traffic, a.seed, 0, a.seconds).arrivals
            status = run_hybrid(cfg, a.seconds, arrivals=arrivals)
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C). Lamps set dark, port closed.")
        return 0
    if status:
        print("\nFinal status:")
        for k, v in status.items():
            print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
