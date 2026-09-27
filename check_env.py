"""Verify the SmartTrafficLights development environment.

Run from the project folder with the venv active:
    python check_env.py

Checks: Python version, every dependency imports, Pygame can create a surface
and save a frame (headless), Matplotlib can save a figure (headless), SciPy's
paired t-test works, pyserial is importable, and every project module imports.
Writes nothing permanent (uses a temp folder).
"""
import importlib
import os
import sys
import tempfile

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # headless Pygame
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

MODULES = ["traffic_gen", "vehicles", "sensors", "conflicts", "signals",
           "controller_smart", "controller_fixed", "stats", "viz",
           "run_experiments", "make_graphs"]

results = []


def check(name, fn):
    try:
        info = fn()
        results.append((True, name, info or ""))
    except Exception as e:  # report every failure, don't stop at the first
        results.append((False, name, f"{type(e).__name__}: {e}"))


def py_version():
    v = sys.version_info
    if not ((3, 10) <= (v.major, v.minor) <= (3, 13)):
        raise RuntimeError(f"Python {v.major}.{v.minor} found; need 3.10-3.13")
    in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    return f"{sys.version.split()[0]} ({'venv' if in_venv else 'NOT in a venv'})"


def pkg(mod):
    return lambda: getattr(importlib.import_module(mod), "__version__", "ok")


def pygame_headless():
    import pygame
    pygame.display.init()   # video only; audio not needed for checks
    pygame.font.init()
    surf = pygame.Surface((200, 200))
    surf.fill((30, 30, 30))
    pygame.draw.circle(surf, (0, 200, 0), (100, 100), 20)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "frame.png")
        pygame.image.save(surf, p)          # frame capture for video (§13)
        size = os.path.getsize(p)
    pygame.quit()
    return f"surface draw + frame save ok ({size} bytes)"


def matplotlib_headless():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    ax.bar(["FT-A", "FT-B", "Smart"], [1, 2, 3])
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "g.png")
        fig.savefig(p)
        size = os.path.getsize(p)
    plt.close(fig)
    return f"figure save ok ({size} bytes)"


def scipy_ttest():
    from scipy import stats
    r = stats.ttest_rel([1.0, 2.0, 3.0, 4.0], [1.1, 2.3, 2.9, 4.4])
    return f"paired t-test ok (p={r.pvalue:.3f})"


def project_modules():
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    importlib.import_module("smarttraffic")
    for m in MODULES:
        importlib.import_module(f"smarttraffic.{m}")
    return f"{len(MODULES)} modules import"


check("Python version", py_version)
for mod in ["pygame", "matplotlib", "numpy", "pandas", "scipy", "serial", "pytest"]:
    check(f"import {mod}", pkg(mod))
check("Pygame headless", pygame_headless)
check("Matplotlib headless", matplotlib_headless)
check("SciPy stats", scipy_ttest)
check("Project package", project_modules)

for ok, name, info in results:
    print(f"[{'PASS' if ok else 'FAIL'}] {name:22s} {info}")
failed = sum(not ok for ok, _, _ in results)
print(f"\n{len(results) - failed}/{len(results)} checks passed.")
sys.exit(1 if failed else 0)
