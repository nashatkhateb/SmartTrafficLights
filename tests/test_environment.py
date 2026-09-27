"""Environment smoke test (pytest). Real unit tests are added per study §16."""
import importlib
import sys

import pytest

MODULES = ["traffic_gen", "vehicles", "sensors", "conflicts", "signals",
           "controller_smart", "controller_fixed", "stats", "viz",
           "run_experiments", "make_graphs"]


def test_python_version():
    assert (3, 10) <= sys.version_info[:2] <= (3, 13)


@pytest.mark.parametrize("mod", ["pygame", "matplotlib", "numpy", "pandas", "scipy", "serial"])
def test_dependency_imports(mod):
    importlib.import_module(mod)


@pytest.mark.parametrize("mod", MODULES)
def test_project_module_imports(mod):
    importlib.import_module(f"smarttraffic.{mod}")
