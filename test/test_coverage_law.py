"""Pruebas de coverage_law.py (sin ROS):  python3 -m pytest test/test_coverage_law.py -v"""
from robcoop_lab import check_p3


def test_voronoi_mask():
    ok, msg = check_p3.check_mask()
    assert ok, msg


def test_centroid():
    ok, msg = check_p3.check_centroid()
    assert ok, msg


def test_coverage_velocity():
    ok, msg = check_p3.check_velocity()
    assert ok, msg


def test_coverage_cost():
    ok, msg = check_p3.check_cost()
    assert ok, msg


def test_lloyd_llega_al_coste_esperado():
    ok, msg = check_p3.check_lloyd(verbose=False)
    assert ok, msg
