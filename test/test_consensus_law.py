"""Pruebas de consensus_law.py (sin ROS):  python3 -m pytest test/ -v"""
from robcoop_lab import check_p1, kinsim


def test_laplaciano():
    ok, msg = check_p1.check_laplacian()
    assert ok, msg


def test_lambda2():
    ok, msg = check_p1.check_lambda2()
    assert ok, msg


def test_ley_de_consenso():
    ok, msg = check_p1.check_velocity()
    assert ok, msg


def test_reunion_converge():
    r = kinsim.simulate('completo', 'reunion', k=0.1, n=4)
    assert r['converged'], f'no converge: error final {r["final_error"]:.3f} m'
