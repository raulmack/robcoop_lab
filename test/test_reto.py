"""Pruebas del reto final (sin ROS):  python3 -m pytest test/test_reto.py -v"""
from robcoop_lab import check_p4, kinsim_p4
from robcoop_lab import mission_core as mc


def test_configuracion_valida():
    ok, msg, _ = check_p4.check_config()
    assert ok, msg


def test_las_tres_leyes():
    for name, functions in check_p4.LAWS:
        ok, msg = check_p4.check_law(functions)
        assert ok, f'{name}: {msg}'


def test_mision_completa():
    r = kinsim_p4.simulate(layout='A', failure='tb3_1')
    b = r['scoreboard']
    assert b['complete'] and b['total'] >= 100.0 and not r['collision'], \
        mc.format_scoreboard(r['phases'], b, r['min_dist'])
