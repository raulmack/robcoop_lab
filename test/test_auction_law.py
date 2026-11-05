"""Pruebas de auction_law.py (sin ROS):  python3 -m pytest test/test_auction_law.py -v"""
from robcoop_lab import check_p2


def test_route_length():
    ok, msg = check_p2.check_route_length()
    assert ok, msg


def test_best_insertion():
    ok, msg = check_p2.check_best_insertion()
    assert ok, msg


def test_choose_winner():
    ok, msg = check_p2.check_choose_winner()
    assert ok, msg


def test_hungarian():
    ok, msg = check_p2.check_hungarian()
    assert ok, msg


def test_subasta_disposiciones():
    ok, msg = check_p2.check_auction(verbose=False)
    assert ok, msg
