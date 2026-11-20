"""v0.1.56 D9: the dock face's indicator votes in the OCS map's hover (map.html voteText)."""
from .conftest import ROOT


def test_map_hover_shows_votes():
    s = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "function voteText(en)" in s and "votes G ${v.green || 0} · R ${v.red || 0}" in s
    assert "${en.colour}${voteText(en)}" in s
    pm = (ROOT / "optiprimex" / "usv" / "perception_map.py").read_text()
    assert 'b["votes"] = dict(e.ind_votes)' in pm
