"""K5: the toast and press-colour logic without Tk (a fake clock)."""
from __future__ import annotations

from optiprimex.tools.tkconsole.toasts import MAX_VISIBLE, PressBook, ToastBook, verdict


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_sent_then_ok_then_gone_after_3_s_from_the_final_state():
    c = Clock()
    b = ToastBook(clock=c)
    i = b.add("declare uav on")
    assert b.items[i].level == "sent" and b.items[i].text == "⟶ declare uav on · sent"
    c.t += 2.5
    b.resolve(i, True, "intent accepted")
    assert b.items[i].text == "✓ declare uav on" and b.items[i].detail == "intent accepted"
    c.t += 2.9
    assert i in [t.id for t in b.visible()[0]]                                   # the 3 s count from the answer, not the press
    c.t += 0.2
    assert b.visible()[0] == []


def test_refused_keeps_the_why_and_the_hint():
    c = Clock()
    b = ToastBook(clock=c)
    i = b.add("start_task")
    b.resolve(i, False, "START_TASK is accepted in AUTO_HOLD only", "abort first")
    t = b.items[i]
    assert t.level == "bad" and t.text == "✗ start_task — START_TASK is accepted in AUTO_HOLD only" and t.detail == "abort first"
    assert b.log[-1][2].startswith("✗ start_task — START_TASK") and "abort first" in b.log[-1][2]


def test_no_answer_turns_amber_at_10_s_and_still_resolves():
    c = Clock()
    b = ToastBook(clock=c)
    i = b.add("stop_reset")
    c.t += 9.9
    b.tick()
    assert b.items[i].level == "sent"
    c.t += 0.2
    b.tick()
    assert b.items[i].level == "wait" and b.items[i].text == "… stop_reset — no answer yet"
    c.t += 60
    assert i in b.items                                                          # waits as long as the worker does
    b.resolve(i, False, "no answer: timeout")
    assert b.items[i].text == "✗ stop_reset — no answer: timeout"


def test_six_toasts_show_five_and_plus_one_newest_on_top():
    b = ToastBook(clock=Clock())
    ids = [b.add(f"w{n}") for n in range(6)]
    shown, more = b.visible()
    assert len(shown) == MAX_VISIBLE == 5 and more == 1 and shown[0].id == ids[-1]
    b.dismiss(ids[-1])                                                           # a click dismisses
    assert b.visible()[1] == 0


def test_a_state_transition_toast():
    c = Clock()
    b = ToastBook(clock=c)
    i = b.event("ok", "REC on · run_20261111_0914")
    assert b.items[i].final and b.items[i].level == "ok"
    c.t += 3.1
    assert b.visible()[0] == []


def test_the_pressed_button_colour():
    c = Clock()
    p = PressBook(clock=c)
    assert p.look("k") == ""
    p.press("k")
    assert p.look("k") == "sending"
    c.t += 15
    assert p.look("k") == "sending"                                              # stays amber until the answer
    p.resolve("k", False)
    assert p.look("k") == "bad"
    c.t += 3.0
    assert p.look("k") == ""


def test_the_verdict_reads_the_answer_like_the_page():
    assert verdict({"ok": True, "lines": ["intent accepted", "  ack DONE state=AUTO_HOLD"]}) == (True, "intent accepted", "")
    assert verdict({"ok": False, "lines": ["unknown command uav (...)"]})[0] is False
    assert verdict({"ok": True, "lines": ["busy: the previous command is still waiting for its acks"]})[0] is False
    assert verdict({"ok": False, "lines": ["no answer: timeout"], "error": "timeout"}) == (False, "no answer: timeout", "")
    assert verdict({"ok": True, "acked": 1}) == (True, "ok", "")                 # /ack
    assert verdict(None) == (False, "no answer", "")
