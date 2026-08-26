"""The parser is the dangerous part: a model reply that half-parses must not become a confident
wrong BlockageEvent. A wrong 'kind' sends the reasoner hunting a control that is not there, and
the trial then records a REASONING failure that was really a PERCEPTION failure."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bringup"))
from ask_blockage import KINDS, parse


def _j(**kw):
    return json.dumps(kw)


def test_clean_reply():
    r = parse(_j(obstruction="a closed glass door", category="door",
                 passable_without_operating_something=False))
    assert r["kind"] == "door" and r["blocked"] is True
    assert "glass door" in r["description"]


def test_fenced_json_is_unwrapped():
    """Models wrap JSON in ```json fences constantly."""
    raw = '```json\n' + _j(obstruction="lift doors", category="elevator",
                           passable_without_operating_something=False) + '\n```'
    assert parse(raw)["kind"] == "elevator"


def test_prose_around_the_json_is_tolerated():
    raw = "Sure! Here is the result:\n" + _j(
        obstruction="a door", category="door",
        passable_without_operating_something=False) + "\nHope that helps."
    assert parse(raw)["kind"] == "door"


def test_unknown_category_becomes_empty_not_a_guess():
    """'other', 'gate', anything unexpected -> unclassified. Never coerced to 'door'."""
    for cat in ("other", "gate", "turnstile", "DOOR-ish", ""):
        r = parse(_j(obstruction="x", category=cat,
                     passable_without_operating_something=False))
        assert r["kind"] in KINDS
        if cat.lower() not in ("door", "elevator"):
            assert r["kind"] == ""


def test_non_json_fails_closed_and_keeps_the_text():
    r = parse("I think there is a door in the way, press the button.")
    assert r["kind"] == "" and r["blocked"] is True
    assert "door in the way" in r["description"]
    assert r["note"]


def test_broken_json_fails_closed():
    r = parse('{"obstruction": "a door", "category": "door",')
    assert r["kind"] == "" and r["blocked"] is True and r["note"]


def test_empty_reply_fails_closed():
    r = parse("")
    assert r["blocked"] is True and r["kind"] == ""


def test_only_an_explicit_true_clears_the_blockage():
    """Driving on because a model omitted a field is the wrong way to be wrong."""
    assert parse(_j(obstruction="clear", category="other",
                    passable_without_operating_something=True))["blocked"] is False
    for bad in (None, "yes", "true", 1, {}, []):
        r = parse(_j(obstruction="clear", category="other",
                     passable_without_operating_something=bad))
        assert r["blocked"] is True, f"{bad!r} must not clear the blockage"
    assert parse(_j(obstruction="clear", category="other"))["blocked"] is True


def test_description_is_bounded():
    r = parse(_j(obstruction="x"*5000, category="door",
                 passable_without_operating_something=False))
    assert len(r["description"]) <= 400


def test_parse_never_raises():
    for raw in ("", "{", "}", "{}", "null", "[]", "{'single': 'quotes'}", "```", "```json```"):
        parse(raw)
