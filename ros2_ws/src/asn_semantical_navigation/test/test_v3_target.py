"""Offline tests for target command parsing and frame confirmation."""
import pytest

from asn_semantical_navigation.search_commands import parse_target
from asn_semantical_navigation.target_filter import TargetFilter


def test_prompt_parses_a_class_without_using_color():
    assert parse_target('Find a black backpack') == 'backpack'
    assert parse_target('please find the coffee cup') == 'cup'
    assert parse_target('look for a hiker') == 'person'


def test_prompt_rejects_unsupported_or_ambiguous_targets():
    with pytest.raises(ValueError):
        parse_target('go to the west room')
    with pytest.raises(ValueError):
        parse_target('find a cup and a backpack')
    with pytest.raises(ValueError):
        parse_target('find a backpack', ['cup'])


def test_detector_needs_consecutive_frames():
    filt = TargetFilter(required_frames=3, min_confidence=0.55)
    filt.set_target('backpack')
    assert not filt.update({'backpack': 0.8})
    assert not filt.update({'cup': 0.99})  # Miss resets streak.
    assert not filt.update({'backpack': 0.7})
    assert not filt.update({'backpack': 0.7})
    assert filt.update({'backpack': 0.7})
    assert filt.set_target('cup')
    assert not filt.confirmed
    assert not filt.update({'cup': 0.8})
