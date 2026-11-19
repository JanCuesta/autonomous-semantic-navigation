"""Deterministic tests for independent rolling RGB-D evidence."""

import pytest

from asn_semantical_navigation.core.rgbd_observation import (
    Observation3D, ObservationHistory)


def observation(index, xyz=(1.0, 2.0, 0.5), *, target='chair',
                identity='', rgb_ns=None, depth_ns=None):
    base = index * 1_000_000_000
    return Observation3D(
        target=target,
        identity=identity,
        rgb_stamp_ns=base + 10 if rgb_ns is None else rgb_ns,
        depth_stamp_ns=base + 20 if depth_ns is None else depth_ns,
        xyz=xyz)


def test_missing_depth_or_tf_failure_retains_prior_observations():
    history = ObservationHistory()
    history.add(observation(1))
    before = list(history.observations)
    # Rejected RGB-only and TF candidates never enter the history.
    assert history.observations == before
    history.add(observation(2, xyz=(1.05, 2.0, 0.5)))
    assert len(history.observations) == 2


def test_one_outlier_is_rejected_and_two_start_new_cluster():
    history = ObservationHistory(max_jitter_m=0.45, outliers_to_reset=2)
    history.add(observation(1))
    history.add(observation(2, xyz=(1.1, 2.0, 0.5)))
    decision, details = history.add(observation(3, xyz=(2.0, 2.0, 0.5)))
    assert decision == 'outlier'
    assert details['consecutive'] == 1
    assert len(history.observations) == 2
    decision, details = history.add(observation(4, xyz=(2.1, 2.0, 0.5)))
    assert decision == 'cluster_reset'
    assert details['previous_count'] == 2
    assert history.observations == [observation(4, xyz=(2.1, 2.0, 0.5))]


def test_three_consistent_independent_positions_confirm():
    history = ObservationHistory(required=3, window_s=4.0)
    for index, x in enumerate((1.0, 1.1, 0.9), start=1):
        assert history.add(observation(index, xyz=(x, 2.0, 0.5)))[0] == 'accepted'
    assert history.confirmed
    assert history.stable_xyz() == pytest.approx((1.0, 2.0, 0.5))


def test_observations_expire_individually():
    history = ObservationHistory(required=3, window_s=4.0)
    for index in (1, 3, 5):
        history.add(observation(index))
    expired = history.expire(5_500_000_020)
    assert [item.depth_stamp_ns for item in expired] == [1_000_000_020]
    assert len(history.observations) == 2
    assert not history.confirmed


def test_reused_depth_and_rgb_do_not_count_as_independent():
    history = ObservationHistory(required=3)
    first = observation(1)
    history.add(first)
    assert history.add(observation(
        2, depth_ns=first.depth_stamp_ns))[0] == 'duplicate_depth'
    assert history.add(observation(
        3, rgb_ns=first.rgb_stamp_ns))[0] == 'duplicate_rgb'
    assert len(history.observations) == 1
    assert not history.confirmed


def test_target_and_identity_change_are_explicit_reset_decisions():
    history = ObservationHistory()
    history.add(observation(1, identity='track-1'))
    assert history.add(observation(
        2, target='bottle', identity='track-1'))[0] == 'target_changed'
    assert history.add(observation(
        2, identity='track-2'))[0] == 'identity_changed'
    assert len(history.observations) == 1


def test_clock_epoch_reset_clears_all_evidence():
    history = ObservationHistory()
    history.add(observation(1))
    history.add(observation(2))
    assert history.clear() == 2
    assert not history.observations


def test_spatial_agreement_is_pairwise_not_only_centroid_based():
    history = ObservationHistory(max_jitter_m=0.45)
    history.add(observation(1, xyz=(0.0, 0.0, 0.0)))
    history.add(observation(2, xyz=(0.4, 0.0, 0.0)))
    decision, _ = history.add(observation(3, xyz=(0.7, 0.0, 0.0)))
    assert decision == 'outlier'
