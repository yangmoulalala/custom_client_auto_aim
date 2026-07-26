"""Tests for MQTT client ID launch validation."""

import pytest

from rm_mqtt.launch_support import normalize_client_id


@pytest.mark.parametrize(
    ('value', 'expected'),
    [('3', '3'), ('0003', '3'), (' 42 ', '42')],
)
def test_normalize_client_id_accepts_positive_decimal(value, expected):
    assert normalize_client_id(value) == expected


@pytest.mark.parametrize('value', ['', 'abc', '0', '-1', '3.0'])
def test_normalize_client_id_rejects_invalid_values(value):
    with pytest.raises(ValueError, match='positive integer'):
        normalize_client_id(value)
