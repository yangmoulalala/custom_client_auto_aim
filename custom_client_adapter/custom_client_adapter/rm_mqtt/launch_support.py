"""Validation helpers used before the ROS node is launched."""


def normalize_client_id(value: str) -> str:
    """Return a canonical positive decimal MQTT client ID."""
    stripped = value.strip()
    if stripped.isdecimal() and int(stripped) > 0:
        return str(int(stripped))
    raise ValueError('client_id must be a positive integer')
