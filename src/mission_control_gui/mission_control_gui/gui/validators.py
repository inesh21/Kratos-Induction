"""Checks if a waypoint's X and Y fields are valid numbers."""


def validate_waypoint(x_text, y_text):
    """Returns (True, x, y) if valid, or (False, None, None) if not."""
    if not x_text.strip() or not y_text.strip():
        return False, None, None

    try:
        x = float(x_text)
        y = float(y_text)
    except ValueError:
        return False, None, None

    return True, x, y