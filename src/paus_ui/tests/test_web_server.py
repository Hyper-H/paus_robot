from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
if str(UI_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(UI_PACKAGE_ROOT))

from paus_ui.web_server import _parse_confirmed_flag


def test_parse_confirmed_flag_rejects_string_values() -> None:
    assert _parse_confirmed_flag(None) is False
    assert _parse_confirmed_flag({"confirmed": False}) is False
    assert _parse_confirmed_flag({"confirmed": True}) is True

    with pytest.raises(ValueError, match="JSON boolean"):
        _parse_confirmed_flag({"confirmed": "false"})
