from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from paus_bringup.ultrasound_launch import ultrasound_actions


def test_receiver_is_opt_in_and_lan_override_is_passed() -> None:
    context = LaunchContext()
    actions = ultrasound_actions()
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    node = next(action for action in actions if isinstance(action, Node))
    assert not node.condition.evaluate(context)
    parameters = node._Node__parameters
    assert evaluate_parameters(context, parameters)[0] == {
        "listen_host": "127.0.0.1", "listen_port": 55001, "stale_timeout_s": 1.0,
    }
    context.launch_configurations["enable_ultrasound"] = "true"
    context.launch_configurations["ultrasound_listen_host"] = "172.16.33.132"
    assert node.condition.evaluate(context)
    assert evaluate_parameters(context, parameters)[0]["listen_host"] == "172.16.33.132"


@pytest.mark.parametrize("name", ["ui", "online_stack"])
def test_operator_launch_contains_exactly_one_optional_receiver(name) -> None:
    spec = importlib.util.spec_from_file_location(f"test_{name}_launch", ROOT / "launch" / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    receivers = [action for action in description.entities if isinstance(action, Node)]
    assert len(receivers) == 1
    assert not receivers[0].condition.evaluate(context)
    assert context.launch_configurations["enable_ultrasound"] == "false"
