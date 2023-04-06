"""Test the driver responds with correct data."""
from random import uniform
from unittest import mock

import pytest

from alicat import command_line
# from alicat.driver import FlowController
from alicat.mock import FlowController

ADDRESS = '/dev/tty.usbserial-FTCJ5EK9'


@pytest.mark.parametrize('unit', ['A', 'B'])
@mock.patch('alicat.FlowController', FlowController)
def test_driver_cli(capsys, unit):
    """Confirm the commandline interface works with different unit IDs."""
    command_line([ADDRESS, '--unit', unit])
    captured = capsys.readouterr()
    assert ("mass_flow" in captured.out)


async def test_flow_setpoint_roundtrip():
    """Confirm that setting/getting flowrates works."""
    async with FlowController(ADDRESS) as device:
        flow_sp = round(uniform(0.01, 0.1), 2)
        await device.set_flow_rate(flowrate=flow_sp)
        # assert flow_sp == await device.get_flow_rate()
        result = await device.get()
        assert flow_sp == result['setpoint']
