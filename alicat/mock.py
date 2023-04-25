"""Mock for offline testing of `FlowController`s."""

from random import choice, random
from time import sleep
from typing import Any, Dict, Union
from unittest.mock import MagicMock

from .driver import FlowController as RealFlowController


class AsyncClientMock(MagicMock):
    """Magic mock that works with async methods."""

    async def __call__(self, *args, **kwargs): # type: ignore [no-untyped-def]
        """Convert regular mocks into into an async coroutine."""
        return super().__call__(*args, **kwargs)


class FlowController(RealFlowController):
    """Mocks an Alicat MFC for offline testing."""

    def __init__(self, address: str, unit: str = 'A', *args: Any, **kwargs: Any) -> None:
        """Initialize the device client."""
        self.hw = AsyncClientMock()
        self.open = True
        self.control_point: str = choice(['flow', 'pressure'])
        self.state: Dict[str, Union[str, float]] = {
            'setpoint': 10,
            'gas': 'N2',
            'mass_flow': 10 * (0.95 + 0.1 * random()),
            'pressure': random() * 50.0,
            'temperature': random() * 50.0,
            'total_flow': 0.0,
            'unit': unit,
            'volumetric_flow': 0.0,
        }
        self.unit: str = unit
        self.button_lock: bool = False
        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'setpoint', 'gas']
        self.firmware: str = '6v21.0-R22 Nov 30 2016,16:04:20'

    async def get(self) -> Dict[str, Union[str, float]]:
        """Return the full state."""
        sleep(random() * 0.25)
        return self.state

    async def _set_setpoint(self, setpoint: float) -> None:
        """Set the target setpoint."""
        self.state['setpoint'] = setpoint

    async def _set_control_point(self, control_point: str) -> None:
        """Set the control point, either 'flow' or 'pressure'."""
        self.control_point = control_point

    async def _get_control_point(self) -> str:
        """Return the control point, either 'flow' or 'pressure'."""
        return self.control_point

    async def set_flow_rate(self, flowrate: float) -> None:
        """Set the flowrate setpoint."""
        await self._set_control_point('flow')
        await self._set_setpoint(flowrate)

    async def set_gas(self, gas: Union[int, str]) -> None:
        """Set the gas type."""
        if isinstance(gas, int):
            gas = self.gases[gas]
        self.state['gas'] = gas

    async def set_pressure(self, pressure: float) -> None:
        """Set the pressure setpoint."""
        await self._set_control_point('pressure')
        await self._set_setpoint(pressure)

    async def lock(self) -> None:
        """Lock the buttons."""
        self.button_lock = True

    async def unlock(self) -> None:
        """Unlock the buttons."""
        self.button_lock = False
