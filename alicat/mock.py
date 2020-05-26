"""Mock for offline testing of serial and tcp FlowControllers."""

import asyncio
from random import random, choice
from unittest.mock import MagicMock

from alicat.serial import FlowController as RealSerialFC
from alicat.tcp import FlowController as RealTCPFC


class SerialFlowController(MagicMock):
    """Mocks an Alicat MFC over TCP for offline testing."""

    def __init__(self, *args, **kwargs):
        """Initialize the device client."""
        super().__init__(spec=RealSerialFC)
        self.controlpoint = choice(['flow', 'pressure'])
        self.setpoint = 10
        self.gas = 'N2'

    def get(self):
        """Get the current state of the flow controller."""
        return {
            'setpoint': self.setpoint,  # Setpoint, either mass flow rate or pressure
            'control_point': self.controlpoint,  # Either 'flow' or 'pressure'
            'gas': self.gas,  # Can be any option in `flow_controller.gases`
            'mass_flow': self.setpoint * (0.95 + 0.1 * random()),  # Mass flow (in units specified at time of purchase)
            'pressure': random() * 50.0,  # Pressure (normally in psia)
            'temperature': random() * 50.0,  # Temperature (normally in C)
            'total_flow': 0.0,  # Optional. If totalizer function purchased, will be included
            'volumetric_flow': 0.0  # Volumetric flow (in units specified at time of purchase)
        }

    def _set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def _set_control_point(self, controlpoint):
        self.controlpoint = controlpoint

    def _get_control_point(self):
        return self.controlpoint

    def set_flow_rate(self, flowrate):
        """Set the target setpoint."""
        self._set_control_point('flow')
        self._set_setpoint(flowrate)

    def set_gas(self, gas):
        """Set the gas type."""
        self.gas = gas

    def set_pressure(self, pressure):
        """Set the target pressure."""
        self._set_control_point('pressure')
        self._set_setpoint(pressure)


class TCPFlowController(SerialFlowController):
    """Mocks an Alicat MFC over TCP for offline testing."""

    def __init__(self, *args, **kwargs):
        """Initialize the device client."""
        super().__init__(spec=RealTCPFC)
        self.controlpoint = choice(['flow', 'pressure'])
        self.setpoint = 10
        self.gas = 'N2'

    async def get(self):
        """Get the current state of the flow controller."""
        await asyncio.sleep(random() * 0.25)
        return {
            'setpoint': self.setpoint,  # Setpoint, either mass flow rate or pressure
            'control_point': self.controlpoint,  # Either 'flow' or 'pressure'
            'gas': self.gas,  # Can be any option in `flow_controller.gases`
            'mass_flow': self.setpoint * (0.95 + 0.1 * random()),  # Mass flow (in units specified at time of purchase)
            'pressure': random() * 50.0,  # Pressure (normally in psia)
            'temperature': random() * 50.0,  # Temperature (normally in C)
            'total_flow': 0.0,  # Optional. If totalizer function purchased, will be included
            'volumetric_flow': 0.0  # Volumetric flow (in units specified at time of purchase)
        }

    async def _set_setpoint(self, setpoint):
        self.setpoint = setpoint

    async def _set_control_point(self, controlpoint):
        self.controlpoint = controlpoint

    async def _get_control_point(self):
        return self.controlpoint

    async def set_flow_rate(self, flowrate):
        """Set the target setpoint."""
        await self._set_control_point('flow')
        await self._set_setpoint(flowrate)

    async def set_gas(self, gas):
        """Set the gas type."""
        self.gas = gas

    async def set_pressure(self, pressure):
        """Set the target pressure."""
        await self._set_control_point('pressure')
        await self._set_setpoint(pressure)
