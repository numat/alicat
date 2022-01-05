"""Mock for offline testing of `FlowController`s."""

from random import random, choice
from time import sleep
from unittest.mock import MagicMock


class FlowController(MagicMock):
    """Mocks an Alicat MFC for offline testing."""

    def __init__(self, *args, **kwargs):
        """Initialize the device client."""
        self.controlpoint = choice(['flow', 'pressure'])
        self.setpoint = 10
        self.gas = 'N2'

    def get(self):
        """Get the current state of the flow controller."""
        sleep(random() * 0.25)
        return {
            'setpoint': self.setpoint,
            'control_point': self.control_point,
            'gas': self.gas,
            'mass_flow': self.setpoint * (0.95 + 0.1 * random()),
            'pressure': random() * 50.0,
            'temperature': random() * 50.0,
            'total_flow': 0.0,
            'volumetric_flow': 0.0,
        }

    def _set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def _set_control_point(self, control_point):
        self.control_point = control_point

    def _get_control_point(self):
        return self.control_point

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
