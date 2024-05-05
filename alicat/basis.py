"""Python driver for Alicat BASIS mass flow devices, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2023 NuMat Technologies
"""
from __future__ import annotations

import asyncio
from typing import Any, ClassVar

import nest_asyncio

from .util import Client, SerialClient, _is_float

nest_asyncio.apply()
class FlowMeter:
    """Python driver for BASIS Flow Meters.

    [Reference](https://www.alicat.com/wp-content/documents/manuals/DOC-MANUAL-BASIS2.pdf).

    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial.
    """

    # A dictionary that maps port names to a tuple of connection
    # objects and the refcounts
    open_ports: ClassVar[dict[int, tuple]] = {}
    gases: ClassVar[list] = ['Air', 'Ar', 'CO2', 'N2', 'O2', 'N2O', 'H2', 'He', 'CH4']

    def __init__(self, address: str = '/dev/ttyUSB0', unit: str = 'A', baudrate: int = 38400, **kwargs: Any) -> None:
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port or TCP address:port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
            baudrate: The baud rate of the device. Default 38400.
        """
        self.hw: Client = SerialClient(address=address, baudrate=baudrate, **kwargs)

        self.unit = unit
        self.keys = ['temperature', 'flow', 'totalizer', 'setpoint',
                     'valve drive', 'gas']
        self.open = True
        self.firmware: str | None = None

        ## check if device is basis. if not, raise error
        async def _is_basis() -> None:
            self.control_point = await self._check_basis()

        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(_is_basis())
        loop.run_until_complete(tasks)

    async def _check_basis(self) -> None:
        """Check if connected device is a BASIS device. If not, raise error."""
        check = await self._write_and_read(f'{self.unit} SN')
        if check.split(" ")[-1][0] != 'B':
            raise OSError('This is not a BASIS device. Please use the regular serial driver.')

    async def __aenter__(self, *args: Any) -> FlowMeter:
        """Provide async enter to context manager."""
        return self

    async def __aexit__(self, *args: Any) -> None:
        """Provide async exit to context manager."""
        await self.close()
        return

    @classmethod
    async def is_connected(cls, port: str, unit: str = 'A') -> bool:
        """Return True if the specified port is connected to this device.

        This class can be used to automatically identify ports with connected
        Alicats. Iterate through all connected interfaces, and use this to
        test. Ports that come back True should be valid addresses.

        Note that this distinguishes between `FlowController` and `FlowMeter`.
        """
        is_device = False
        try:
            device = cls(port, unit)

            try:
                c = await device.get()
                if cls.__name__ == 'FlowMeter':
                    assert c
                    assert 'setpoint' not in device.keys
                elif cls.__name__ == 'FlowController':
                    assert c
                    assert 'setpoint' in device.keys
                else:
                    raise NotImplementedError('Must be meter or controller.')
                is_device = True
            finally:
                await device.close()
        except Exception:
            pass
        return is_device

    def _test_controller_open(self) -> None:
        """Raise an IOError if the FlowMeter has been closed.

        Does nothing if the meter is open and good for read/write
        otherwise raises an IOError. This only checks if the meter
        has been closed by the FlowMeter.close method.
        """
        pass
        if not self.open:
            raise OSError(f"The FlowMeter with unit ID {self.unit} and \
                           port {self.hw.address} is not open")

    async def _write_and_read(self, command: str) -> str | None:
        """Wrap the communicator request, to call _test_controller_open() before any request."""
        self._test_controller_open()
        return await self.hw._write_and_read(command)

    async def get(self) -> dict:
        """Get the current state of the flow controller.

        From the Alicat mass flow controller documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Mass flow (in units specified at time of order)
         * Total flow
         * Currently selected gas

        Args:
            retries: Number of times to re-attempt reading. Default 2.
        Returns:
            The state of the flow controller, as a dictionary.

        """
        command = f'{self.unit}'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not read values")
        spl = line.split()
        unit, values = spl[0], spl[1:]

        # Over range errors for mass, volume, pressure, and temperature
        # Explicitly silenced because I find it redundant.
        while values[-1].upper() in ['MOV', 'VOV', 'POV', 'TOV']:
            del values[-1]
        if unit != self.unit:
            raise ValueError("Flow controller unit ID mismatch.")
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-3]
        return {k: (float(v) if _is_float(v) else v)
                for k, v in zip(self.keys, values)}

    async def set_gas(self, gas: str | int) -> None:
        """Set the gas type.

        Args:
            gas: The gas type, as a string or integer. Supported strings are:
                'Air', 'Ar', 'CO2', 'N2', 'O2', 'N2O', 'H2', 'He', 'CH4'
        """
        if isinstance(gas, str):
            if gas not in self.gases:
                raise ValueError(f"{gas} not supported!")
            gas_number = self.gases.index(gas)
        else:
            gas_number = gas
        command = f'{self.unit}GS {gas_number}'
        res = await self._write_and_read(command)

        if gas != res.split(" ")[-3]:
            raise OSError("Cannot set gas.")

    async def tare(self) -> None:
        """Tare flow."""
        command = f'{self.unit} V'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to tare flow.")

    async def reset_totalizer(self) -> None:
        """Reset the totalizer."""
        command = f'{self.unit}T'
        await self._write_and_read(command)

    async def get_firmware(self) -> str:
        """Get the device firmware version."""
        if self.firmware is None:
            command = f'{self.unit}VE'
            self.firmware = await self._write_and_read(command)
        if not self.firmware:
            raise OSError("Unable to get firmware.")
        return self.firmware

    async def flush(self) -> None:
        """Read all available information. Use to clear queue."""
        self._test_controller_open()
        await self.hw._clear()

    async def close(self) -> None:
        """Close the flow meter. Call this on program termination.

        Also closes the serial port if no other FlowMeter object has
        a reference to the port.
        """
        if not self.open:
            return
        await self.hw.close()
        self.open = False


class FlowController(FlowMeter):
    """Python driver for Alicat Flow Controllers.

    [Reference](http://www.alicat.com/products/mass-flow-meters-and-
    controllers/mass-flow-controllers/).

    This communicates with the flow controller over a USB or RS-232/RS-485
    connection using pyserial.

    To set up your Alicat flow controller, power on the device and make sure
    that the "Input" option is set to "Serial".
    """

    def __init__(self, address: str='/dev/ttyUSB0', unit: str='A', baudrate: int = 38400, **kwargs: Any) -> None:
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
            baudrate: The baud rate of the device. Default 38400.
        """
        FlowMeter.__init__(self, address, unit, baudrate, **kwargs)

    async def __aenter__(self, *args: Any) -> FlowController:
        """Provide async enter to context manager."""
        return self

    async def _write_and_read(self, command: str) -> str | None:
        """Wrap the communicator request.

        (1) Ensure _init_task is called once before the first request
        (2) Call _test_controller_open() before any request
        """
        self._test_controller_open()
        return await self.hw._write_and_read(command)

    async def get(self) -> dict:
        """Get the current state of the flow controller.

        From the Alicat mass flow controller documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Volumetric flow (in units specified at time of order)
         * Mass flow (in units specified at time of order)
         * Flow setpoint (in units of control point)
         * Flow control point (either 'flow' or 'pressure')
         * Total flow (only on models with the optional totalizer function)
         * Currently selected gas

        Returns:
            The state of the flow controller, as a dictionary.
        """
        state = await super().get()
        if state is None:
            return None
        return state

    async def get_totalizer_batch(self) -> str:
        """Get the totalizer batch volume.

        Returns:
            line: Remaining batch volume
        """
        remaining = await self._write_and_read(f'{self.unit}DV 64')
        current = await self._write_and_read(f'{self.unit}TB')
        if current == '?' or remaining == '?':
            raise OSError("Unable to read totalizer batch volume.")
        return f'Totalizer currently set to {current}. Remaining volume {remaining.split(" ")[-1]}'

    async def set_totalizer_batch(self, batch_volume: float, batch: int = 1, units: str = 'default') -> None:
        """Set the totalizer batch volume.

        Args:
            batch_volume: Target batch volume, in same units as units
                on device
        """
        command = f'{self.unit}TB {batch_volume}'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to set totalizer batch volume. Check if volume is out of range for device.")

    async def hold(self, percentage: float) -> None:
        """Override command to issue a valve hold at a certain percentage of full drive.

        Args:
            percentage : Percentage of full valve drive
        """
        command = f'{self.unit}HPUR {percentage}' ## need the space otherwise won't work
        await self._write_and_read(command)

    async def cancel_hold(self) -> None:
        """Cancel valve hold."""
        command = f'{self.unit}C'
        await self._write_and_read(command)

    async def get_pid(self) -> dict:
        """Read the current PID values on the controller.

        Values include the P value and I value.
        Values returned as a dictionary.
        """
        self.pid_keys = ['P', 'I']

        command = f'{self.unit}LCG'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not get PID values.")
        spl = line.split()
        return dict(zip(self.pid_keys, spl[1:]))

    async def set_pid(self, p: int | None=None,
                            i: int | None=None,) -> None:
        """Set specified PID parameters.

        Args:
            p: Proportional gain
            i: Integral gain. Only used in PD2I loop type.
        """
        current = await self.get_pid()
        p = [p if p is not None else current['P']][0]
        i = [i if i is not None else current['I']][0]

        command = f'{self.unit}LCG {p} {i}'
        await self._write_and_read(command)

    async def set_setpoint(self, setpoint: float) -> None:
        """Set the target setpoint."""
        command = f'{self.unit}S {setpoint:.2f}'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not set setpoint.")
        try:
            current = float(line.split()[4])
        except IndexError:
            current = None
        if current is not None and abs(current - setpoint) > 0.01:
            raise OSError("Could not set setpoint.")
