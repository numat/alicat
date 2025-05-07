"""Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2024 Alex Ruddick
Copyright (C) 2023 NuMat Technologies
"""
from __future__ import annotations

import asyncio
from typing import Any, ClassVar

from .util import Client, SerialClient, TcpClient, _is_float


class FlowMeter:
    """Python driver for Alicat Flow Meters.

    [Reference](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).

    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial, or an Ethernet <-> serial converter.
    """

    # mapping of port names to a tuple of Client objects and their refcounts
    open_ports: ClassVar[dict[str, tuple[Client, int]]] = {}
    gases: ClassVar[list] = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                             'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                             'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                             'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                             'P-5']

    def __init__(self, address: str = '/dev/ttyUSB0', unit: str = 'A', **kwargs: Any) -> None:
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port or TCP address:port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
        """
        if address.startswith('/dev') or address.startswith('COM'):  # serial
            if address in FlowMeter.open_ports:
                # Reuse existing connection
                self.hw, refcount = FlowMeter.open_ports[address]
                FlowMeter.open_ports[address] = (self.hw, refcount + 1)
            else:
                # Open a new connection and store it
                self.hw: Client = SerialClient(address=address, **kwargs)  # type: ignore[no-redef]
                FlowMeter.open_ports[address] = (self.hw, 1)
        else:
            self.hw = TcpClient(address=address, **kwargs)

        self.unit = unit
        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'setpoint', 'gas']
        self.open = True
        self.firmware: str | None = None

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
        if not self.open:
            raise OSError(f"The FlowMeter with unit ID {self.unit} and "
                          f"port {self.hw.address} is not open")

    async def _write_and_read(self, command: str) -> str | None:
        """Wrap the communicator request, to call _test_controller_open() before any request."""
        self._test_controller_open()
        return await self.hw._write_and_read(command)

    async def get(self) -> dict:
        """Get the current state of the flow controller.

        From the Alicat mass flow controller documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Volumetric flow (in units specified at time of order)
         * Mass flow (in units specified at time of order)
         * Total flow (only on models with the optional totalizer function)
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
        if values[-1].upper() == 'LCK':
            self.button_lock = True
            del values[-1]
        else:
            self.button_lock = False
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-2]
        elif len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, 'total flow')
        elif len(values) == 2 and len(self.keys) == 6:
            self.keys.insert(1, 'setpoint')
        elif len(values) == 4 and len(self.keys) == 6:  # LCR (liquid)
            del self.keys[-1]  # gas
            del self.keys[2]  # volumetric flow
        return {k: (float(v) if _is_float(v) else v)
                for k, v in zip(self.keys, values)}
    async def set_gas(self, gas: str | int) -> None:
        """Set the gas type.

        Args:
            gas: The gas type, as a string or integer. Supported strings are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'

                Gas mixes may only be called by their mix number.
        """
        if isinstance(gas, str):
            if gas not in self.gases:
                raise ValueError(f"{gas} not supported!")
            gas_number = self.gases.index(gas)
        else:
            gas_number = gas
        command = f'{self.unit}$$W46={gas_number}'
        await self._write_and_read(command)
        reg46 = await self._write_and_read(f'{self.unit}$$R46')
        if not reg46:
            raise OSError("Cannot set gas.")
        reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if gas_number != reg46_gasbit:
            raise OSError("Cannot set gas.")

    async def create_mix(self, mix_no: int, name: str, gases: dict) -> None:
        """Create a gas mix.

        Gas mixes are made using COMPOSER software.
        COMPOSER mixes are only allowed for firmware 5v or greater.

        Args:
        mix_no: The mix number. Gas mixes are stored in slots 236-255.
        name: A name for the gas that will appear on the front panel.
        Names greater than six letters will be cut off.
        gases: A dictionary of the gas by name along with the associated
        percentage in the mix.
        """
        firmware = await self.get_firmware()
        if any(v in firmware for v in ['2v', '3v', '4v', 'GP']):
            raise OSError("This unit does not support COMPOSER gas mixes.")

        if mix_no < 236 or mix_no > 255:
            raise ValueError("Mix number must be between 236-255!")

        total_percent = sum(gases.values())
        if total_percent != 100:
            raise ValueError("Percentages of gas mix must add to 100%!")

        if any(gas not in self.gases for gas in gases):
            raise ValueError("Gas not supported!")

        gas_list = [f'{percent} {self.gases.index(gas)}' for gas, percent in gases.items()]
        command = ' '.join([
            self.unit,
            'GM',
            name,
            str(mix_no),
            ' '.join(gas_list),
])

        line = await self._write_and_read(command)

        # If a gas mix is not successfully created, ? is returned.
        if line == '?':
            raise OSError("Unable to create mix.")

    async def delete_mix(self, mix_no: int) -> None:
        """Delete a gas mix."""
        command = f'{self.unit}GD{mix_no}'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to delete mix.")

    async def lock(self) -> None:
        """Lock the buttons."""
        command = f'{self.unit}$$L'
        await self._write_and_read(command)

    async def unlock(self) -> None:
        """Unlock the buttons."""
        command = f'{self.unit}$$U'
        await self._write_and_read(command)

    async def is_locked(self) -> bool:
        """Return whether the buttons are locked."""
        await self.get()
        return self.button_lock

    async def tare_pressure(self) -> None:
        """Tare the pressure."""
        command = f'{self.unit}$$PC'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to tare pressure.")

    async def tare_volumetric(self) -> None:
        """Tare volumetric flow."""
        command = f'{self.unit}$$V'
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

        Also close the serial port if no other FlowMeter object has
        a reference to the port.
        """
        if not self.open:
            return
        port = self.hw.address
        if port in FlowMeter.open_ports:
            connection, refcount = FlowMeter.open_ports[port]
            if refcount > 1:
                FlowMeter.open_ports[port] = (connection, refcount - 1)
            else:
                await connection.close()  # Close the port if no other instance uses it
                del FlowMeter.open_ports[port]
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

    registers: ClassVar[dict] = {'mass flow': 0b00100101, 'vol flow': 0b00100100,
                                 'abs pressure': 0b00100010, 'gauge pressure': 0b00100110,
                                 'diff pressure': 0b00100111}

    def __init__(self, address: str='/dev/ttyUSB0', unit: str='A', **kwargs: Any) -> None:
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port or TCP address:port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
        """
        FlowMeter.__init__(self, address, unit, **kwargs)
        self.control_point = None
        async def _init_control_point() -> None:
            self.control_point = await self._get_control_point()
        self._init_task = asyncio.create_task(_init_control_point())

    async def __aenter__(self, *args: Any) -> FlowController:
        """Provide async enter to context manager."""
        return self

    async def _write_and_read(self, command: str) -> str | None:
        """Wrap the communicator request.

        (1) Ensure _init_task is called once before the first request
        (2) Call _test_controller_open() before any request
        """
        if 'R122' not in command:
            await self._init_task
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
        state['control_point'] = self.control_point
        return state

    async def set_flow_rate(self, flowrate: float) -> None:
        """Set the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        if self.control_point in ['abs pressure', 'gauge pressure', 'diff pressure']:
            await self._set_setpoint(0)
            await self._set_control_point('mass flow')
        await self._set_setpoint(flowrate)

    async def set_pressure(self, pressure: float) -> None:
        """Set the target pressure.

        Args:
            pressure: The target pressure, in units specified at time of
                purchase. Likely in psia.
        """
        if self.control_point in ['mass flow', 'vol flow']:
            await self._set_setpoint(0)
            await self._set_control_point('abs pressure')
        await self._set_setpoint(pressure)

    async def get_totalizer_batch(self, batch: int = 1) -> str:
        """Get the totalizer batch volume (firmware 10v00).

        Args:
            batch: Which of the two totalizer batches to query.
                Default is 1; some devices have 2

        Returns:
            line: Current value of totalizer batch
        """
        command = f'{self.unit}$$TB {batch}'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to read totalizer batch volume.")
        values = line.split(" ")  # type: ignore[union-attr]
        return f'{values[2]} {values[4]}' # returns 'batch vol' 'units'

    async def set_totalizer_batch(self, batch_volume: float, batch: int = 1, units: str = 'default') -> None:
        """Set the totalizer batch volume (firmware 10v00).

        Args:
            batch: Which of the two totalizer batches to set.
                Default is 1; some devices have 2
            batch_volume: Target batch volume, in same units as units
                on device
            units: Units of the volume being provided. Default
                is 0, so device returns default engineering units.
        """
        engineering_units_table = {"default":0, "SμL":2, "SmL":3, "SL":4, \
                    "Scm3":6, "Sm3":7, "Sin3":8, "Sft3":9, "kSft3":10, "NμL":32, \
                    "NmL":33, "NL":34, "Ncm3":36, "Nm3":37}

        if units in engineering_units_table:
            units_no = engineering_units_table[units]
        else:
            raise ValueError("Units not in unit list. Please consult Appendix B-3 of the Alicat Serial Primer.")

        command = f'{self.unit}$$TB {batch} {batch_volume} {units_no}'
        line = await self._write_and_read(command)

        if line == '?':
            raise OSError("Unable to set totalizer batch volume. Check if volume is out of range for device.")

    async def hold(self) -> None:
        """Override command to issue a valve hold (firmware 5v07).

        For a single valve controller, hold the valve at the present value.
        For a dual valve flow controller, hold the valve at the present value.
        For a dual valve pressure controller, close both valves.
        """
        command = f'{self.unit}$$H'
        await self._write_and_read(command)

    async def cancel_hold(self) -> None:
        """Cancel valve hold."""
        command = f'{self.unit}$$C'
        await self._write_and_read(command)

    async def get_pid(self) -> dict:
        """Read the current PID values on the controller.

        Values include the loop type, P value, D value, and I value.
        Values returned as a dictionary.
        """
        self.pid_keys = ['loop_type', 'P', 'D', 'I']

        command = f'{self.unit}$$r85'
        read_loop_type = await self._write_and_read(command)
        if not read_loop_type:
            raise OSError("Could not get PID values.")
        spl = read_loop_type.split()

        loopnum = int(spl[3])
        loop_type = ['PD/PDF', 'PD/PDF', 'PD2I'][loopnum]
        pid_values = [loop_type]
        for register in range(21, 24):
            value = await self._write_and_read(f'{self.unit}$$r{register}')
            if not value:
                raise OSError(f"Could not read register {register}")
            value_spl = value.split()
            pid_values.append(value_spl[3])

        return {k: (v if k == self.pid_keys[-1] else str(v))
                for k, v in zip(self.pid_keys, pid_values)}

    async def set_pid(self, p: int | None=None,
                            i: int | None=None,
                            d: int | None=None,
                            loop_type: str | None=None) -> None:
        """Set specified PID parameters.

        Args:
            p: Proportional gain
            i: Integral gain. Only used in PD2I loop type.
            d: Derivative gain
            loop_type: Algorithm option, either 'PD/PDF' or 'PD2I'

        This communication works by writing Alicat registers directly.
        """
        if loop_type is not None:
            options = ['PD/PDF', 'PD2I']
            if loop_type not in options:
                raise ValueError(f'Loop type must be {options[0]} or {options[1]}.')
            loop_num=options.index(loop_type) + 1
            command = f'{self.unit}$$w85={loop_num}'
            await self._write_and_read(command)
        if p is not None:
            command = f'{self.unit}$$w21={p}'
            await self._write_and_read(command)
        if i is not None:
            command = f'{self.unit}$$w23={i}'
            await self._write_and_read(command)
        if d is not None:
            command = f'{self.unit}$$w22={d}'
            await self._write_and_read(command)

    async def _set_setpoint(self, setpoint: float) -> None:
        """Set the target setpoint.

        Called by `set_flow_rate` and `set_pressure`, which both use the same
        command once the appropriate register is set.
        """
        command = f'{self.unit}S{setpoint:.2f}'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not set setpoint.")
        try:
            current = float(line.split()[5])
        except IndexError:
            current = None
        if current is not None and abs(current - setpoint) > 0.01:
            # possibly the setpoint is being ramped
            command = f'{self.unit}LS'
            line = await self._write_and_read(command)
            if not line:
                raise OSError("Could not set setpoint.")
            try:
                commanded = float(line.split()[2])
            except IndexError:
                raise OSError("Could not set setpoint.") from None
            if commanded is not None and abs(commanded - setpoint) > 0.01:
                raise OSError("Could not set setpoint.")

    async def _get_control_point(self) -> str:
        """Get the control point, and save to internal variable."""
        command = f'{self.unit}R122'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not read control point.")
        value = int(line.split('=')[-1])
        try:
            cp = next(p for p, r in self.registers.items() if value == r)
            self.control_point = cp
            return cp
        except StopIteration:
            raise ValueError(f"Unexpected register value: {value:d}") from None

    async def _set_control_point(self, point: str) -> None:
        """Set whether to control on mass flow or pressure.

        Args:
            point: Either "flow" or "pressure".
        """
        if point not in self.registers:
            raise ValueError("Control point must be 'flow' or 'pressure'.")
        reg = self.registers[point]
        command = f'{self.unit}W122={reg:d}'
        line = await self._write_and_read(command)
        if not line:
            raise OSError("Could not set control point.")
        value = int(line.split('=')[-1])
        if value != reg:
            raise OSError("Could not set control point.")
        self.control_point = point

    async def set_ramp_config(self, config: dict[str, bool]) -> None:
        """Configure the setpoint ramp settings (firmware 10v05).

        `up`: whether the controller ramps when increasing the setpoint,
        `down`: whether the controller ramps when decreasing the setpoint,
                (this includes setpoints below 0 on bidirectional devices),
        `zero`: whether the controller ramps when establishing a zero setpoint,
        `power`: whether the controller ramps when using a power-up setpoint
        """
        command = (f"{self.unit}LSRC"
                  f" {1 if config['up'] else 0}"
                  f" {1 if config['down'] else 0}"
                  f" {1 if config['zero'] else 0}"
                  f" {1 if config['power'] else 0}")
        line = await self._write_and_read(command)
        if not line or self.unit not in line:
            raise OSError("Could not set ramp config.")


    async def get_ramp_config(self) -> dict[str, bool]:
        """Get the setpoint ramp settings (firmware 10v05).

        `up`: whether the controller ramps when increasing the setpoint,
        `down`: whether the controller ramps when decreasing the setpoint,
                (this includes setpoints below 0 on bidirectional devices),
        `zero`: whether the controller ramps when establishing a zero setpoint,
        `power`: whether the controller ramps when using a power-up setpoint
        """
        command = f"{self.unit}LSRC"
        line = await self._write_and_read(command)
        if not line or self.unit not in line:
            raise OSError("Could not read ramp config.")
        values = line[2:].split(' ')
        if len(values) != 4:
            raise OSError("Could not read ramp config.")
        return {
            'up': values[0] == '1',
            'down': values[1] == '1',
            'zero': values[2] == '1',
            'power': values[3] == '1',
        }
