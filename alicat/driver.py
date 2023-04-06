"""Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2023 NuMat Technologies
"""
from typing import Dict

from .util import Client, SerialClient, TcpClient


class FlowMeter:
    """Python driver for Alicat Flow Meters.

    [Reference](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).

    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial.
    """

    # A dictionary that maps port names to a tuple of connection
    # objects and the refcounts
    open_ports: Dict[int, tuple] = {}

    def __init__(self, address='/dev/ttyUSB0', unit='A', **kwargs):
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port or TCP address:port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
        """
        if address.startswith('/dev') or address.startswith('COM'):  # serial
            self.hw: Client = SerialClient(address=address, **kwargs)
        else:
            self.hw = TcpClient(address=address, **kwargs)

        self.unit = unit
        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'setpoint', 'gas']
        self.gases = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                      'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                      'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                      'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                      'P-5']

        self.open = True

    async def __aenter__(self, *args):
        """Provide async enter to context manager."""
        return self

    async def __aexit__(self, *args):
        """Provide async exit to context manager."""
        return

    @classmethod
    async def is_connected(cls, port, unit='A') -> bool:
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
                    assert c and 'setpoint' not in device.keys
                elif cls.__name__ == 'FlowController':
                    assert c and 'setpoint' in device.keys
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
            raise IOError("The FlowMeter with unit ID {} and \
                          port {} is not open".format(self.unit,
                                                      self.hw.address))

    async def get(self, retries=2) -> dict:
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
        self._test_controller_open()

        command = f'{self.unit}\r'
        line = await self._write_and_read(command, retries)
        spl = line.split()
        unit, values = spl[0], spl[1:]

        # Mass/volume over range error.
        # Explicitly silenced because I find it redundant.
        while values[-1].upper() in ['MOV', 'VOV', 'POV']:
            del values[-1]
        if unit != self.unit:
            raise ValueError("Flow controller unit ID mismatch.")
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-2]
        elif len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, 'total flow')
        elif len(values) == 2 and len(self.keys) == 6:
            self.keys.insert(1, 'setpoint')
        return {k: (v if k == self.keys[-1] or isinstance(v, str) else float(v))
                for k, v in zip(self.keys, values)}

    async def set_gas(self, gas, retries=2):
        """Set the gas type.

        Args:
            gas: The gas type, as a string or integer. Supported strings are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'

                Gas mixes may only be called by their mix number.
        """
        self._test_controller_open()

        if isinstance(gas, int):
            return await self._set_gas_number(gas, retries)
        else:
            return await self._set_gas_name(gas, retries)

    async def _set_gas_number(self, number, retries):
        """Set flow controller gas type by number.

        See supported gases in 'FlowController.gases'.
        """
        self._test_controller_open()
        command = '{unit}$${index}\r'.format(unit=self.unit,
                                             index=number)
        await self._write_and_read(command, retries)

        reg46 = await self._write_and_read('{unit}$$R46\r'.format(
            unit=self.unit
        ), retries)
        reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if number != reg46_gasbit:
            raise IOError("Cannot set gas.")

    async def _set_gas_name(self, name, retries):
        """Set flow controller gas type by name.

        See the Alicat manual for usage.
        """
        self._test_controller_open()
        if name not in self.gases:
            raise ValueError(f"{name} not supported!")
        command = '{unit}$${gas}\r'.format(
            unit=self.unit,
            gas=self.gases.index(name)
        )
        await self._write_and_read(command, retries)

        reg46 = await self._write_and_read('{unit}$$R46\r'.format(
            unit=self.unit
        ), retries)
        reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if self.gases.index(name) != reg46_gasbit:
            raise IOError("Cannot set gas.")

    async def create_mix(self, mix_no, name, gases, retries=2) -> None:
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
        self._test_controller_open()

        read = f'{self.unit}VE\r'
        firmware = await self._write_and_read(read, retries)
        if any(v in firmware for v in ['2v', '3v', '4v', 'GP']):
            raise IOError("This unit does not support COMPOSER gas mixes.")

        if mix_no < 236 or mix_no > 255:
            raise ValueError("Mix number must be between 236-255!")

        total_percent = sum(gases.values())
        if total_percent != 100:
            raise ValueError("Percentages of gas mix must add to 100%!")

        if any(gas not in self.gases for gas in gases):
            raise ValueError("Gas not supported!")

        gas_list = ' '.join(
            [' '.join([str(percent), str(self.gases.index(gas))])
                for gas, percent in gases.items()])
        command = ' '.join([self.unit,
                            'GM',
                            name,
                            str(mix_no),
                            gas_list]) + '\r'

        line = await self._write_and_read(command, retries)

        # If a gas mix is not successfully created, ? is returned.
        if line == '?':
            raise IOError("Unable to create mix.")

    async def delete_mix(self, mix_no, retries=2) -> None:
        """Delete a gas mix."""
        self._test_controller_open()
        command = "{unit}GD{mixNumber}\r".format(unit=self.unit,
                                                 mixNumber=mix_no)
        line = await self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to delete mix.")

    async def lock(self, retries=2) -> None:
        """Lock the display."""
        self._test_controller_open()
        command = f'{self.unit}$$L\r'
        await self._write_and_read(command, retries)

    async def unlock(self, retries=2) -> None:
        """Unlock the display."""
        self._test_controller_open()
        command = f'{self.unit}$$U\r'
        await self._write_and_read(command, retries)

    async def tare_pressure(self, retries=2) -> None:
        """Tare the pressure."""
        self._test_controller_open()

        command = f'{self.unit}$$PC\r'
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare pressure.")

    async def tare_volumetric(self, retries=2) -> None:
        """Tare volumetric flow."""
        self._test_controller_open()
        command = f'{self.unit}$$V\r'
        line = await self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare flow.")

    async def reset_totalizer(self, retries=2) -> None:
        """Reset the totalizer."""
        self._test_controller_open()
        command = f'{self.unit}T\r'
        await self._write_and_read(command, retries)

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

        await self.flush()

        self.hw.close

        self.open = False

    async def _write_and_read(self, command, retries=2) -> str:
        """Write a command and reads a response from the flow controller."""
        self._test_controller_open()

        for _ in range(retries + 1):
            await self.hw._write(command)
            line = await self._readline()
            if line:
                return line
        else:
            raise IOError("Could not read from flow controller.")

    async def _readline(self) -> str:
        """Read a line using a custom newline character (CR in this case).

        Function from http://stackoverflow.com/questions/16470903/
        pyserial-2-6-specify-end-of-line-in-readline
        """
        self._test_controller_open()
        return await self.hw._readline()


class FlowController(FlowMeter):
    """Python driver for Alicat Flow Controllers.

    [Reference](http://www.alicat.com/products/mass-flow-meters-and-
    controllers/mass-flow-controllers/).

    This communicates with the flow controller over a USB or RS-232/RS-485
    connection using pyserial.

    To set up your Alicat flow controller, power on the device and make sure
    that the "Input" option is set to "Serial".
    """

    registers = {'mass flow': 0b00100101, 'vol flow': 0b00100100,
                 'abs pressure': 0b00100010, 'gauge pressure': 0b00100110,
                 'diff pressure': 0b00100111}

    def __init__(self, address='/dev/ttyUSB0', unit='A'):
        """Connect this driver with the appropriate USB / serial port.

        Args:
            address: The serial port or TCP address:port. Default '/dev/ttyUSB0'.
            unit: The Alicat-specified unit ID, A-Z. Default 'A'.
        """
        FlowMeter.__init__(self, address, unit)
        # try:
        #     self.control_point = self._get_control_point()
        # except Exception:
        #     self.control_point = None
        self.control_point = None

    async def get(self, retries=2) -> dict:
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

        Args:
            retries: Number of times to re-attempt reading. Default 2.
        Returns:
            The state of the flow controller, as a dictionary.

        """
        state = await FlowMeter.get(self, retries)
        if state is None:
            return None
        state['control_point'] = self.control_point
        return state

    async def set_flow_rate(self, flow, retries=2) -> None:
        """Set the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        if self.control_point in ['abs pressure', 'gauge pressure', 'diff pressure']:
            await self._set_setpoint(0, retries)
            await self._set_control_point('mass flow', retries)
        await self._set_setpoint(flow, retries)

    async def set_pressure(self, pressure, retries=2) -> None:
        """Set the target pressure.

        Args:
            pressure: The target pressure, in units specified at time of
                purchase. Likely in psia.
        """
        if self.control_point in ['mass flow', 'vol flow']:
            await self._set_setpoint(0, retries)
            await self._set_control_point('abs pressure', retries)
        await self._set_setpoint(pressure, retries)

    async def hold(self, retries=2) -> None:
        """Override command to issue a valve hold.

        For a single valve controller, hold the valve at the present value.
        For a dual valve flow controller, hold the valve at the present value.
        For a dual valve pressure controller, close both valves.
        """
        self._test_controller_open()
        command = f'{self.unit}$$H\r'
        await self._write_and_read(command, retries)

    async def cancel_hold(self, retries=2) -> None:
        """Cancel valve hold."""
        self._test_controller_open()
        command = f'{self.unit}$$C\r'
        await self._write_and_read(command, retries)

    async def get_pid(self, retries=2) -> dict:
        """Read the current PID values on the controller.

        Values include the loop type, P value, D value, and I value.
        Values returned as a dictionary.
        """
        self._test_controller_open()

        self.pid_keys = ['loop_type', 'P', 'D', 'I']

        command = f'{self.unit}$$r85\r'
        read_loop_type = await self._write_and_read(command, retries)
        spl = read_loop_type.split()

        loopnum = int(spl[3])
        loop_type = ['PD/PDF', 'PD/PDF', 'PD2I'][loopnum]
        pid_values = [loop_type]
        for register in range(21, 24):
            value = await self._write_and_read('{}$$r{}\r'.format(
                self.unit,
                register))
            value_spl = value.split()
            pid_values.append(value_spl[3])

        return {k: (v if k == self.pid_keys[-1] else str(v))
                for k, v in zip(self.pid_keys, pid_values)}

    async def set_pid(self, p=None, i=None, d=None, loop_type=None, retries=2) -> None:
        """Set specified PID parameters.

        Args:
            p: Proportional gain
            i: Integral gain. Only used in PD2I loop type.
            d: Derivative gain
            loop_type: Algorithm option, either 'PD/PDF' or 'PD2I'

        This communication works by writing Alicat registers directly.
        """
        self._test_controller_open()
        if loop_type is not None:
            options = ['PD/PDF', 'PD2I']
            if loop_type not in options:
                raise ValueError(f'Loop type must be {options[0]} or {options[1]}.')
            command = '{unit}$$w85={loop_num}\r'.format(
                unit=self.unit,
                loop_num=options.index(loop_type) + 1
            )
            await self._write_and_read(command, retries)
        if p is not None:
            command = f'{self.unit}$$w21={p}\r'
            await self._write_and_read(command, retries)
        if i is not None:
            command = f'{self.unit}$$w23={i}\r'
            await self._write_and_read(command, retries)
        if d is not None:
            command = f'{self.unit}$$w22={d}\r'
            await self._write_and_read(command, retries)

    async def _set_setpoint(self, setpoint, retries=2) -> None:
        """Set the target setpoint.

        Called by `set_flow_rate` and `set_pressure`, which both use the same
        command once the appropriate register is set.
        """
        self._test_controller_open()

        command = '{unit}S{setpoint:.2f}\r'.format(unit=self.unit,
                                                   setpoint=setpoint)
        line = await self._write_and_read(command, retries)
        try:
            current = float(line.split()[5])
        except IndexError:
            current = None
        if current is not None and abs(current - setpoint) > 0.01:
            raise IOError("Could not set setpoint.")

    async def _get_control_point(self, retries=2):
        """Get the control point, and save to internal variable."""
        command = f'{self.unit}R122\r'
        line = await self._write_and_read(command, retries)
        if not line:
            return None
        value = int(line.split('=')[-1])
        try:
            return next(p for p, r in self.registers.items() if value == r)
        except StopIteration:
            raise ValueError(f"Unexpected register value: {value:d}")

    async def _set_control_point(self, point, retries=2) -> None:
        """Set whether to control on mass flow or pressure.

        Args:
            point: Either "flow" or "pressure".
        """
        if point not in self.registers:
            raise ValueError("Control point must be 'flow' or 'pressure'.")
        reg = self.registers[point]
        command = f'{self.unit}W122={reg:d}\r'
        line = await self._write_and_read(command, retries)

        value = int(line.split('=')[-1])
        if value != reg:
            raise IOError("Could not set control point.")
        self.control_point = point
