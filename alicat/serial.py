"""Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2019 NuMat Technologies
"""
import serial


class FlowMeter(object):
    """Python driver for Alicat Flow Meters.

    [Reference](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).

    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial.
    """

    # A dictionary that maps port names to a tuple of connection
    # objects and the refcounts
    open_ports = {}

    def __init__(self, port='/dev/ttyUSB0', address='A'):
        """Connect this driver with the appropriate USB / serial port.

        Args:
            port: The serial port. Default '/dev/ttyUSB0'.
            address: The Alicat-specified address, A-Z. Default 'A'.
        """
        self.address = address
        self.port = port

        if port in FlowMeter.open_ports:
            self.connection, refcount = FlowMeter.open_ports[port]
            FlowMeter.open_ports[port] = (self.connection, refcount + 1)
        else:
            self.connection = serial.Serial(port, 19200, timeout=1.0)
            FlowMeter.open_ports[port] = (self.connection, 1)

        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'setpoint', 'gas']
        self.gases = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                      'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                      'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                      'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                      'P-5']

        self.open = True

    @classmethod
    def is_connected(cls, port, address='A'):
        """Return True if the specified port is connected to this device.

        This class can be used to automatically identify ports with connected
        Alicats. Iterate through all connected interfaces, and use this to
        test. Ports that come back True should be valid addresses.

        Note that this distinguishes between `FlowController` and `FlowMeter`.
        """
        is_device = False
        try:
            device = cls(port, address)
            try:
                c = device.get()
                if cls.__name__ == 'FlowMeter':
                    assert c and 'setpoint' not in device.keys
                elif cls.__name__ == 'FlowController':
                    assert c and 'setpoint' in device.keys
                else:
                    raise NotImplementedError('Must be meter or controller.')
                is_device = True
            finally:
                device.close()
        except Exception:
            pass
        return is_device

    def _test_controller_open(self):
        """Raise an IOError if the FlowMeter has been closed.

        Does nothing if the meter is open and good for read/write
        otherwise raises an IOError. This only checks if the meter
        has been closed by the FlowMeter.close method.
        """
        if not self.open:
            raise IOError("The FlowController with address {} and \
                          port {} is not open".format(self.address,
                                                      self.port))

    def get(self, retries=2):
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

        command = '{addr}\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)
        spl = line.split()
        address, values = spl[0], spl[1:]

        # Mass/volume over range error.
        # Explicitly silenced because I find it redundant.
        while values[-1].upper() in ['MOV', 'VOV', 'POV']:
            del values[-1]
        if address != self.address:
            raise ValueError("Flow controller address mismatch.")
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-2]
        elif len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, 'total flow')
        elif len(values) == 2 and len(self.keys) == 6:
            self.keys.insert(1, 'setpoint')
        return {k: (v if k == self.keys[-1] else float(v))
                for k, v in zip(self.keys, values)}

    def set_gas(self, gas, retries=2):
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
            return self._set_gas_number(gas, retries)
        else:
            return self._set_gas_name(gas, retries)

    def _set_gas_number(self, number, retries):
        """Set flow controller gas type by number.

        See supported gases in 'FlowController.gases'.
        """
        self._test_controller_open()
        command = '{addr}$${index}\r'.format(addr=self.address,
                                             index=number)
        self._write_and_read(command, retries)

        reg46 = self._write_and_read('{addr}$$R46\r'.format(
            addr=self.address
        ), retries)
        reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if number != reg46_gasbit:
            raise IOError("Cannot set gas.")

    def _set_gas_name(self, name, retries):
        """Set flow controller gas type by name.

        See the Alicat manual for usage.
        """
        self._test_controller_open()
        if name not in self.gases:
            raise ValueError(f"{name} not supported!")
        command = '{addr}$${gas}\r'.format(
            addr=self.address,
            gas=self.gases.index(name)
        )
        self._write_and_read(command, retries)

        reg46 = self._write_and_read('{addr}$$R46\r'.format(
            addr=self.address
        ), retries)
        reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if self.gases.index(name) != reg46_gasbit:
            raise IOError("Cannot set gas.")

    def create_mix(self, mix_no, name, gases, retries=2):
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

        read = '{addr}VE\r'.format(addr=self.address)
        firmware = self._write_and_read(read, retries)
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
        command = ' '.join([self.address,
                            'GM',
                            name,
                            str(mix_no),
                            gas_list]) + '\r'

        line = self._write_and_read(command, retries)

        # If a gas mix is not successfully created, ? is returned.
        if line == '?':
            raise IOError("Unable to create mix.")

    def delete_mix(self, mix_no, retries=2):
        """Delete a gas mix."""
        self._test_controller_open()
        command = "{addr}GD{mixNumber}\r".format(addr=self.address,
                                                 mixNumber=mix_no)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to delete mix.")

    def lock(self, retries=2):
        """Lock the display."""
        self._test_controller_open()
        command = '{addr}$$L\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def unlock(self, retries=2):
        """Unlock the display."""
        self._test_controller_open()
        command = '{addr}$$U\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def tare_pressure(self, retries=2):
        """Tare the pressure."""
        self._test_controller_open()

        command = '{addr}$$PC\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare pressure.")

    def tare_volumetric(self, retries=2):
        """Tare volumetric flow."""
        self._test_controller_open()
        command = '{addr}$$V\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare flow.")

    def reset_totalizer(self, retries=2):
        """Reset the totalizer."""
        self._test_controller_open()
        command = '{addr}T\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def flush(self):
        """Read all available information. Use to clear queue."""
        self._test_controller_open()

        self.connection.flush()
        self.connection.flushInput()
        self.connection.flushOutput()

    def close(self):
        """Close the flow meter. Call this on program termination.

        Also closes the serial port if no other FlowMeter object has
        a reference to the port.
        """
        if not self.open:
            return

        self.flush()

        if FlowMeter.open_ports[self.port][1] <= 1:
            self.connection.close()
            del FlowMeter.open_ports[self.port]
        else:
            connection, refcount = FlowMeter.open_ports[self.port]
            FlowMeter.open_ports[self.port] = (connection, refcount - 1)

        self.open = False

    def _write_and_read(self, command, retries=2):
        """Write a command and reads a response from the flow controller."""
        self._test_controller_open()

        for _ in range(retries + 1):
            self.connection.write(command.encode('ascii'))
            line = self._readline()
            if line:
                return line
        else:
            raise IOError("Could not read from flow controller.")

    def _readline(self):
        """Read a line using a custom newline character (CR in this case).

        Function from http://stackoverflow.com/questions/16470903/
        pyserial-2-6-specify-end-of-line-in-readline
        """
        self._test_controller_open()

        line = bytearray()
        while True:
            c = self.connection.read(1)
            if c:
                line += c
                if line[-1] == ord('\r'):
                    break
            else:
                break
        return line.decode('ascii').strip()


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

    def __init__(self, port='/dev/ttyUSB0', address='A'):
        """Connect this driver with the appropriate USB / serial port.

        Args:
            port: The serial port. Default '/dev/ttyUSB0'.
            address: The Alicat-specified address, A-Z. Default 'A'.
        """
        FlowMeter.__init__(self, port, address)
        try:
            self.control_point = self._get_control_point()
        except Exception:
            self.control_point = None

    def get(self, retries=2):
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
        state = FlowMeter.get(self, retries)
        if state is None:
            return None
        state['control_point'] = self.control_point
        return state

    def set_flow_rate(self, flow, retries=2):
        """Set the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        if self.control_point in ['abs pressure', 'gauge pressure', 'diff pressure']:
            self._set_setpoint(0, retries)
            self._set_control_point('mass flow', retries)
        self._set_setpoint(flow, retries)

    def set_pressure(self, pressure, retries=2):
        """Set the target pressure.

        Args:
            pressure: The target pressure, in units specified at time of
                purchase. Likely in psia.
        """
        if self.control_point in ['mass flow', 'vol flow']:
            self._set_setpoint(0, retries)
            self._set_control_point('abs pressure', retries)
        self._set_setpoint(pressure, retries)

    def hold(self, retries=2):
        """Override command to issue a valve hold.

        For a single valve controller, hold the valve at the present value.
        For a dual valve flow controller, hold the valve at the present value.
        For a dual valve pressure controller, close both valves.
        """
        self._test_controller_open()
        command = '{addr}$$H\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def cancel_hold(self, retries=2):
        """Cancel valve hold."""
        self._test_controller_open()
        command = '{addr}$$C\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def get_pid(self, retries=2):
        """Read the current PID values on the controller.

        Values include the loop type, P value, D value, and I value.
        Values returned as a dictionary.
        """
        self._test_controller_open()

        self.pid_keys = ['loop_type', 'P', 'D', 'I']

        command = '{addr}$$r85\r'.format(addr=self.address)
        read_loop_type = self._write_and_read(command, retries)
        spl = read_loop_type.split()

        loopnum = int(spl[3])
        loop_type = ['PD/PDF', 'PD/PDF', 'PD2I'][loopnum]
        pid_values = [loop_type]
        for register in range(21, 24):
            value = self._write_and_read('{}$$r{}\r'.format(
                self.address,
                register))
            value_spl = value.split()
            pid_values.append(value_spl[3])

        return {k: (v if k == self.pid_keys[-1] else str(v))
                for k, v in zip(self.pid_keys, pid_values)}

    def set_pid(self, p=None, i=None, d=None, loop_type=None, retries=2):
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
            command = '{addr}$$w85={loop_num}\r'.format(
                addr=self.address,
                loop_num=options.index(loop_type) + 1
            )
            self._write_and_read(command, retries)
        if p is not None:
            command = '{addr}$$w21={v}\r'.format(addr=self.address, v=p)
            self._write_and_read(command, retries)
        if i is not None:
            command = '{addr}$$w23={v}\r'.format(addr=self.address, v=i)
            self._write_and_read(command, retries)
        if d is not None:
            command = '{addr}$$w22={v}\r'.format(addr=self.address, v=d)
            self._write_and_read(command, retries)

    def _set_setpoint(self, setpoint, retries=2):
        """Set the target setpoint.

        Called by `set_flow_rate` and `set_pressure`, which both use the same
        command once the appropriate register is set.
        """
        self._test_controller_open()

        command = '{addr}S{setpoint:.2f}\r'.format(addr=self.address,
                                                   setpoint=setpoint)
        line = self._write_and_read(command, retries)
        try:
            current = float(line.split()[5])
        except IndexError:
            current = None
        if current is not None and abs(current - setpoint) > 0.01:
            raise IOError("Could not set setpoint.")

    def _get_control_point(self, retries=2):
        """Get the control point, and save to internal variable."""
        command = '{addr}R122\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)
        if not line:
            return None
        value = int(line.split('=')[-1])
        try:
            return next(p for p, r in self.registers.items() if value == r)
        except StopIteration:
            raise ValueError("Unexpected register value: {:d}".format(value))

    def _set_control_point(self, point, retries=2):
        """Set whether to control on mass flow or pressure.

        Args:
            point: Either "flow" or "pressure".
        """
        if point not in self.registers:
            raise ValueError("Control point must be 'flow' or 'pressure'.")
        reg = self.registers[point]
        command = '{addr}W122={reg:d}\r'.format(addr=self.address, reg=reg)
        line = self._write_and_read(command, retries)

        value = int(line.split('=')[-1])
        if value != reg:
            raise IOError("Could not set control point.")
        self.control_point = point


def command_line(args):
    """CLI interface, accessible when installed through pip."""
    import json
    from time import time

    flow_controller = FlowController(port=args.port, address=args.address)

    if args.set_gas:
        flow_controller.set_gas(args.set_gas)
    if args.set_flow_rate is not None and args.set_pressure is not None:
        raise ValueError("Cannot set both flow rate and pressure.")
    if args.set_flow_rate is not None:
        flow_controller.set_flow_rate(args.set_flow_rate)
    if args.set_pressure is not None:
        flow_controller.set_pressure(args.set_pressure)
    if args.lock:
        flow_controller.lock()
    if args.unlock:
        flow_controller.unlock()
    if args.hold:
        flow_controller.hold()
    if args.cancel_hold:
        flow_controller.cancel_hold()
    if args.reset_totalizer:
        flow_controller.reset_totalizer()
    state = flow_controller.get()
    if args.stream:
        try:
            print('time\t' + '\t'.join(flow_controller.keys))
            t0 = time()
            while True:
                state = flow_controller.get()
                print('{:.2f}\t'.format(time() - t0) +
                      '\t\t'.join('{:.2f}'.format(state[key])
                                  for key in flow_controller.keys[:-1]) +
                      '\t\t' + state['gas'])
        except KeyboardInterrupt:
            pass
    else:
        print(json.dumps(state, indent=2, sort_keys=True))
    flow_controller.close()
