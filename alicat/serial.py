"""Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2019 NuMat Technologies
"""
try:
    import serial
except ImportError:
    pass


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
        while values[-1].upper() in ['MOV', 'VOV']:
            del values[-1]

        if address != self.address:
            raise ValueError("Flow controller address mismatch.")
        if len(values) == 5 and len(self.keys) == 6:
            del self.keys[-2]
        elif len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, 'total flow')
        return {k: (v if k == self.keys[-1] else float(v))
                for k, v in zip(self.keys, values)}

    def set_gas(self, gas, retries=2):
        """Set the gas type.

        Args:
            gas: The gas type, as a string or integer. Supported gas types by string are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'

                For the full gas table, please see page 52 of the controller manual here:
                https://documents.alicat.com/manuals/DOC-MANUAL-MC.pdf

                Gas mixes may only be called by their mix number.
        """
        self._test_controller_open()

        if isinstance(gas, str) is False:
            num = True
        else:
            num = False

        if num is True:
            command = '{addr}$${index}\r'.format(addr=self.address,
                                               index=gas)
            self._write_and_read(command, retries)
        else:
            if gas not in self.gases:
                raise ValueError("{} not supported!".format(gas))
            command = '{addr}$${gas}\r'.format(addr=self.address, gas=self.gases.index(gas))
            self._write_and_read(command, retries)

        read_reg46 = self._write_and_read('{addr}$$R46\r'.format(addr=self.address), retries)
        reg46 = int(read_reg46.split()[-1])
        bits = [32768, 16384, 8192, 4096, 2048, 1024, 512]

        for u in range(0, 7):
            reg46 = reg46 - bits[u]
            if reg46 < 0:
                reg46 = reg46 + bits[u]
            elif reg46 < 255:
                break

        if num is True:
            if gas != reg46:
                raise IOError("Cannot set gas, gas set to Air.")
        else:
            if self.gases.index(gas) != reg46:
                raise IOError("Cannot set gas.")

    def create_mix(self, mix_no, name, gas1, percent1, gas2, percent2, gas3=None, percent3=None, gas4=None, percent4=None, gas5=None, percent5=None, retries=2):
        """Create a gas mix.

        Gas mixes are made using COMPOSER software located from the front panel and over serial.
        COMPOSER mixes can only be made on Alicat devices with firmware 5v or greater.

        Args:
            mix_no: The mix number. Gas mixes are stored in slots 236-255. A mix number of 0 will create a mix in
            the earliest available spot.
            name: A name for the gas that will appear on the front panel. Names greater than six letters will be
            cut off.
            gas#: Name of the gas, as a string. Supported gas types are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'
            percent#: The percentage of the mix for the corresponding gas."""

        self._test_controller_open()

        read = '{addr}VE\r'.format(addr=self.address)
        firmware = self._write_and_read(read, retries)
        if "2v" in firmware or "3v" in firmware or "4v" in firmware or "GP" in firmware:
            raise IOError("This unit does not support COMPOSER gas mixes.")

        if mix_no < 236 or mix_no > 255:
            raise ValueError("Mix number must be between 236-225!")

        total_percent = 0
        mix_list = [percent1, percent2, percent3, percent4, percent5]
        for i in range(0, 5):
            if mix_list[i] is not None:
                total_percent += mix_list[i]
        if total_percent != 100:
            raise ValueError("Percentages of gas mix must add to 100%!")

        if gas1 is not None and gas1 not in self.gases:
            raise ValueError("{} not supported!".format(gas1))
        if gas2 is not None and gas2 not in self.gases:
            raise ValueError("{} not supported!".format(gas2))
        if gas3 is not None and gas3 not in self.gases:
            raise ValueError("{} not supported!".format(gas3))
        if gas4 is not None and gas4 not in self.gases:
            raise ValueError("{} not supported!".format(gas4))
        if gas5 is not None and gas5 not in self.gases:
            raise ValueError("{} not supported!".format(gas5))

        command = '{addr} GM {shortName} {mixNumber} {p1} {g1}' \
                  ' {p2} {g2} {p3} {g3} {p4} {g4} {p5}' \
                  ' {g5}\r'.format(addr=self.address, shortName=name, mixNumber=mix_no, p1=percent1,
                                     g1=self.gases.index(gas1), p2=percent2, g2=self.gases.index(gas2),
                                     p3=percent3 if gas3 is not None else "", g3=self.gases.index(gas3) if gas3 is not None else "", p4=percent4 if gas4 is not None else "",
                                     g4=gas4 if gas4 is not None else "", p5=percent5 if gas5 is not None else "", g5=self.gases.index(gas5) if gas5 is not None else "")

        line = self._write_and_read(command, retries)

        # If a gas mix is not successfully created, a ? is returned.
        if line == '?':
            raise IOError("Unable to create mix.")

    def delete_mix(self, mix_no, retries=2):
        """Delete a gas mix."""

        self._test_controller_open()
        command = "{addr}GD{mixNumber}\r".format(addr=self.address, mixNumber=mix_no)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to delete mix.")

    def lock(self, retries=2):
        """Lock the display.

        Only supported on devices with a display."""

        self._test_controller_open()
        command = '{addr}$$L\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def unlock(self, retries=2):
        """Unlock the display.

        Only supported on devices with a display."""

        self._test_controller_open()
        command = '{addr}$$U\r'.format(addr=self.address)
        self._write_and_read(command, retries)

    def tare_pressure(self, retries=2):
        """Tare the pressure.

        Should only be performed if device at ambient conditions
        e.g. no flow, device open to atmosphere"""

        self._test_controller_open()

        command = '{addr}$$PC\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare pressure.")

    def tare_volumetric(self, retries=2):
        """Tare volumetric flow.

        Should only be performed if device at ambient conditions
        e.g. no flow, device open to atmosphere"""

        self._test_controller_open()
        command = '{addr}$$V\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)

        if line == '?':
            raise IOError("Unable to tare flow.")


    def reset_tot(self, retries=2):
        """Reset the totalizer, only valid for mass flow or liquid Alicats with a totalizer"""

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

    registers = {'flow': 0b00100101, 'pressure': 0b00100010}

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
        if self.control_point is not None and self.control_point != 'flow':
            self._set_setpoint(0, retries)
            self._set_control_point('flow', retries)
        self._set_setpoint(flow, retries)

    def set_pressure(self, pressure, retries=2):
        """Set the target pressure.

        Args:
            pressure: The target pressure, in units specified at time of
                purchase. Likely in psia.
        """
        if self.control_point is not None and self.control_point != 'pressure':
            self._set_setpoint(0, retries)
            self._set_control_point('pressure', retries)
        self._set_setpoint(pressure, retries)

    def hold(self, retries=2):
        """Override command to issue a valve hold.

        For a single valve mass flow/pressure controller, hold the valve at the present value.
        For a dual valve mass flow controller, hold the valve at the present value.
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

    def read_PID(self, retries=2):
        """Read the current PID values on the controller.

        Values include the loop type, P value, D value, and I value.

        Values returned as a dictionary"""

        self._test_controller_open()

        self.pid_keys = ['loop_type', 'P', 'D', 'I']

        command = '{addr}$$r85\r'.format(addr=self.address)
        read_loop_type = self._write_and_read(command, retries)
        spl = read_loop_type.split()

        if spl[3] == '2':
            loop_type = 'PD2I'
        elif spl[3] == '1' or spl[3] == '0':
            loop_type = 'PD/PDF'

        pid_values = [loop_type]
        for register in range(21, 24):
            value = self._write_and_read('{}$$r{}\r'.format(self.address, register))
            value_spl = value.split()
            pid_values.append(value_spl[3])

        result = {k: (v if k == self.pid_keys[-1] else str(v))
                  for k, v in zip(self.pid_keys, pid_values)}

        return result

    def write_PID_looptype(self, looptype, retries=2):
        """Change the PID loop from PD/PDF to PD2I and vice versa

        Done by changing the appropriate bits in register 85."""

        self._test_controller_open()

        if looptype == 'PD/PDF':
            command = '{addr}$$w85=1\r'.format(addr=self.address)
        elif looptype == 'PD2I':
            command = '{addr}$$w85=2\r'.format(addr=self.address)
        else:
            raise ValueError('Not a valid loop type.')

        self._write_and_read(command, retries)

    def write_PID_P(self, p_value, retries=2):
        """Changing P value for PID tuning.

        P is the proportional control variable and controlls how fast setpoint can be achieved."""
        self._test_controller_open()
        value = p_value
        command = '{addr}$$w21={v}\r'.format(addr=self.address, v=value)
        self._write_and_read(command, retries)

    def write_PID_D(self, d_value, retries=2):
        """Changing D value for PID tuning.

        D is the derivative term and primarily operates to dampen overshoots and reduce oscillations."""
        self._test_controller_open()
        value = d_value
        command = '{addr}$$w22={v}\r'.format(addr=self.address, v=value)
        self._write_and_read(command, retries)

    def write_PID_I(self, i_value, retries=2):
        """Changing I value for PID tuning.

        I is the integral term and accounts for past behaviour to provide a control response.
        Only used in PD2I tuning. It can be changed if loop type is PD/PDF but it will have no effect no control."""
        self._test_controller_open()
        value = i_value
        command = '{addr}$$w23={v}\r'.format(addr=self.address, v=value)
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
        flow_controller.reset_tot()
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
