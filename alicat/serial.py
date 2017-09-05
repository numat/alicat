#!/usr/bin/python
"""
A Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2015 NuMat Technologies
"""
try:
    import serial
except ImportError:
    pass


class FlowMeter(object):
    """Python driver for [Alicat Flow Meters](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).

    This communicates with the flow meter over a USB or RS-232/RS-485
    connection using pyserial.
    """
    # A dictionary that maps port names to a tuple of connection
    # objects and the refcounts
    open_ports = {}

    def __init__(self, port='/dev/ttyUSB0', address='A'):
        """Connects this driver with the appropriate USB / serial port.

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
                     'flow_setpoint', 'gas']
        self.gases = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                      'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                      'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                      'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                      'P-5']

        self.open = True

    @classmethod
    def is_connected(cls, port, address='A'):
        """Returns True if the specified port is connected to this device.

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
                    assert c and 'flow_setpoint' not in device.keys
                elif cls.__name__ == 'FlowController':
                    assert c and 'flow_setpoint' in device.keys
                else:
                    raise NotImplementedError('Must be meter or controller.')
                is_device = True
            finally:
                device.close()
        except:
            pass
        return is_device

    def _test_controller_open(self):
        """Raises an IOError if the FlowMeter has been closed.

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
         * Flow setpoint (only for flow controllers)
         * Total flow (only on models with the optional totalizer function)
         * Currently selected gas

        Args:
            retries: Number of times to re-attempt reading. Default 2.
        Returns:
            The state of the flow controller, as a dictionary.
        """
        self._test_controller_open()

        command = '*@={addr}\r'.format(addr=self.address)
        line = self._write_and_read(command, retries)
        spl = line.split()
        address, values = spl[0], spl[1:]

        # Mass/volume over range error.
        # Explicitly silenced because I find it redundant.
        while values[-1] in ['MOV', 'VOV']:
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
        """Sets the gas type.

        Args:
            gas: The gas type, as a string. Supported gas types are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'
        """
        self._test_controller_open()

        if gas not in self.gases:
            raise ValueError("{} not supported!".format(gas))
        command = '{addr}$${gas}\r'.format(addr=self.address,
                                           gas=self.gases.index(gas))
        line = self._write_and_read(command, retries)
        if line.split()[-1] != gas:
            raise IOError("Could not set gas type")

    def flush(self):
        """Reads all available information. Use to clear queue."""
        self._test_controller_open()

        self.connection.flush()
        self.connection.flushInput()
        self.connection.flushOutput()

    def close(self):
        """Closes the flow meter. Call this on program termination.

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
        """Writes a command and reads a response from the flow controller."""
        self._test_controller_open()

        for _ in range(retries+1):
            self.connection.write(command.encode('ascii'))
            line = self._readline()
            if line:
                return line
        else:
            raise IOError("Could not read from flow controller.")

    def _readline(self):
        """Reads a line using a custom newline character (CR in this case).

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
    """Python driver for [Alicat Flow Controllers](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-controllers/).

    This communicates with the flow controller over a USB or RS-232/RS-485
    connection using pyserial.

    To set up your Alicat flow controller, power on the device and make sure
    that the "Input" option is set to "Serial".
    """
    def set_flow_rate(self, flow, retries=2):
        """Sets the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        self._test_controller_open()

        command = '{addr}S{flow:.2f}\r'.format(addr=self.address, flow=flow)
        line = self._write_and_read(command, retries)

        # Some Alicat models don't return the setpoint. This accounts for
        # these devices.
        try:
            setpoint = float(line.split()[-2])
        except IndexError:
            setpoint = None

        if setpoint is not None and abs(setpoint - flow) > 0.01:
            raise IOError("Could not set flow.")


def command_line(args):
    import json
    from time import time

    flow_controller = FlowController(port=args.port, address=args.address)

    if args.set_gas:
        flow_controller.set_gas(args.set_gas)
    if args.set_flow_rate is not None:
        flow_controller.set_flow_rate(args.set_flow_rate)
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
