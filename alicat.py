#!/usr/bin/python
"""
A Python driver for Alicat mass flow controllers, using serial communication.

Distributed under the GNU General Public License v2
Copyright (C) 2015 NuMat Technologies
"""
import serial


class FlowController(object):
    """Python driver for [Alicat Flow Controllers](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-controllers/).

    This communicates with the flow controller over a USB or RS-232/RS-485
    connection using pyserial.

    To set up your Alicat flow controller, power on the device and make sure
    that the "Input" option is set to "Serial".
    """
    def __init__(self, port="/dev/ttyUSB0", address="A"):
        """Connects this driver with the appropriate USB / serial port.

        Args:
            port: The serial port. Default "/dev/ttyUSB0".
            address: The Alicat-specified address, A-Z. Default "A".
        """
        self.address = address
        self.connection = serial.Serial(port, 19200, timeout=1)
        self.keys = ["pressure", "temperature", "volumetric_flow", "mass_flow",
                     "flow_setpoint", "gas"]
        self.gases = ["Air", "Ar", "CH4", "CO", "CO2", "C2H6", "H2", "He",
                      "N2", "N2O", "Ne", "O2", "C3H8", "n-C4H10", "C2H2",
                      "C2H4", "i-C2H10", "Kr", "Xe", "SF6", "C-25", "C-10",
                      "C-8", "C-2", "C-75", "A-75", "A-25", "A1025", "Star29",
                      "P-5"]

    def get(self):
        """Get the current state of the flow controller.

        From the Alicat mass flow controller documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Volumetric flow (in units specified at time of order)
         * Mass flow (in units specified at time of order)
         * Flow setpoint
         * Total flow (only on models with the optional totalizer function)
         * Currently selected gas

        Returns:
            The state of the flow controller, as a dictionary.
        """
        command = "*@={addr}\r\n".format(addr=self.address)
        self.connection.write(command)
        line = self.connection.readline().split()
        address, values = line[0], line[1:]
        if address != self.address:
            raise Exception("Flow controller address mismatch.")
        if len(values) == 7 and len(self.keys) == 6:
            self.keys.insert(5, "total flow")
        return {k: (v if k == self.keys[-1] else float(v))
                for k, v in zip(self.keys, values)}

    def set_flow_rate(self, flow):
        """Sets the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        command = "{addr}S{flow:.2f}\r\n".format(addr=self.address, flow=flow)
        self.connection.write(command)

    def set_gas(self, gas):
        """Sets the gas type.

        Args:
            gas: The gas type, as a string. Supported gas types are:
                "Air", "Ar", "CH4", "CO", "CO2", "C2H6", "H2", "He", "N2",
                "N2O", "Ne", "O2", "C3H8", "n-C4H10", "C2H2", "C2H4",
                "i-C2H10", "Kr", "Xe", "SF6", "C-25", "C-10", "C-8", "C-2",
                "C-75", "A-75", "A-25", "A1025", "Star29", "P-5"
        """
        command = "{addr}$${gas}\r\n".format(addr=self.address, gas=gas)
        self.connection.write(command)

    def close(self):
        """Closes the serial port. Call this on program termination."""
        self.connection.close()
