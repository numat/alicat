"""Python driver for Alicat mass flow meters and controllers, using asynchronous TCP.

Distributed under the GNU General Public License v2
Copyright (C) 2019 NuMat Technologies
"""
try:
    import asyncio
except ImportError:
    raise ImportError("TCP connections require python >=3.5.")

import logging


class FlowMeter(object):
    """Python driver for Alicat Flow Meters.

    [Reference](http://www.alicat.com/
    products/mass-flow-meters-and-controllers/mass-flow-meters/).

    This communicates with the flow meter over a TCP bridge such as the
    [StarTech Device Server](https://www.startech.com/Networking-IO/
    Serial-over-IP/4-Port-Serial-Ethernet-Device-Server-with-PoE~NETRS42348PD).

    To set up your Alicat flow meter, power on the device and make sure
    that the "Setpoint Source" option is set to "Serial".
    """

    def __init__(self, ip, port, address='A'):
        """Initialize the device client.

        Args:
            ip: IP address of the device server
            port: Port of the device server
            address: The Alicat-specified address, A-Z. Default 'A'.
        """
        self.ip = ip
        self.port = port
        self.address = address
        self.open = False
        self.lock = asyncio.Lock()
        self.timeouts = 0
        self.max_timeouts = 10
        self.keys = ['pressure', 'temperature', 'volumetric_flow', 'mass_flow',
                     'setpoint', 'gas']
        self.gases = ['Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He',
                      'N2', 'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2',
                      'C2H4', 'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10',
                      'C-8', 'C-2', 'C-75', 'A-75', 'A-25', 'A1025', 'Star29',
                      'P-5']

    async def _connect(self):
        """Asynchronously open a TCP connection with the server."""
        self.open = False
        reader, writer = await asyncio.open_connection(self.ip, self.port)
        self.connection = {'reader': reader, 'writer': writer}
        self.open = True

    async def get(self):
        """Get the current state of the flow meter.

        From the Alicat mass flow meter documentation, this data is:
         * Pressure (normally in psia)
         * Temperature (normally in C)
         * Volumetric flow (in units specified at time of order)
         * Mass flow (in units specified at time of order)
         * Total flow (only on models with the optional totalizer function)
         * Currently selected gas

        Returns:
            The state of the flow controller, as a dictionary.

        """
        command = '{addr}\r'.format(addr=self.address)
        line = await self._write_and_read(command)
        if line:
            spl = line.split()
            address, values = spl[0], spl[1:]

            # Mass/volume over range error.
            # Explicitly silenced because I find it redundant.
            while values[-1].upper() in ['MOV', 'VOV']:
                del values[-1]

            if address != self.address:
                raise ValueError("Flow controller address mismatch.")
            if '=' in values:
                raise ValueError(f'Received reply {values} intended for another request. '
                                 f'Are there simultaneous connections?')
            if len(values) == 5 and len(self.keys) == 6:
                del self.keys[-2]
            elif len(values) == 7 and len(self.keys) == 6:
                self.keys.insert(5, 'total flow')
            return {k: (v if k == self.keys[-1] else float(v))
                    for k, v in zip(self.keys, values)}
        else:
            return {k: None for k in self.keys}

    async def set_gas(self, gas):
        """Set the gas type.

        Args:
            gas: The gas type, as a string or integer. Supported strings are:
                'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', 'He', 'N2',
                'N2O', 'Ne', 'O2', 'C3H8', 'n-C4H10', 'C2H2', 'C2H4',
                'i-C2H10', 'Kr', 'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2',
                'C-75', 'A-75', 'A-25', 'A1025', 'Star29', 'P-5'

                Gas mixes may only be called by their mix number.
        """
        if isinstance(gas, int):
            return self._set_gas_number(gas, retries)
        else:
            return self._set_gas_name(gas, retries)

    def _set_gas_number(self, number, retries):
        command = '{addr}$${index}\r'.format(addr=self.address,
                                             index=number)
        await self._write_and_read(command, retries)

        reg46 = await self._write_and_read('{addr}$$R46\r'.format(
            addr=self.address
        ), retries)

        if reg46:
            reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

            if number != reg46_gasbit:
                raise IOError("Cannot set gas.")
            pass

    async def _set_gas_name(self, name, retries):
        if name not in self.gases:
            raise ValueError("{} not supported!".format(gas))
        command = '{addr}$${gas}\r'.format(
            addr=self.address,
            gas=self.gases.index(name)
        )
        await self._write_and_read(command, retries)

        reg46 = await self._write_and_read('{addr}$$R46\r'.format(
            addr=self.address
        ), retries)

        if reg46:
            reg46_gasbit = int(reg46.split()[-1]) & 0b0000000111111111

        if self.gases.index(name) != reg46_gasbit:
            raise IOError("Cannot set gas.")
        pass

    async def create_mix(self, mix_no, name, gases, retries=2):
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
        read = '{addr}VE\r'.format(addr=self.address)
        firmware = await self._write_and_read(read, retries)
        if "2v" in firmware \
                or "3v" in firmware \
                or "4v" in firmware \
                or "GP" in firmware:
            raise IOError("This unit does not support COMPOSER gas mixes.")

        if mix_no < 236 or mix_no > 255:
            raise ValueError("Mix number must be between 236-255!")

        gas = [str(i) for i in gases.keys()]
        percent = [str(i) for i in gases.values()]

        total_percent = 0
        for i in percent:
            total_percent += int(i)

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

        line = await self._write_and_read(command)

        # If a gas mix is not successfully created, ? is returned.
        if line == '?':
            raise IOError("Unable to create mix.")

    async def delete_mix(self, mix_no):
        """Delete a gas mix."""
        command = "{addr}GD{mixNumber}\r".format(addr=self.address,
                                                 mixNumber=mix_no)
        line = await self._write_and_read(command)

        if line == "?":
            raise IOError("Unable to delete mix.")

    async def lock(self):
        """Lock the display."""
        command = '{addr}$$L\r'.format(addr=self.address)
        await self._write_and_read(command)

    async def unlock(self):
        """Unlock the display."""
        command = '{addr}$$U\r'.format(addr=self.address)
        await self._write_and_read(command)

    async def tare_pressure(self):
        """Tare the pressure."""
        command = '{addr}$$PC\r'.format(addr=self.address)
        line = await self._write_and_read(command)

        if line == '?':
            raise IOError("Unable to tare pressure.")

    async def tare_volumetric(self):
        """Tare volumetric flow."""
        command = '{addr}$$V\r'.format(addr=self.address)
        line = await self._write_and_read(command)

        if line == '?':
            raise IOError("Unable to tare flow.")

    async def reset_tot(self):
        """Reset the totalizer."""
        command = '{addr}T\r'.format(addr=self.address)
        await self._write_and_read(command)

    def close(self):
        """Close the device server connection."""
        if self.open:
            self.connection['writer'].close()
        self.open = False

    async def _write_and_read(self, command):
        """Write a command and reads a response from the flow controller.

        There are two fail points here: 1. communication between this driver
        and the proxy, and 2. communication between the proxy and the Alicat.
        We need to separately check for and manage each.
        A lock is used to queue multiple requests.
        """
        async with self.lock:  # lock releases on CancelledError
            try:
                if not self.open:
                    await asyncio.wait_for(self._connect(), timeout=0.25)
                self.reconnecting = False
            except asyncio.TimeoutError:
                if not self.reconnecting:
                    logging.error('Connecting to {}:{} timed out.'.format(
                                  self.ip, self.port))
                self.reconnecting = True
                return None
            except Exception as e:
                logging.warning('Failed to connect: {}'.format(e))
                self.close()
                return None

            try:
                self.connection['writer'].write(command.encode())
                future = self.connection['reader'].readuntil(b'\r')
                line = await asyncio.wait_for(future, timeout=0.25)
                result = line.decode().strip()
                self.timeouts = 0
            except asyncio.TimeoutError:
                self.timeouts += 1
                if self.timeouts == self.max_timeouts:
                    logging.error('Reading Alicat from {}:{} timed out {} times.'
                                  .format(self.ip, self.port, self.max_timeouts))
                result = None
            except Exception as e:
                logging.warning('Failed to connect: {}'.format(e))
                self.close()
                result = None
        return result


class FlowController(FlowMeter):
    """Python driver for Alicat Flow Controllers.

    [Reference](http://www.alicat.com/products/mass-flow-meters-and-
    controllers/mass-flow-controllers/).

    This communicates with the flow controller over a TCP bridge such as the
    [StarTech Device Server](https://www.startech.com/Networking-IO/
    Serial-over-IP/4-Port-Serial-Ethernet-Device-Server-with-PoE~NETRS42348PD).

    To set up your Alicat flow controller, power on the device and make sure
    that the "Setpoint Source" option is set to "Serial".
    """
    registers = {'mass flow': 0b00100101, 'vol flow': 0b00100100,
                 'abs pressure': 0b00100010, 'gauge pressure': 0b00100110,
                 'diff pressure': 0b00100111}

    def __init__(self, ip, port, address='A'):
        """Initialize the device client.

        Args:
            ip: IP address of the device server
            port: Port of the device server
            address: The Alicat-specified address, A-Z. Default 'A'.
        """
        FlowMeter.__init__(self, ip, port, address)

        self.control_point = 'unknown'
        self.init_lock = asyncio.Lock()
        asyncio.ensure_future(self._get_control_point())

    async def get(self):
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
        state = await FlowMeter.get(self)
        if state is None:
            return None
        state['control_point'] = self.control_point
        return state

    async def set_flow_rate(self, flow):
        """Set the target flow rate.

        Args:
            flow: The target flow rate, in units specified at time of purchase
        """
        if any([self.control_point is not None and
                self.control_point == 'abs pressure',
                self.control_point is not None and
                self.control_point == 'gauge pressure',
                self.control_point is not None and
                self.control_point == 'diff pressure']):
            await self._set_setpoint(0)
            await self._set_control_point('mass flow')
        await self._set_setpoint(flow)

    async def set_pressure(self, pressure):
        """Set the target pressure.

        Args:
            pressure: The target pressure, in units specified at time of
                purchase. Likely in psia.
        """
        if any([self.control_point is not None and
                self.control_point == 'mass flow',
                self.control_point is not None and
                self.control_point == 'vol flow']):
            await self._set_setpoint(0)
            await self._set_control_point('abs pressure')
        await self._set_setpoint(pressure)

    async def hold(self):
        """Override command to issue a valve hold.

        For a single valve controller, hold the valve at the present value.
        For a dual valve flow controller, hold the valve at the present value.
        For a dual valve pressure controller, close both valves.
        """
        command = "{addr}$$H\r".format(addr=self.address)
        await self._write_and_read(command)

    async def cancel_hold(self):
        """Cancel valve hold."""
        command = "{addr}$$C\r".format(addr=self.address)
        await self._write_and_read(command)

    async def read_PID(self):
        """Read the current PID values on the controller.

        Values include the loop type, P value, D value, and I value.

        Values returned as a dictionary.
        """
        self.pid_keys = ['loop_type', 'P', 'D', 'I']

        command = '{addr}$$r85\r'.format(addr=self.address)
        read_loop_type = await self._write_and_read(command)
        spl = read_loop_type.split()

        loopnum = int(spl[3])
        loop_type = ['PD/PDF', 'PD/PDF', 'PD2I'][loopnum]
        pid_values = [loop_type]
        for register in range(21, 24):
            value = await self._write_and_read('{}$$r{}\r'.format(
                self.address,
                register))
            value_spl = value.split()
            pid_values.append(value_spl[3])

        result = {k: (v if k == self.pid_keys[-1] else str(v))
                  for k, v in zip(self.pid_keys, pid_values)}

        return result

    async def write_PID_looptype(self, looptype):
        """Change the PID loop from PD/PDF to PD2I and vice versa.

        Done by changing the appropriate bits in register 85.
        """
        command = '{addr}$$w85={loop_num}\r'.format(
            addr=self.address,
            loop_num=['', 'PD/PDF', 'PD2I'].index(looptype)
        )

        await self._write_and_read(command)

    async def write_PID_P(self, p_value):
        """Change P value for PID tuning."""
        value = p_value
        command = '{addr}$$w21={v}\r'.format(addr=self.address, v=value)
        await self._write_and_read(command)

    async def write_PID_D(self, d_value):
        """Change D value for PID tuning."""
        value = d_value
        command = '{addr}$$w22={v}\r'.format(addr=self.address, v=value)
        await self._write_and_read(command)

    async def write_PID_I(self, i_value):
        """Change I value for PID tuning.

        Only used in PD2I loop type.
        """
        value = i_value
        command = '{addr}$$w23={v}\r'.format(addr=self.address, v=value)
        await self._write_and_read(command)

    async def _set_setpoint(self, setpoint):
        """Set the target setpoint.

        Called by `set_flow_rate` and `set_pressure`, which both use the same
        command once the appropriate register is set.
        """
        command = '{addr}S{setpoint:.2f}\r'.format(addr=self.address,
                                                   setpoint=setpoint)
        line = await self._write_and_read(command)

        # Some Alicat models don't return the setpoint. This accounts for
        # these devices.
        try:
            current = float(line.split()[5])
        except (IndexError, AttributeError):
            current = None
        if current is not None and abs(current - setpoint) > 0.01:
            raise IOError("Could not set setpoint.")

    async def _get_control_point(self):
        """Get the control point, and save to internal variable."""
        async with self.init_lock:
            command = '{addr}R122\r'.format(addr=self.address)
            line = await self._write_and_read(command)
            if not line:
                return None
            if '=' not in line:
                raise ValueError(f'Received reply {line} intended for another request. '
                                 f'Are there simultaneous connections?')
            value = int(line.split('=')[-1])
            try:
                self.control_point = next(p for p, r in self.registers.items() if value == r)
                return self.control_point
            except StopIteration:
                raise ValueError("Unexpected register value: {:d}".format(value))

    async def _set_control_point(self, point):
        """Set whether to control on mass flow or pressure.

        Args:
            point: Either "flow" or "pressure".
        """
        if point not in self.registers:
            raise ValueError("Control point must be 'flow' or 'pressure'.")
        reg = self.registers[point]
        command = '{addr}W122={reg:d}\r'.format(addr=self.address, reg=reg)
        line = await self._write_and_read(command)
        if '=' not in line:
            raise ValueError(f'Received reply {line} intended for another request. '
                             f'Are there simultaneous connections?')
        value = int(line.split('=')[-1])
        if value != reg:
            raise IOError("Could not set control point.")
        self.control_point = point


def command_line(args):
    """CLI interface, accessible when installed through pip."""
    import json
    from time import time

    ip, port = args.port[6:].split(':')  # strip 'tcp://'
    port = int(port)
    flow_controller = FlowController(ip, port, args.address)

    async def print_state():
        if args.set_gas:
            await flow_controller.set_gas(args.set_gas)
        if args.set_flow_rate is not None and args.set_pressure is not None:
            raise ValueError("Cannot set both flow rate and pressure.")
        if args.set_flow_rate is not None:
            async with flow_controller.init_lock:  # make sure _get_control_point has finished
                await flow_controller.set_flow_rate(args.set_flow_rate)
        if args.set_pressure is not None:
            async with flow_controller.init_lock:
                await flow_controller.set_pressure(args.set_pressure)
        if args.lock:
            await flow_controller.lock()
        if args.unlock:
            await flow_controller.unlock()
        if args.hold:
            await flow_controller.hold()
        if args.cancel_hold:
            await flow_controller.cancel_hold()
        if args.reset_totalizer:
            await flow_controller.reset_tot()
        state = await flow_controller.get()
        if args.stream:
            print('time\t' + '\t'.join(flow_controller.keys))
            t0 = time()
            while True:
                state = await flow_controller.get()
                print('{:.2f}\t'.format(time() - t0) +
                      '\t\t'.join('{:.2f}'.format(state[key])
                                  for key in flow_controller.keys[:-1]) +
                      '\t\t' + state['gas'])
        else:
            print(json.dumps(state, indent=2, sort_keys=True))

    ioloop = asyncio.get_event_loop()
    try:
        ioloop.run_until_complete(print_state())
    except KeyboardInterrupt:
        pass
    flow_controller.close()
    ioloop.close()
