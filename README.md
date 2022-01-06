alicat
======

Serial driver and command line tool for
[Alicat mass flow controllers](http://www.alicat.com/products/mass-flow-meters-and-controllers/mass-flow-controllers/).

<p align="center">
  <img src="http://www.alicat.com/wpinstall/wp-content/uploads/2012/01/gas-mass-flow-controller1.jpg" height="400" />
</p>

*If you are using Analyt-MTC flow controllers, go to [this repository](https://github.com/schlenzmeister/AnalytMTC/wiki) for more info.*

Example Connections
===================

| Type | Usage |
| --- | --- |
| The standard [DB9 cable](http://www.alicat.com/wpinstall/wp-content/uploads/2013/07/MD8DB9.jpg) connected directly to a computer (unix: `/dev/ttyS0`, windows: `COM1`). | Good with older computers that still have the connector. |
| The cable connected to a computer through a [USB converter](https://www.amazon.com/gp/product/B0007T27H8) (unix: `/dev/ttyUSB0`, windows: `COM1`). | Good for newer computers and maker boards such as Raspberry Pis. |
| Multiple cables connected to one port via a [splitter](https://www.amazon.com/gp/product/B007F2E188) and Alicat's addressing (`A`-`Z`). | Good when number of ports is limited. |

Installation
============

```
pip install alicat
```

Usage
=====

## Command Line

For basic tasks, this driver includes a command-line interface. Read the help
for more.

```
alicat --help
```

## Python

For more complex projects, use python to automate your workflow.

```python
from alicat import FlowController
flow_controller = FlowController(port='/dev/ttyUSB0')
print(flow_controller.get())
```

If the flow controller is communicating on the specified port, this should
return a dictionary of the form:

```python
{
  'setpoint': 0.0,         # Setpoint, either mass flow rate or pressure
  'control_point': 'flow', # Either 'flow' or 'pressure'
  'gas': 'Air',            # Can be any option in `flow_controller.gases`
  'mass_flow': 0.0,        # Mass flow (in units specified at time of purchase)
  'pressure': 25.46,       # Pressure (normally in psia)
  'temperature': 23.62,    # Temperature (normally in C)
  'total_flow': 0.0,       # Optional. If totalizer function purchased, will be included
  'volumetric_flow': 0.0   # Volumetric flow (in units specified at time of purchase)
}
```

On flow controllers, you can set the flow or pressure setpoints.

```python
flow_controller.set_flow_rate(1.0)
flow_controller.set_pressure(20)
```

### Gas Type

You can set the gas type by name or by index. For more on setting by index, see the gas table in your Alicat's manual.

```python
flow_controller.set_gas('N2')
flow_controller.set_gas(8)
```

For firmware 5v and greater, you can create and set gas mixes. Mixes can contain up to five gases and are stored in gas indices 236-255.

```python
flow_controller.create_mix(mix_no=236, name="Mix1", gases={'N2': 50, 'O2': 30, 'CO2': 20})
flow_controller.set_gas(236)
flow_controller.delete_mix(236)
```

### PID Parameters

For flow controllers, read and write PID loop settings for device tuning.

```python
flow_controller.set_pid(p=4000, i=4000, d=10, loop_type='PD2I')
print(flow_controller.get_pid())
{
    'loop_type': 'PD2I',
    'P': '4000',
    'I': '4000',
    'D': '10',
}
```

### Other Features

Additional features include override commands to increase device functionality.

```python
flow_controller.lock()            # Lock the front display.
flow_controller.unlock()          # Unlock the front display.
flow_controller.hold()            # Hold the valve in its current position.
flow_controller.cancel_hold()     # Cancel the valve hold.
flow_controller.tare_volumetric() # Tare volumetric flow.
flow_controller.tare_pressure()   # Tare pressure.
flow_controller.reset_totalizer() # Reset totalizer, if totalizer functionality included.
```

### Addressing

You can have multiple controllers on the same port by using Alicat's `A`-`Z` addresses
and an [RS-232 splitter](https://www.amazon.com/gp/product/B007F2E188).

```python
flow_controller_1 = FlowController(address='A')
flow_controller_2 = FlowController(address='B')

flow_controller_1.set_flow_rate(1.0)
flow_controller_2.set_flow_rate(0.5)

flow_controller_1.close() # /dev/ttyUSB0 is still open!
flow_controller_2.close()
```

TCP Support
===

The last version with working TCP support is [here](https://github.com/numat/alicat/tree/8af92647cb396401c0d604d83fb95a49d9a82be9) and can be installed with:

```
pip install alicat==0.3.1
```

If you use the TCP driver, please let us know and we'll add it back to the current version.
