alicat
======

Serial driver and command line tool for
[Alicat mass flow controllers](http://www.alicat.com/products/mass-flow-meters-and-controllers/mass-flow-controllers/).

<p align="center">
  <img src="http://www.alicat.com/wpinstall/wp-content/uploads/2012/01/gas-mass-flow-controller1.jpg" height="400" />
</p>

###### If you are using Analyt-MTC flow controllers, go to [this repository](https://github.com/schlenzmeister/AnalytMTC/wiki) for more info.

Example Connections
===================

 * The standard [DB9 cable](http://www.alicat.com/wpinstall/wp-content/uploads/2013/07/MD8DB9.jpg) connected directly to a computer (unix: `/dev/ttyS0`, windows: `COM1`).
   * Good with older computers that still have the connector.
 * The cable connected to a computer through a [USB converter](https://www.amazon.com/gp/product/B0007T27H8) (unix: `/dev/ttyUSB0`, windows: `COM1`).
   * Good for newer computers and maker boards such as Raspberry Pis.
 * Cables routed through a [TCP device server](https://www.amazon.com/gp/product/B00I5EYB2Q) (`tcp://192.168.1.100:4000`, requires python >3.4).
    * Good in conjunction with PLCs for professional-looking control boxes.
 * Multiple cables connected to one port via a [splitter](https://www.amazon.com/gp/product/B007F2E188) and Alicat's addressing (`A`-`D`).
    * Good when number of ports is limited.

Installation
============

```
pip install alicat
```

If you don't like pip, you can also install from source:

```
git clone https://github.com/numat/alicat.git
cd alicat
python setup.py install
```


Usage
=====

### Command Line

For basic tasks, this driver includes a command-line interface. Read the help
for more.

```
alicat --help
```

### Python

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

You can also set the gas type and flow rate / pressure.

```python
flow_controller.set_gas('N2')
flow_controller.set_flow_rate(1.0)
flow_controller.set_pressure(20)
```

### Alicat Addressing

You can have multiple controllers on the same port by using Alicat's `A`-`D` addresses
and an [RS-232 splitter](https://www.amazon.com/gp/product/B007F2E188).

```python
flow_controller_1 = FlowController(address='A')
flow_controller_2 = FlowController(address='B')

flow_controller_1.set_flow_rate(1.0)
flow_controller_2.set_flow_rate(0.5)

flow_controller_1.close() #/dev/ttyUSB0 is still open!
flow_controller_2.close()
```

### Asynchronous TCP

Some people wire their RS-232 devices through an ethernet proxy server ([example](https://www.amazon.com/gp/product/B00I5EYB2Q)), enabling network
access. This is supported through asyncio (python >3.4) and python's built-in
asynchronous syntax.

```python
import asyncio
from alicat.async import FlowController

flow_controller = FlowController('192.168.1.100', 4000)

async def print_state():
    print(await flow_controller.get())

ioloop = asyncio.get_event_loop()
ioloop.run_until_complete(print_state())
ioloop.close()
```
