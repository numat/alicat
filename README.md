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
 * Multiple cables connected to one port via a [splitter](https://www.amazon.com/gp/product/B007F2E188) and Alicat's addressing (`A`-`Z`).
    * Good when number of ports is limited.

Installation
============

```
git clone https://github.com/marinapalese/alicat.git
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
flow_controller.set_gas(8)         # Optionally set a gas by it's number; find the full gas table on page 52 of the Alicat manual.
flow_controller.set_flow_rate(1.0)
flow_controller.set_pressure(20)
```

For firmwave 5v and greater, create and set gas mixes using COMPOSER software loaded into the device. Mixes can contain up to five gases, and are stored in gas indices 236-255. 

```python
flow_controller.create_mix(mix_no=236, name="Mix1", gas1="N2", percent1=50, gas2="Ar", percent2=50)
flow_controller.set_gas(236)
flow_controller.delete_mix(236)
```

Additional features include override commands to increase device functionality.

```python
flow_controller.lock()            # Lock the front display.
flow_controller.unlock()          # Unlock the front display.
flow_controller.hold()            # Hold the valve in its current position. 
flow_controller.cancel_hold()     # Cancel the valve hold.
flow_controller.tare_volumetric() # Tare volumetric hold.
flow_controller.tare_pressure()   # Tare pressure.
flow_controller.reset_tot()       # Reset totalizer, if totalizer functionality included.
```

For flow controllers, read and write PID loop settings for device tuning. 

```python
flow_controller.write_PID_looptype("PD2I")
flow_controller.write_PID_P(4000)
flow_controller.write_PID_D(10)
flow_controller.write_PID_I(4000)
print(flow_controller.read_PID())

>>>{
'loop_type': 'PD2I',
'P': '4000',
'D': '10',
'I': '4000'
}
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
from alicat.tcp import FlowController

flow_controller = FlowController('192.168.1.100', 4000)

async def print_state():
    print(await flow_controller.get())

ioloop = asyncio.get_event_loop()
ioloop.run_until_complete(print_state())
ioloop.close()
```
