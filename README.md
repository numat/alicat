BASIS
======

Serial driver and command line tool for
[Alicat BASIS devices](https://www.alicat.com/articles/basis-2-electronic-mass-flow-meters-and-controllers-expand-to-100-slpm/).

<p align="center">
  <img src="https://www.alicat.com/wp-content/uploads/2024/12/basis-bc-series-controller-prod-900px.webp" height="400" />
</p>

*If you are using standard Alicat devices, go to [this repository](https://github.com/numat/alicat)* 

Installation
============

```
git clone https://github.com/avichalk/alicat/tree/basis-support
```

NOTE: The downloaded repository will have to be added to $PATH on your system to be properly imported. Otherwise, your script should be placed in the repository's parent folder.  

Usage
=====

To communicate with a BASIS device, please use the following syntax.


```python
import asyncio
from alicat import basis

async def get():
    async with basis.FlowController(address = "com_port", unit = "unit_id") as flow_controller:
        print(await flow_controller.get())

asyncio.run(get())
```

If the flow controller is communicating on the specified port, this should
return a dictionary of the form:

```python
{
  'setpoint': 0.0,         # Setpoint, either mass flow rate or pressure
  'gas': 'Air',            # Can be any option in `flow_controller.gases`
  'mass_flow': 0.0,        # Mass flow (in units specified at time of purchase)
  'temperature': 23.62,    # Temperature (normally in C)
  'volumetric_flow': 0.0   # Volumetric flow (in units specified at time of purchase)
}
```

On flow controllers, you can set the flow setpoint.

```python
await flow_controller.set_flow_rate(1.0)
```

### Gas Type

You can set the gas type by name or by index. For more on setting by index, see the gas table in your Alicat's manual.

```python
await flow_controller.set_gas('N2')
await flow_controller.set_gas(8)
```

### PID Parameters

For flow controllers, read and write PID loop settings for device tuning.

```python
await flow_controller.set_pid(p=4000, i=4000, d=10, loop_type='PD2I')
print(await flow_controller.get_pid())
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
await flow_controller.hold()            # Hold the valve in its current position.
await flow_controller.cancel_hold()     # Cancel the valve hold.
await flow_controller.reset_totalizer() # Reset totalizer, if totalizer functionality included.
```

### Addressing

You can have multiple controllers on the same port by using Alicat's `A`-`Z` addresses
and an [RS-232 splitter](https://www.amazon.com/gp/product/B007F2E188).

```python
flow_controller_1 = FlowController(address='A')
flow_controller_2 = FlowController(address='B')

await flow_controller_1.set_flow_rate(1.0)
await flow_controller_2.set_flow_rate(0.5)

await flow_controller_1.close() # /dev/ttyUSB0 is still open!
await flow_controller_2.close()
```
