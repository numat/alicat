"""Python driver for Alicat mass flow controllers.

Distributed under the GNU General Public License v2
Copyright (C) 2023 NuMat Technologies
"""
from typing import Any

from alicat.driver import FlowController, FlowMeter  # noqa


def command_line(args: Any = None) -> None:
    """CLI interface, accessible when installed through pip."""
    import argparse
    import asyncio
    import json
    from time import time
    parser = argparse.ArgumentParser(description="Control an Alicat mass "
                                     "flow controller from the command line.")
    parser.add_argument('address', type=str, default='/dev/ttyUSB0', help="The "
                        "target serial port (or TCP address:port). Default '/dev/ttyUSB0'.")
    parser.add_argument('--unit', '-u', default='A', type=str, help="The "
                        "device unit ID, A-Z. Should only be used if multiple "
                        "flow controllers are connected to one port or if"
                        "device ID is not A.")
    parser.add_argument('--set-gas', '-g', default=None, type=str,
                        help="Sets the gas type. Supported gas types are: "
                             "'Air', 'Ar', 'CH4', 'CO', 'CO2', 'C2H6', 'H2', "
                             "'He', 'N2', 'N2O', 'Ne', 'O2', 'C3H8', "
                             "'n-C4H10', 'C2H2', 'C2H4', 'i-C2H10', 'Kr', "
                             "'Xe', 'SF6', 'C-25', 'C-10', 'C-8', 'C-2', "
                             "'C-75', 'A-75', 'A-25', 'A1025', 'Star29', "
                             "'P-5'")
    parser.add_argument('--set-flow-rate', '-f', default=None, type=float,
                        help="Sets the target flow rate of the controller.")
    parser.add_argument('--set-pressure', '-p', default=None, type=float,
                        help="Sets the target pressure of the controller.")
    parser.add_argument('--stream', '-s', action='store_true',
                        help="Sends a constant stream of flow controller "
                             "data, formatted as a tab-separated table.")
    parser.add_argument("--lock", "-l", action="store_true",
                        help="Locks device display.")
    parser.add_argument("--unlock", "-ul", action="store_true",
                        help="Unlocks device display.")
    parser.add_argument("--hold", "-hd", action="store_true",
                        help="Holds the valve at the present value.")
    parser.add_argument("--cancel-hold", "-c", action="store_true",
                        help="Cancel valve hold.")
    parser.add_argument("--reset-totalizer", "-r", action="store_true",
                        help="Reset current value of totalizer to zero.")
    args = parser.parse_args(args)
    async def get() -> None:
        async with FlowController(address=args.address, unit=args.unit) as flow_controller:
            if args.set_gas:
                await flow_controller.set_gas(args.set_gas)
            if args.set_flow_rate is not None and args.set_pressure is not None:
                raise ValueError("Cannot set both flow rate and pressure.")
            if args.set_flow_rate is not None:
                await flow_controller.set_flow_rate(args.set_flow_rate)
            if args.set_pressure is not None:
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
                await flow_controller.reset_totalizer()
            state = await flow_controller.get()
            if args.stream:
                try:
                    print('time\t' + '\t'.join(flow_controller.keys))
                    t0 = time()
                    while True:
                        state = await flow_controller.get()
                        print(f'{time() - t0:.2f}\t' +
                              '\t\t'.join(f'{state[key]:.2f}'
                                          for key in flow_controller.keys[:-1]) +
                              '\t\t' + state['gas'])
                except KeyboardInterrupt:
                    pass
            else:
                print(json.dumps(state, indent=2, sort_keys=True))
            await flow_controller.close()

    asyncio.run(get())


if __name__ == '__main__':
    command_line()
