"""Python driver for Alicat mass flow controllers.

Distributed under the GNU General Public License v2
Copyright (C) 2019 NuMat Technologies
"""
from alicat.serial import FlowMeter, FlowController, command_line  # noqa


def run():
    """CLI interface, accessible when installed through pip."""
    import argparse

    parser = argparse.ArgumentParser(description="Control an Alicat mass "
                                     "flow controller from the command line.")
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0', help="The "
                        "target serial port. Default '/dev/ttyUSB0'.")
    parser.add_argument('--address', '-a', default='A', type=str, help="The "
                        "device address, A-Z. Should only be used if multiple "
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
    parser.add_argument("--unlock", "-u", action="store_true",
                        help="Unlocks device display.")
    parser.add_argument("--hold", "-hd", action="store_true",
                        help="Holds the valve at the present value.")
    parser.add_argument("--cancel-hold", "-c", action="store_true",
                        help="Cancel valve hold.")
    parser.add_argument("--reset-totalizer", "-r", action="store_true",
                        help="Reset current value of totalizer to zero.")
    args = parser.parse_args()

    command_line(args)


if __name__ == '__main__':
    run()
