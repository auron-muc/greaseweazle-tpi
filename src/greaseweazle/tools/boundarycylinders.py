# greaseweazle/tools/boundarycylinders.py
#
# Greaseweazle control script: Get/Set outer boundary cylinders.
#
# Written & released by Keir Fraser <keir.xen@gmail.com>
# Adapted by Tobias Meyer <floppymod@auron.de>
#
# This is free and unencumbered software released into the public domain.
# See the file COPYING for more details, or visit <http://unlicense.org>.

description = "Display (and optionally modify) drive cylinder boundaries."

import sys

from greaseweazle.tools import util
from greaseweazle import usb as USB

def main(argv) -> None:

    parser = util.ArgumentParser(usage='%(prog)s [options]')
    parser.add_argument("--device", help="device name (COM/serial port)")
    parser.add_argument("--min", type=int, metavar="N",
                        help="The outer cylinder the firmware will allow seeking to. Only used for flippy drives. Must be a negative number.")
    parser.add_argument("--max", type=int, metavar="N",
                        help="The innermost cylinder the firmware will allow seeking to. ")
    parser.add_argument("--I-Know-What-Im-Doing", type=bool, default=False, dest='IKWID')
    parser.description = description
    parser.prog += ' ' + argv[1]
    args = parser.parse_args(argv[2:])

    try:

        usb = util.usb_open(args.device)

        if args.IKWID:
            if args.min is not None:
                usb.min_cylinders = args.min
            if args.max is not None:
                usb.max_cylinders = args.max
        elif ( args.min is not None or args.max is not None):
            print("Not changing: You must agree that you know what you're doing. Changing these may harm your drive.")

        print("Outmost cylinder boundary (flippy): %u" % usb.min_cylinders)
        print("Innermost cylinder boundary:        %u" % usb.max_cylinders)

    except USB.CmdError as error:
        print("Command Failed: %s" % error)


if __name__ == "__main__":
    main(sys.argv)

# Local variables:
# python-indent: 4
# End:
