from config import *
import sys

if FMU_JTAG is None:
    print("No JTAG device for FMU found")
    sys.exit(1)

if IO_JTAG is None:
    print("No JTAG device for IO found")
    sys.exit(1)

if BARCODE_SCANNER is None:
    print("No barcode scanner found")
    sys.exit(1)

