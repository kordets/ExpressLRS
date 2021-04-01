#!/usr/bin/env python
import sys, os

output = "expresslrs_packet.bin"


if len(sys.argv) != 4:
    print("[ERROR] usage: elrs_bin_pack.py <bl_size> <bl_bin> <elrs_bin>\n\n");
    sys.exit()

bl_size = eval(sys.argv[1])
bl_file = sys.argv[2]
app_file = sys.argv[3]
print("")
if bl_file:
    print("Bootloader:")
    print("    file: %s" % bl_file)
    print("    size: %u" % bl_size)
print("ExpressLRS:")
print("    file: %s" % app_file)
print("")

print("packing...")

with open(output, "wb") as _out:
    if bl_file:
        with open(bl_file, "rb") as _bl:
            buff = _bl.read()
            buff_len = len(buff)
            if buff_len:
                if buff_len != bl_size:
                    buff = buff + bytes(bl_size - buff_len)
                _out.write(buff)

    with open(app_file, "rb") as _app:
        _out.write(_app.read())

print("    done!\n")
