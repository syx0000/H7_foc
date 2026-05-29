"""Post-build script: inject Stage 2 OTA header into the application hex.

Inputs (default paths relative to this script's parent dir = repo root):
  MDK-ARM/cubemx_yxsui/cubemx_yxsui.bin    fromelf output (raw application image)

Outputs:
  MDK-ARM/cubemx_yxsui/cubemx_yxsui_signed.hex   App-A region + App-A header

Usage:
  Called from MDK "After Build" command line:
    py ../tools/build_app_hex.py --version=1

Or manually:
  py tools/build_app_hex.py --version=2 --bin path/to.bin --out path/to.hex

The header layout MUST match Core/Inc/ota_app.h::app_header_t:
  uint32 magic = 'FOCA' (0x41434F46)
  uint32 version
  uint32 app_size
  uint32 app_crc32 (IEEE 802.3, same as Flash_Crc32 / zlib.crc32)
  uint32 boot_count = 0
  uint32 flags = 0x01 (bit0=valid)
  uint32 build_time = 0
  uint32 reserved[25]   (pad to 128 bytes)

Slot addresses (Stage 2 layout, see wiggly-knitting-journal.md §12.1):
  App-A app    : 0x08020000  (Bank1 Sector 1~6, 768KB)
  App-A header : 0x080E0000  (Bank1 Sector 7,   128KB, only first 128B used)
"""

import argparse
import os
import struct
import sys
import zlib

try:
    from intelhex import IntelHex
except ImportError:
    sys.stderr.write(
        "ERROR: intelhex not installed. Run: py -m pip install intelhex\n"
    )
    sys.exit(2)


APP_HEADER_MAGIC = 0x41434F46  # 'FOCA'
APP_A_BASE       = 0x08020000
APP_A_HEADER_BASE = 0x080E0000
HEADER_SIZE      = 128
APP_SLOT_SIZE    = 768 * 1024


def build_header(version: int, app_size: int, app_crc32: int) -> bytes:
    """Pack app_header_t (128 bytes)."""
    return struct.pack(
        "<IIIIIII100x",
        APP_HEADER_MAGIC,
        version,
        app_size,
        app_crc32,
        0,        # boot_count
        0x01,     # flags: bit0=valid
        0,        # build_time (unused for now)
    )


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    default_bin = os.path.join(repo, "MDK-ARM", "cubemx_yxsui", "cubemx_yxsui.bin")
    default_out = os.path.join(repo, "MDK-ARM", "cubemx_yxsui", "cubemx_yxsui.hex")

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bin", default=default_bin, help=f"Input .bin (default: {default_bin})")
    p.add_argument("--out", default=default_out, help=f"Output .hex (default: {default_out})")
    p.add_argument("--version", type=int, default=1, help="Firmware version (integer)")
    args = p.parse_args()

    if not os.path.isfile(args.bin):
        sys.stderr.write(f"ERROR: input bin not found: {args.bin}\n")
        return 1

    with open(args.bin, "rb") as f:
        data = f.read()

    if len(data) > APP_SLOT_SIZE:
        sys.stderr.write(
            f"ERROR: app size {len(data)} > slot capacity {APP_SLOT_SIZE}\n"
        )
        return 1

    crc32 = zlib.crc32(data) & 0xFFFFFFFF
    header = build_header(args.version, len(data), crc32)
    assert len(header) == HEADER_SIZE

    ih = IntelHex()
    ih.frombytes(data, offset=APP_A_BASE)
    ih.frombytes(header, offset=APP_A_HEADER_BASE)
    ih.write_hex_file(args.out)

    print(
        f"build_app_hex: version={args.version}  size={len(data)}  "
        f"crc=0x{crc32:08X}  out={args.out}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
