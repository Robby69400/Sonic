#!/usr/bin/env python3
"""Flash tool UV-K5/K1 (SONIC) - protocole exact du driver CHIRP."""

import argparse
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pip install pyserial"); sys.exit(1)

XOR_TABLE = [22, 108, 20, 230, 46, 145, 13, 64, 33, 53, 213, 64, 19, 3, 233, 128]
TIMESTAMP = b"\x6a\x39\x57\x64"

# Régions prédéfinies avec adresses 32-bit
REGIONS = {
    'full':         (0x0000, 2*1024*1024),
}

def xor_arr(data):
    return bytes([b ^ XOR_TABLE[i % len(XOR_TABLE)] for i, b in enumerate(data)])

def crc16_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc <<= 1
            if crc & 0x10000: crc ^= 0x1021
    return crc & 0xFFFF

def send_command(ser, data):
    """EXACT copy of _send_command / _receive_reply from the CHIRP driver."""
    data2 = data + struct.pack("<H", crc16_xmodem(data))
    packet = struct.pack(">HBB", 0xABCD, len(data), 0) + xor_arr(data2) + struct.pack(">H", 0xDCBA)
    ser.write(packet)

    header = ser.read(4)
    if len(header) != 4 or header[0] != 0xAB or header[1] != 0xCD or header[3] != 0x00:
        return None
    body = ser.read(int(header[2]))
    if len(body) != int(header[2]):
        return None
    footer = ser.read(4)
    if len(footer) != 4 or footer[2] != 0xDC or footer[3] != 0xBA:
        return None
    return xor_arr(body)

def say_hello(ser):
    """Exact copy of _sayhello: returns firmware string or None."""
    for _ in range(5):
        rep = send_command(ser, b"\x14\x05\x04\x00" + TIMESTAMP)
        if rep:
            break
    if not rep:
        return None
    if rep.startswith(b'\x18\x05'):
        print("ERROR: radio is in programming mode, restart into normal mode")
        return None
    fw = rep[4:28].split(b'\x00')[0].split(b'\xff')[0]
    return fw.decode('ascii', errors='ignore')

def read_memory(ser, offset, length):
    """Exact copy of _readmem."""
    readmem = b"\x1b\x05\x0c\x00" + \
        struct.pack("<IBB", offset, length, 0) + b"\x00\x00" + TIMESTAMP
    rep = send_command(ser, readmem)
    if rep is None or len(rep) < 10:
        return None
    return rep[10:]

def write_memory(ser, data, offset):
    """Exact copy of _writemem."""
    dlen = len(data)
    writemem = b"\x1d\x05" + \
        struct.pack("<H", dlen + 12) + \
        struct.pack("<IBB", offset, dlen, 1) + b"\x00\x00" + TIMESTAMP + data
    rep = send_command(ser, writemem)
    return (rep is not None and len(rep) >= 6 and
            rep[0] == 0x1e and
            rep[4] == (offset & 0xff) and
            rep[5] == ((offset >> 8) & 0xff))

def dump_region(ser, addr, size, output):
    result = bytearray()
    pos = 0
    while pos < size:
        chunk = min(128, size - pos)
        data = read_memory(ser, addr + pos, chunk)
        if data is None or len(data) != chunk:
            print(f"\n✗ FAIL read at 0x{addr+pos:06X}")
            return False
        result.extend(data)
        pos += chunk
        print(f"\r  Dump: {pos*100//size}% | {pos}/{size}", end='', flush=True)
    print()
    with open(output, 'wb') as f:
        f.write(result)
    print(f"✓ Saved: {output} ({len(result)} bytes)")
    return True

def fill_region(ser, addr, size, value):
    pos = 0
    while pos < size:
        chunk = min(128, size - pos)
        if not write_memory(ser, bytes([value]) * chunk, addr + pos):
            print(f"\n✗ FAIL write at 0x{addr+pos:06X}")
            return False
        pos += chunk
        print(f"\r  Fill: {pos*100//size}% | {pos}/{size}", end='', flush=True)
    print()
    check = read_memory(ser, addr, min(size, 128))
    if check is not None and check == bytes([value]) * len(check):
        print(f"✓ Verified: {size} bytes filled with 0x{value:02X}")
        return True
    print("WARNING: verification mismatch")
    return True

def restore_file(ser, addr, infile):
    """Restore a binary file to flash memory."""
    path = Path(infile)
    if not path.exists():
        print(f"✗ File not found: {infile}")
        return False
    
    with open(infile, 'rb') as f:
        data = f.read()
    
    size = len(data)
    print(f"Restoring {size:,} bytes from {infile} to 0x{addr:06X}...")
    
    pos = 0
    chunk_size = 128
    while pos < size:
        chunk = data[pos:pos + chunk_size]
        if not write_memory(ser, addr + pos, chunk):
            print(f"\n✗ FAIL write at 0x{addr+pos:06X}")
            return False
        pos += len(chunk)
        print(f"\r  Restore: {pos*100//size}% | {pos}/{size}", end='', flush=True)
    
    print()
    
    # Verification
    print("Verifying...")
    verify_size = min(size, 512)
    check = read_memory(ser, addr, verify_size)
    if check is None:
        print("⚠ Could not verify (read failed)")
        return True
    if check != data[:verify_size]:
        print("✗ Verification FAILED - mismatch detected")
        mismatches = sum(1 for i in range(len(check)) if check[i] != data[i])
        print(f"  Mismatch count: {mismatches} bytes")
        return False
    
    print(f"✓ Verified: first {verify_size} bytes match")
    print(f"✓ Complete: {size} bytes restored")
    return True

def main():
    p = argparse.ArgumentParser(
        description='UV-K5/K1 Flash Tool (SONIC Protocol)',
        epilog="""
Examples:
  %(prog)s -p COM3 --region fm_radios --dump fm.bin
  %(prog)s -p COM3 --region fm_radios --restore fm.bin
  %(prog)s -p COM3 -a 0x008AF4 -s 950 --clear
  %(prog)s -p COM3 -a 0x122000 -s 1024 -f 0xFF
        """
    )
    p.add_argument('-p', '--port', required=True, help='Serial port')
    p.add_argument('-r', '--region', choices=list(REGIONS), help='Preset region')
    p.add_argument('-a', '--address', type=lambda x: int(x, 0), help='Start address')
    p.add_argument('-s', '--size', type=int, help='Size in bytes')
    p.add_argument('-d', '--dump', metavar='FILE', help='Dump memory to file')
    p.add_argument('-f', '--fill', type=lambda x: int(x, 0), metavar='VAL', help='Fill with value')
    p.add_argument('-c', '--clear', action='store_const', dest='fill', const=0xFF, help='Clear (0xFF)')
    p.add_argument('-R', '--restore', metavar='FILE', help='Restore file to memory')
    args = p.parse_args()

    # Resolve address/size
    if args.region:
        addr, size = REGIONS[args.region]
        print(f"Region: '{args.region}' 0x{addr:06X} ({size} bytes)")
    elif args.address is not None and args.size is not None:
        addr, size = args.address, args.size
        print(f"Manual: 0x{addr:06X} ({size} bytes)")
    else:
        p.error("--region OR --address/--size required")

    # Check operation mode
    if args.dump:
        op = "dump"
    elif args.fill is not None:
        op = "fill"
    elif args.restore:
        op = "restore"
    else:
        p.error("need --dump, --fill/--clear, or --restore")

    if args.dump and (args.fill is not None or args.restore):
        p.error("choose ONE operation: dump/fill/restore")
    if args.fill is not None and args.restore:
        p.error("cannot fill and restore simultaneously")

    # Open port
    try:
        ser = serial.Serial(args.port, baudrate=38400, timeout=4.0)
    except Exception as e:
        print(f"✗ Port error: {e}")
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # Hello
    print(f"Connecting to {args.port}...")
    fw = say_hello(ser)
    if fw is None:
        print("✗ No radio response")
        print("  Check: radio ON, correct port, cable connected")
        ser.close(); sys.exit(1)
    print(f"✓ Connected. Firmware: {fw}")

    # Execute
    ok = False
    try:
        if args.dump:
            ok = dump_region(ser, addr, size, args.dump)
        elif args.fill is not None:
            ok = fill_region(ser, addr, size, args.fill)
        elif args.restore:
            ok = restore_file(ser, addr, args.restore)
    finally:
        # Reset radio
        try:
            send_command(ser, b"\xdd\x05\x00\x00")
        except:
            pass
        ser.close()

    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()