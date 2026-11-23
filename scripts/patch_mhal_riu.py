#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Redirect the observed legacy mhal RIU helper through its local stub."""

import argparse
import struct
import sys

ELF32_EHDR_SHOFF = 0x20
ELF32_EHDR_SHENTSIZE = 0x2e
ELF32_EHDR_SHNUM = 0x30
ELF32_SHF_EXECINSTR = 0x4
SHT_SYMTAB = 2


def sections(data):
    shoff, = struct.unpack_from('<I', data, ELF32_EHDR_SHOFF)
    shentsize, = struct.unpack_from('<H', data, ELF32_EHDR_SHENTSIZE)
    shnum, = struct.unpack_from('<H', data, ELF32_EHDR_SHNUM)
    return [struct.unpack_from('<10I', data, shoff + i * shentsize)
            for i in range(shnum)]


def cstr(data, off):
    return data[off:data.index(0, off)].decode()


def symbols(data, sh):
    for section in sh:
        if section[1] != SHT_SYMTAB:
            continue
        strings = sh[section[6]]
        for off in range(section[4], section[4] + section[5], 16):
            name, value, size, info, other, index = struct.unpack_from('<IIIBBH', data, off)
            yield cstr(data, strings[4] + name), value, index


def thumb_branch(delta, link):
    if delta & 1 or not -(1 << 24) <= delta < (1 << 24):
        sys.exit('branch offset out of range: %#x' % delta)
    value = delta & 0x1ffffff
    s = value >> 24 & 1
    i1 = value >> 23 & 1
    i2 = value >> 22 & 1
    j1 = (~(i1 ^ s)) & 1
    j2 = (~(i2 ^ s)) & 1
    return (0xf000 | s << 10 | (value >> 12 & 0x3ff),
            (0xd000 if link else 0x9000) |
            j1 << 13 | j2 << 11 | (value >> 1 & 0x7ff))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--command-buffer-trap', action='store_true',
                    help='replace the RegBuf call with a one-shot Thumb UDF trap')
    ap.add_argument('module')
    args = ap.parse_args()
    data = bytearray(open(args.module, 'rb').read())
    sh = sections(data)
    syms = {name: (value, index) for name, value, index in symbols(data, sh)}
    try:
        func, section = syms['HalUtilityW2BYTEMSKDirectAndNoshadow']
        stub, stub_section = syms['osm_mhal_riu_r0']
        mov_stub, mov_stub_section = syms['osm_mhal_riu_mov_r3']
        add_stub, add_stub_section = syms['osm_mhal_riu_add_r0']
        add_r4_stub, add_r4_stub_section = syms['osm_mhal_riu_add_r4']
        add_r3_stub, add_r3_stub_section = syms['osm_mhal_riu_add_r3']
        cmdq_stub, cmdq_stub_section = syms['osm_mhal_riu_cmdq_r2']
        cmdq_field_stub, cmdq_field_stub_section = syms['osm_mhal_riu_cmdq_field_r2']
        r3_r0_stub, r3_r0_stub_section = syms['osm_mhal_riu_r3_r0tmp']
        r3_r1_stub, r3_r1_stub_section = syms['osm_mhal_riu_r3_r1tmp']
    except KeyError as e:
        sys.exit('missing symbol: %s' % e.args[0])
    if section != stub_section or not sh[section][2] & ELF32_SHF_EXECINSTR:
        sys.exit('unexpected code section')
    func &= ~1
    stub &= ~1

    if args.command_buffer_trap:
        write, write_section = syms['HalUtilityW2BYTEDirect']
        write &= ~1
        if write_section != section:
            sys.exit('unexpected HalUtilityW2BYTEDirect section')
        trapoff = sh[write_section][4] + write + 0x12
        old_first, old_second = struct.unpack_from('<HH', data, trapoff)
        if old_first & 0xf800 != 0xf000 or old_second & 0xd000 != 0xd000:
            sys.exit('unexpected HalUtilityW2BYTEDirect RegBuf call: %04x %04x' %
                     (old_first, old_second))
        # UDF enters the kernel's undefined-instruction/Oops path before the
        # command-buffer helper is invoked; KGDB then has the live registers.
        struct.pack_into('<HH', data, trapoff, 0xde00, 0xbf00)
        open(args.module, 'wb').write(data)
        print('%s: patched command-buffer trap' % args.module)
        return

    def patch_r0_helper(name, address):
        fileoff = sh[section][4] + address
        old = bytes(data[fileoff:fileoff + 6])
        expected = b'\x40\x00\x00\xf1\x7d\x40'
        if old != expected:
            sys.exit('unexpected %s prologue: %s' % (name, old.hex()))
        first, second = thumb_branch(stub - (address + 2 + 4), True)
        struct.pack_into('<HHH', data, fileoff, 0x46f4, first, second)

    patch_r0_helper('HalUtilityW2BYTEMSKDirectAndNoshadow', func)
    read, read_section = syms['HalUtilityR2BYTEDirect']
    read &= ~1
    if read_section != section:
        sys.exit('unexpected HalUtilityR2BYTEDirect section')
    patch_r0_helper('HalUtilityR2BYTEDirect', read)

    init, init_section = syms['HalUtilityInit']
    init &= ~1
    mov_stub &= ~1
    if init_section != mov_stub_section or not sh[init_section][2] & ELF32_SHF_EXECINSTR:
        sys.exit('unexpected HalUtilityInit code section')
    initoff = sh[init_section][4] + init + 4
    old = bytes(data[initoff:initoff + 4])
    expected = b'\x4f\xf0\x7d\x43'
    if old != expected:
        sys.exit('unexpected HalUtilityInit RIU move: %s' % old.hex())
    first, second = thumb_branch(mov_stub - (init + 4 + 4), False)
    struct.pack_into('<HH', data, initoff, first, second)
    stuboff = sh[mov_stub_section][4] + mov_stub
    old = bytes(data[stuboff + 4:stuboff + 8])
    if old != b'\xff\xf7\xfe\xbf':
        sys.exit('unexpected RIU move stub tail: %s' % old.hex())
    first, second = thumb_branch(init + 8 - (mov_stub + 4 + 4), False)
    struct.pack_into('<HH', data, stuboff + 4, first, second)

    direct, direct_section = syms['HalUtilityW2BYTEMSKDirect']
    direct &= ~1
    add_stub &= ~1
    if direct_section != add_stub_section or not sh[direct_section][2] & ELF32_SHF_EXECINSTR:
        sys.exit('unexpected HalUtilityW2BYTEMSKDirect code section')
    directoff = sh[direct_section][4] + direct + 0x12
    old = bytes(data[directoff:directoff + 4])
    if old != b'\x00\xf1\x7d\x40':
        sys.exit('unexpected HalUtilityW2BYTEMSKDirect RIU add: %s' % old.hex())
    first, second = thumb_branch(add_stub - (direct + 0x12 + 4), True)
    struct.pack_into('<HH', data, directoff, first, second)

    write, write_section = syms['HalUtilityW2BYTEDirect']
    write &= ~1
    add_r4_stub &= ~1
    if write_section != add_r4_stub_section or not sh[write_section][2] & ELF32_SHF_EXECINSTR:
        sys.exit('unexpected HalUtilityW2BYTEDirect code section')
    writeoff = sh[write_section][4] + write + 10
    old = bytes(data[writeoff:writeoff + 4])
    if old != b'\x04\xf1\x7d\x44':
        sys.exit('unexpected HalUtilityW2BYTEDirect RIU add: %s' % old.hex())
    first, second = thumb_branch(add_r4_stub - (write + 10 + 4), True)
    struct.pack_into('<HH', data, writeoff, first, second)

    write8, write8_section = syms['HalUtilityWBYTEMSKDirect']
    write8 &= ~1
    add_r3_stub &= ~1
    if write8_section != add_r3_stub_section:
        sys.exit('unexpected HalUtilityWBYTEMSKDirect section')
    write8off = sh[write8_section][4] + write8 + 0x1a
    old = bytes(data[write8off:write8off + 4])
    if old != b'\x03\xf1\x7d\x43':
        sys.exit('unexpected HalUtilityWBYTEMSKDirect RIU add: %s' % old.hex())
    first, second = thumb_branch(add_r3_stub - (write8 + 0x1a + 4), True)
    struct.pack_into('<HH', data, write8off, first, second)

    writecmdq, writecmdq_section = syms['HalUtilityW2BYTEMSKDirectCmdq']
    writecmdq &= ~1
    if writecmdq_section != add_r4_stub_section:
        sys.exit('unexpected HalUtilityW2BYTEMSKDirectCmdq section')
    writecmdqoff = sh[writecmdq_section][4] + writecmdq + 0x4e
    old = bytes(data[writecmdqoff:writecmdqoff + 4])
    if old != b'\x04\xf1\x7d\x44':
        sys.exit('unexpected HalUtilityW2BYTEMSKDirectCmdq RIU add: %s' % old.hex())
    first, second = thumb_branch(add_r4_stub - (writecmdq + 0x4e + 4), True)
    struct.pack_into('<HH', data, writecmdqoff, first, second)

    cmdq, cmdq_section = syms['HAL_CMDQ_SetBank']
    cmdq &= ~1
    cmdq_stub &= ~1
    if cmdq_section != cmdq_stub_section or not sh[cmdq_section][2] & ELF32_SHF_EXECINSTR:
        sys.exit('unexpected HAL_CMDQ_SetBank section')
    cmdqoff = sh[cmdq_section][4] + cmdq + 0x48
    old = bytes(data[cmdqoff:cmdqoff + 4])
    if old != b'\x02\xf5\x00\x11':
        sys.exit('unexpected HAL_CMDQ_SetBank base add: %s' % old.hex())
    first, second = thumb_branch(cmdq_stub - (cmdq + 0x48 + 4), True)
    struct.pack_into('<HH', data, cmdqoff, first, second)

    cmdq_field_stub &= ~1
    if cmdq_section != cmdq_field_stub_section:
        sys.exit('unexpected HAL_CMDQ_SetBank field stub section')
    cmdqfieldoff = sh[cmdq_section][4] + cmdq + 0x2c
    old = bytes(data[cmdqfieldoff:cmdqfieldoff + 4])
    if old != b'\x01\xeb\x42\x02':
        sys.exit('unexpected HAL_CMDQ_SetBank field add: %s' % old.hex())
    first, second = thumb_branch(cmdq_field_stub - (cmdq + 0x2c + 4), True)
    struct.pack_into('<HH', data, cmdqfieldoff, first, second)

    def patch_r3_helper(name, address, target, target_section):
        address &= ~1
        target &= ~1
        if target_section != section:
            sys.exit('unexpected %s stub section' % name)
        fileoff = sh[section][4] + address
        old = bytes(data[fileoff:fileoff + 6])
        if old != b'\x43\x00\x03\xf1\x7d\x43':
            sys.exit('unexpected %s RIU prologue: %s' % (name, old.hex()))
        first, second = thumb_branch(target - (address + 2 + 4), True)
        struct.pack_into('<HHH', data, fileoff, 0x46f4, first, second)

    readmask, readmask_section = syms['HalUtilityR2BYTEMaskDirect']
    if readmask_section != r3_r0_stub_section:
        sys.exit('unexpected HalUtilityR2BYTEMaskDirect section')
    patch_r3_helper('HalUtilityR2BYTEMaskDirect', readmask, r3_r0_stub,
                    r3_r0_stub_section)

    read4, read4_section = syms['HalUtilityR4BYTEDirect']
    if read4_section != r3_r1_stub_section:
        sys.exit('unexpected HalUtilityR4BYTEDirect section')
    patch_r3_helper('HalUtilityR4BYTEDirect first', read4, r3_r1_stub,
                    r3_r1_stub_section)
    patch_r3_helper('HalUtilityR4BYTEDirect second', (read4 & ~1) + 8,
                    r3_r1_stub, r3_r1_stub_section)

    open(args.module, 'wb').write(data)
    print('%s: patched %s, %s, %s, %s, %s, %s, %s, %s and %s' % (args.module,
          'HalUtilityW2BYTEMSKDirectAndNoshadow', 'HalUtilityR2BYTEDirect',
          'HalUtilityInit', 'HalUtilityW2BYTEMSKDirect', 'HalUtilityW2BYTEDirect',
          'HAL_CMDQ_SetBank base', 'HAL_CMDQ_SetBank field',
          'HalUtilityR2BYTEMaskDirect', 'HalUtilityR4BYTEDirect'))


if __name__ == '__main__':
    main()
