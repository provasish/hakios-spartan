#!/usr/bin/env python3
"""Check that HakiOS's U-Boot keeps its hands off every carve-out of the phone's own Android device tree.

usage: devices/spartan/tools/check-memory.py ANDROID.dtb [U-BOOT.dtb]

ANDROID.dtb is the phone's own tree (adb shell su -c 'cat /sys/firmware/fdt' > devices/spartan/prebuilt/device-tree.dtb),
U-BOOT.dtb the one U-Boot was built with (u-boot/build-spartan/u-boot.dtb). With one argument it prints what the Android
tree reserves and what this port has to agree with (the ids ABL matches, the keys, the panel's memory); with two it also
says, for every fixed carve-out, whether U-Boot's tree reserves all of it, and exits 1 if any is not, or if the HakiOS
payload area (0xF0000000, 64 MiB) falls on one. Regions with only a size (Linux's CMA pools) are not the firmware's and are
listed but not checked. Only reads; nothing on the phone or in the trees is changed.
"""
import struct
import sys

PAYLOAD = (0xF0000000, 0x4000000)


def parse(path):
    blob = open(path, 'rb').read()
    if blob[:4] != b'\xd0\x0d\xfe\xed':
        sys.exit('check-memory.py: %s is not a flattened device tree' % path)
    off_struct, off_strings = struct.unpack('>II', blob[8:16])
    size_strings, = struct.unpack('>I', blob[32:36])
    strings = blob[off_strings:off_strings + size_strings]
    nodes, path_parts, at = {}, [], off_struct
    while True:
        token, = struct.unpack('>I', blob[at:at + 4])
        at += 4
        if token == 1:                                  # FDT_BEGIN_NODE
            end = blob.index(b'\0', at)
            path_parts.append(blob[at:end].decode())
            at = (end + 4) & ~3
            nodes['/'.join(path_parts) or '/'] = {}
        elif token == 2:                                # FDT_END_NODE
            path_parts.pop()
        elif token == 3:                                # FDT_PROP
            length, name_off = struct.unpack('>II', blob[at:at + 8])
            at += 8
            name = strings[name_off:strings.index(b'\0', name_off)].decode()
            nodes['/'.join(path_parts) or '/'][name] = blob[at:at + length]
            at = (at + length + 3) & ~3
        elif token == 9:                                # FDT_END
            return nodes
        elif token != 4:                                # 4 = FDT_NOP
            sys.exit('check-memory.py: %s: unexpected token %d' % (path, token))


def cells(value):
    return [struct.unpack('>I', value[i:i + 4])[0] for i in range(0, len(value) - len(value) % 4, 4)]


def number(words):
    n = 0
    for w in words:
        n = (n << 32) | w
    return n


def reserved(nodes):
    """(name, base, size) for every child of /reserved-memory with a reg; (name, None, size) for one with a size only."""
    top = next((p for p in nodes if p.rsplit('/', 1)[-1] == 'reserved-memory' and p.count('/') == 1), None)
    if top is None:
        return []
    ac = number(cells(nodes[top].get('#address-cells', struct.pack('>I', 2))))
    sc = number(cells(nodes[top].get('#size-cells', struct.pack('>I', 2))))
    out = []
    for path, props in nodes.items():
        if path.rsplit('/', 1)[0] != top:
            continue
        name = path.rsplit('/', 1)[-1]
        if 'reg' in props:
            words = cells(props['reg'])
            for i in range(0, len(words) - (ac + sc) + 1, ac + sc):
                out.append((name, number(words[i:i + ac]), number(words[i + ac:i + ac + sc])))
        elif 'size' in props:
            out.append((name, None, number(cells(props['size']))))
    return out


def uncovered(base, size, regions):
    """The parts of [base, base + size) that none of regions covers."""
    gaps, at, end = [], base, base + size
    for b, s in sorted((b, s) for _, b, s in regions if b is not None):
        if b > at and b < end:
            gaps.append((at, min(b, end)))
        at = max(at, b + s)
        if at >= end:
            break
    if at < end:
        gaps.append((at, end))
    return gaps


def by_phandle(nodes):
    out = {}
    for path, props in nodes.items():
        for key in ('phandle', 'linux,phandle'):
            if key in props:
                out[number(cells(props[key]))] = path
    return out


def show_board(nodes):
    root = nodes.get('/', {})
    if 'model' in root:
        print('model: %s' % root['model'].rstrip(b'\0').decode(errors='replace'))
    for key in ('qcom,msm-id', 'qcom,board-id'):
        if key in root:
            print('%s: %s' % (key, ' '.join('0x%x' % w for w in cells(root[key]))))
    handles = by_phandle(nodes)
    for path, props in nodes.items():
        if b'gpio-keys' not in props.get('compatible', b'').split(b'\0'):
            continue
        for child, cprops in nodes.items():
            if child.rsplit('/', 1)[0] != path or 'gpios' not in cprops:
                continue
            words = cells(cprops['gpios'])
            code = number(cells(cprops['linux,code'])) if 'linux,code' in cprops else None
            controller = handles.get(words[0], 'phandle 0x%x' % words[0]) if words else '?'
            print('key %s: %s pin %s, code %s' % (child.rsplit('/', 1)[-1], controller, words[1] if len(words) > 1 else '?', code))


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__.strip().split('\n\n')[1])
    android = parse(sys.argv[1])
    show_board(android)
    regions = reserved(android)
    print('\n%s reserves:' % sys.argv[1])
    for name, base, size in sorted(regions, key=lambda r: (r[1] is None, r[1] or 0)):
        where = '%#012x-%#012x' % (base, base + size) if base is not None else '(dynamic)'.ljust(25)
        print('  %s %8d KiB  %s' % (where, size // 1024, name))

    bad = 0
    for name, base, size in regions:
        if base is not None and base < PAYLOAD[0] + PAYLOAD[1] and PAYLOAD[0] < base + size:
            print('\nPAYLOAD AREA: %s (%#x, %#x) overlaps the loader\'s 0xF0000000 area' % (name, base, size))
            bad += 1
    if len(sys.argv) == 3:
        mine = reserved(parse(sys.argv[2]))
        print('\nagainst %s:' % sys.argv[2])
        for name, base, size in sorted((r for r in regions if r[1] is not None), key=lambda r: r[1]):
            gaps = uncovered(base, size, mine)
            if gaps:
                bad += 1
                print('  NOT COVERED  %-28s %s' % (name, ', '.join('%#x-%#x' % g for g in gaps)))
            else:
                print('  covered      %s' % name)
    if bad:
        print('\n%d problem(s): add the regions above to the reserved-memory of sm8250-realme-spartan.dts' % bad)
        sys.exit(1)
    if len(sys.argv) == 3:
        print('\nevery fixed carve-out of the Android tree is reserved by U-Boot\'s')


if __name__ == '__main__':
    main()
