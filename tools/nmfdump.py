"""nmfdump - list the objects inside a Workers & Resources .nmf model.

Format as read from CR400AF (Workshop 3019370041), not from any spec:
  "fromObj\\0", u32 a, u32 b, u32 file size
  a x 64-byte name table (materials / parts)
  object blocks, chained: at S: u32 size; S+4: name[64] (UTF-8, NUL-padded);
    S+68: u32; S+72: 4x4 float matrix; S+136: 4x4 float matrix;
    S+200: bbox min xyz, max xyz (floats); then u32 fields (sizes, counts,
    flags) and the vertex/index payload. The next block's size field is at
    S + size.
"""
import struct
import sys


def objects(data):
    tab = struct.unpack_from('<I', data, 8)[0]
    s = 0x14 + tab * 64 + 4
    while s + 232 <= len(data):
        size = struct.unpack_from('<I', data, s)[0]
        raw = data[s + 4:s + 68].split(b'\0')[0]
        try:
            name = raw.decode('utf-8')
        except UnicodeDecodeError:
            break
        if not name or size < 232:
            break
        m1 = struct.unpack_from('<16f', data, s + 72)
        m2 = struct.unpack_from('<16f', data, s + 136)
        bb = struct.unpack_from('<6f', data, s + 200)
        tail = struct.unpack_from('<10I', data, s + 224)
        yield s, size, name, m1, m2, bb, tail
        s += size


def main():
    for path in sys.argv[1:]:
        data = open(path, 'rb').read()
        print(f"== {path}  ({len(data)} bytes, header {struct.unpack_from('<3I', data, 8)})")
        for s, size, name, m1, m2, bb, tail in objects(data):
            ident = all(abs(v - (1.0 if i % 5 == 0 else 0.0)) < 1e-6 for i, v in enumerate(m1)) and \
                    all(abs(v - (1.0 if i % 5 == 0 else 0.0)) < 1e-6 for i, v in enumerate(m2))
            mn, mx = bb[:3], bb[3:]
            print(f"  {name:<22} x {mn[0]:7.2f}..{mx[0]:6.2f}  y {mn[1]:6.2f}..{mx[1]:6.2f}  "
                  f"z {mn[2]:7.2f}..{mx[2]:7.2f}  len {mx[2]-mn[2]:5.2f}  "
                  f"{'identity' if ident else 'MATRIX'}  u={list(tail[:8])}")
            if not ident:
                print("      m1:", ['%.3g' % v for v in m1])
                print("      m2:", ['%.3g' % v for v in m2])


if __name__ == '__main__':
    main()
