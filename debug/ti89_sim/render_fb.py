import sys
words = []
with open('/tmp/ti89_sim/ram_dump.hex') as f:
    for line in f:
        line = line.strip()
        if line:
            words.append(int(line, 16))
BASE = 0x4C00
W, H = 160, 100
rows = []
for y in range(H):
    row = []
    for x in range(W):
        bofs = BASE + y*30 + (x >> 3)
        w = words[bofs >> 1]
        b = (w >> 8) if (bofs & 1) == 0 else (w & 0xFF)
        px = (b >> (7 - (x & 7))) & 1
        row.append(px)
    rows.append(row)
with open(sys.argv[1] if len(sys.argv) > 1 else '/tmp/ti89_sim/fb_render.pbm', 'w') as f:
    f.write(f'P1\n{W} {H}\n')
    for row in rows:
        f.write(''.join(str(p) for p in row) + '\n')
n = sum(sum(r) for r in rows)
print(f"pixels on: {n}/{W*H} ({100.0*n/(W*H):.1f}%)")
