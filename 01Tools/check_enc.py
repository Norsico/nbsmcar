# -*- coding: utf-8 -*-
import os

def detect_encoding(filepath):
    with open(filepath, 'rb') as f:
        raw = f.read(4)
    if raw[:3] == b'\xef\xbb\xbf':
        return 'UTF-8-BOM'
    elif raw[:2] == b'\xff\xfe' or raw[:2] == b'\xfe\xff':
        return 'UTF-16'
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            f.read()
        return 'UTF-8'
    except UnicodeDecodeError:
        try:
            with open(filepath, 'r', encoding='gbk') as f:
                f.read()
            return 'GB2312/GBK'
        except:
            return 'Unknown'

dirs = ['project', '02libraries']
total = 0
non_utf8 = []
for d in dirs:
    if os.path.exists(d):
        for root, dirs_f, files in os.walk(d):
            for f in files:
                if f.endswith('.c') or f.endswith('.h'):
                    path = os.path.join(root, f)
                    enc = detect_encoding(path)
                    total += 1
                    if enc != 'UTF-8':
                        non_utf8.append(enc + '|' + path)

print('Total .c/.h files:', total)
print('Non-UTF-8 files:', len(non_utf8))
for x in non_utf8:
    print(x)