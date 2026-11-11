"""Keep the validated ordinary-operation NICCE images byte-identical."""
from pathlib import Path
import hashlib
import re

source = (Path(__file__).resolve().parents[1] / 'hal/hal8852b_fw.c').read_text()
expected = {
    'array_8852b_u2_nicce': 'fb8a2ae15324449745462ca2005bf3d1507a723168f0d357537acecb4dba84fc',
    'array_8852b_u3_nicce': 'ba0dc6d2a86f4430bb99f89f5582c83d4ff900c03f1eee813c81ee03477dc3c6',
}
for name, digest in expected.items():
    body = re.search(r'\b' + name + r'\[\]\s*=\s*\{(.*?)\};', source, re.S)[1]
    image = bytes(int(byte, 16) for byte in re.findall(r'0x([0-9a-fA-F]{2})\b', body))
    assert hashlib.sha256(image).hexdigest() == digest, name
print('PASS: both baseline NICCE images remain byte-identical')
