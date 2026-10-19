# Simulates Arduino IDE auto-generating function prototypes in a .ino.
# Usage: python3 arduino_prototype_sim.py IN.ino OUT.ino after_includes|before_first_function
# Compile OUT.ino with test_sketch.cpp to catch "does not name a type" errors.
import re,sys
src=open(sys.argv[1]).read().split('\n')
pat=re.compile(r'^(?!static|struct|enum|using|#|//|\s)([A-Za-z_][\w:<>&\* ]*?[\s&\*])([A-Za-z_]\w*)\(([^;]*)\)\s*\{')
protos=[];first=None
for i,l in enumerate(src):
    m=pat.match(l)
    if m:
        protos.append(f"{m.group(1).strip()} {m.group(2)}({m.group(3)});")
        if first is None: first=i
last_inc=max(i for i,l in enumerate(src) if l.startswith('#include'))
mode=sys.argv[3]
at = last_inc+1 if mode=='after_includes' else first
out=src[:at]+protos+src[at:]
open(sys.argv[2],'w').write('\n'.join(out))
print(len(protos),'prototypes inserted at line',at+1)
