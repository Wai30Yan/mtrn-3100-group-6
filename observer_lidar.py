#!/usr/bin/python

import re

h = """
abs(wk + nx*(txf + lx*cos(ttf) - ly*sin(ttf)) + ny*(tyf + ly*cos(ttf) + lx*sin(ttf)))/abs(nx*cos(lt + ttf) + ny*sin(lt + ttf))
"""

hj = """
[(nx*sign(wk + nx*(txf + lx*cos(ttf) - ly*sin(ttf)) + ny*(tyf + ly*cos(ttf) + lx*sin(ttf))))/abs(nx*cos(lt + ttf) + ny*sin(lt + ttf)), (ny*sign(wk + nx*(txf + lx*cos(ttf) - ly*sin(ttf)) + ny*(tyf + ly*cos(ttf) + lx*sin(ttf))))/abs(nx*cos(lt + ttf) + ny*sin(lt + ttf)), - (sign(wk + nx*(txf + lx*cos(ttf) - ly*sin(ttf)) + ny*(tyf + ly*cos(ttf) + lx*sin(ttf)))*(nx*(ly*cos(ttf) + lx*sin(ttf)) - ny*(lx*cos(ttf) - ly*sin(ttf))))/abs(nx*cos(lt + ttf) + ny*sin(lt + ttf)) - (abs(wk + nx*(txf + lx*cos(ttf) - ly*sin(ttf)) + ny*(tyf + ly*cos(ttf) + lx*sin(ttf)))*sign(nx*cos(lt + ttf) + ny*sin(lt + ttf))*(ny*cos(lt + ttf) - nx*sin(lt + ttf)))/abs(nx*cos(lt + ttf) + ny*sin(lt + ttf))^2]
"""

h = h.strip().replace("\n", ",\n")
hj = hj.strip().replace("\n", ",\n").replace("[", "").replace("]", "")

subs = {
    "abs": "f32::abs",
    "sign": "f32::signum",

    "lx": "lp.translation.x",
    "ly": "lp.translation.y",
    "lt": "lp.rotation.angle()",

    "nx": "wn[0]",
    "ny": "wn[1]",
    "wk": "wd",

    "cos": "cosf32",
    "sin": "sinf32",
}

for k, v in subs.items():
    h = h.replace(k, v)
    hj = hj.replace(k, v)

beta = [
    "txf",
    "tyf", 
    "ttf",
    "vxf",
    "vyf",
    "vtf",
    "bxf",
    "byf",
    "btf",
    "axf",
    "ayf",
    "atf",
]

for i in range(len(beta)):
    h = h.replace(beta[i], f"beta[{i}]")
    hj = hj.replace(beta[i], f"beta[{i}]")

print(re.sub(r"\s+", "", h))
print("\n")
print(re.sub(r"\s+", "", hj) + ",0.0"*6)
