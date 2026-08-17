#!/usr/bin/python

import re

f = """
tx + (rd*cos(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/2
ty + (rd*sin(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/2
                                 tt - (rd*(dwl - dwr))/bw
"""

fj = """
[1, 0, -(rd*sin(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/2]
[0, 1,  (rd*cos(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/2]
[0, 0,                                                     1]
"""

fju = """
[(rd*cos(tt - (rd*(dwl - dwr))/(2*bw)))/2 + (rd^2*sin(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/(4*bw), (rd*cos(tt - (rd*(dwl - dwr))/(2*bw)))/2 - (rd^2*sin(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/(4*bw)]
[(rd*sin(tt - (rd*(dwl - dwr))/(2*bw)))/2 - (rd^2*cos(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/(4*bw), (rd*sin(tt - (rd*(dwl - dwr))/(2*bw)))/2 + (rd^2*cos(tt - (rd*(dwl - dwr))/(2*bw))*(dwl + dwr))/(4*bw)]
[                                                                                                -rd/bw,                                                                                                  rd/bw]
"""

f = f.strip().replace("\n", ",\n")
fj = fj.strip().replace("\n", ",\n").replace("[", "").replace("]", "")
fju = fju.strip().replace("\n", ",\n").replace("[", "").replace("]", "")

subs = {
    "rd^2" : "R*R",

    "rd": "R",
    "bw": "B",

    "0" : "0.",
    "1" : "1.",
    "2" : "2.",
    "4" : "4.",
    "16" : "16.",
    
    "cos": "cosf32",
    "sin": "sinf32",

    "tx": "self.state[0]",
    "ty": "self.state[1]",
    "tt": "self.state[2]",
}

for k, v in subs.items():
    f = f.replace(k, v)
    fj = fj.replace(k, v)
    fju = fju.replace(k, v)

print(re.sub(r"\s+", "", f))
print("\n")
print(re.sub(r"\s+", "", fj))
print("\n")
print(re.sub(r"\s+", "", fju))
