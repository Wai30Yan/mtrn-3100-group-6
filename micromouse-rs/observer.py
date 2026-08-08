#!/usr/bin/python

import re

f = """
                                                                                                                            (axf*dt^2)/2 - vxf*dt + txf
                                                                                                                            (ayf*dt^2)/2 - vyf*dt + tyf
                                                                                                                            (atf*dt^2)/2 - vtf*dt + ttf
                                                                                                                                           vxf - axf*dt
                                                                                                                                           vyf - ayf*dt
                                                                                                                                           vtf - atf*dt
                                                                                                                                                    bxf
                                                                                                                                                    byf
                                                                                                                                                    btf
(dt*(2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2) + 2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) - bw*(vtf - (atf*dt)/2)))/rd
(dt*(2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2) + 2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) + bw*(vtf - (atf*dt)/2)))/rd
                                      cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) - sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2)
                                          bxf + atf*iy + ix*vtf^2 + axf*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf) + ayf*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)
                                          byf + atf*ix + iy*vtf^2 + ayf*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf) - axf*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)
                                                                                                                                              btf + vtf
"""

w = """
[1, 0,                                                                                                                               0,                                            -dt,                                              0,                                                                                                                                       0, 0, 0, 0,                                          dt^2/2,                                               0,                                                                                                                                                          0]
[0, 1,                                                                                                                               0,                                              0,                                            -dt,                                                                                                                                       0, 0, 0, 0,                                               0,                                          dt^2/2,                                                                                                                                                          0]
[0, 0,                                                                                                                               1,                                              0,                                              0,                                                                                                                                     -dt, 0, 0, 0,                                               0,                                               0,                                                                                                                                                     dt^2/2]
[0, 0,                                                                                                                               0,                                              1,                                              0,                                                                                                                                       0, 0, 0, 0,                                             -dt,                                               0,                                                                                                                                                          0]
[0, 0,                                                                                                                               0,                                              0,                                              1,                                                                                                                                       0, 0, 0, 0,                                               0,                                             -dt,                                                                                                                                                          0]
[0, 0,                                                                                                                               0,                                              0,                                              0,                                                                                                                                       1, 0, 0, 0,                                               0,                                               0,                                                                                                                                                        -dt]
[0, 0,                                                                                                                               0,                                              0,                                              0,                                                                                                                                       0, 1, 0, 0,                                               0,                                               0,                                                                                                                                                          0]
[0, 0,                                                                                                                               0,                                              0,                                              0,                                                                                                                                       0, 0, 1, 0,                                               0,                                               0,                                                                                                                                                          0]
[0, 0,                                                                                                                               0,                                              0,                                              0,                                                                                                                                       0, 0, 0, 1,                                               0,                                               0,                                                                                                                                                          0]
[0, 0, (dt*(2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) - 2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2)))/rd, (2*dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, (2*dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, -(dt*(bw + dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) - dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2)))/rd, 0, 0, 0, -(dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, -(dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd,  (dt*((bw*dt)/2 + (dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2))/2 - (dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2))/2))/rd]
[0, 0, (dt*(2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) - 2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2)))/rd, (2*dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, (2*dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd,  (dt*(bw - dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2) + dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2)))/rd, 0, 0, 0, -(dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, -(dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/rd, -(dt*((bw*dt)/2 - (dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2))/2 + (dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2))/2))/rd]
[0, 0,             - cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2) - sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2),          -sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf),           cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf),         (dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2))/2 + (dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2))/2, 0, 0, 0,     (dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2,    -(dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2,                      - (dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vxf - (axf*dt)/2))/4 - (dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf)*(vyf - (ayf*dt)/2))/4]
[0, 0,                                             ayf*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf) - axf*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf),                                              0,                                              0,                            2*ix*vtf - (ayf*dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2 + (axf*dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2, 1, 0, 0,            cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf),            sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf),                                                 iy + (ayf*dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/4 - (axf*dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/4]
[0, 0,                                           - axf*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf) - ayf*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf),                                              0,                                              0,                            2*iy*vtf + (axf*dt*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2 + (ayf*dt*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/2, 0, 1, 0,           -sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf),            cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf),                                                 ix - (axf*dt^2*cos((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/4 - (ayf*dt^2*sin((atf*dt^2)/4 - (vtf*dt)/2 + ttf))/4]
[0, 0,                                                                                                                               0,                                              0,                                              0,                                                                                                                                       1, 0, 0, 1,                                               0,                                               0,                                                                                                                                                          0]
"""

f = f.strip().replace("\n", ",\n")
w = w.strip().replace("\n", ",\n").replace("[", "").replace("]", "")

subs = {
    "dt^2" : "dt*dt",
    "vtf^2" : "vtf*vtf",

    "dt": "DT",
    "rd": "R",
    "bw": "B",
    "ix": "IX",
    "iy": "IY",

    "0" : "0.",
    "1" : "1.",
    "2" : "2.",
    "4" : "4.",
    
    "cos": "cosf32",
    "sin": "sinf32",
}

for k, v in subs.items():
    f = f.replace(k, v)
    w = w.replace(k, v)

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
    f = f.replace(beta[i], f"beta[{i}]")
    w = w.replace(beta[i], f"beta[{i}]")

print(re.sub(r"\s+", "", f))
print("\n")
print(re.sub(r"\s+", "", w))
