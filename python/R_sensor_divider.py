#!/bin/env python

import R

R_smin = 82
R_smax = 134

# expression for divider voltage
U = lambda R_1, R_2, R_s: 5*(R_2 + R_s)/(R_1 + R_2 + R_s)

# expression for sensor power
P = lambda R_1, R_2, R_s: ((5/(R_1 + R_2 + R_s))**2)*R_s

# how many results to display
n = 32

R_vals = R.R(R.E12, {0, 1, 2, 3})

V = []
for R_1 in R_vals:
    for R_2 in R_vals:
        p = P(R_1, R_2, R_smin)
        # max power = 100uW
        if p > 0.0001:
            continue

        u_min = U(R_1, R_2, R_smin)
        u_max = U(R_1, R_2, R_smax)
        # voltage can't be close to the edges
        # of the supply voltage for amp to work
        if not ((0.7 < u_min < 3.6) and (0.7 < u_max < 3.6)):
            continue

        V.append((R_1, R_2, p, u_min, u_max, abs(u_min - u_max)))

# sort by the largest change in voltage
V.sort(key=lambda x: x[5], reverse=True)

i = 0
while i < min(n, len(V)):
    R_1, R_2, p, u_min, u_max, delta = V[i]
    
    print("{}Ohm, {}Ohm = {}W {}V {}V {}V".format( \
        R.prefix(R_1), R.prefix(R_2), R.prefix(p), u_min, u_max, delta))

    i += 1
