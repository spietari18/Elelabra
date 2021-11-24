#!/bin/env python

import R

# expression for divider voltage
U = lambda R_1, R_2: 5*(R_2)/(R_1 + R_2)

U_t = 1.3989524225229157

# how many results to display
n = 32

R_vals = R.R(R.E12)

V = []
for R_1 in R_vals:
    for R_2 in R_vals:
        V.append((R_1, R_2, U(R_1, R_2)))

# sort by the largest change in voltage
V.sort(key=lambda x: abs(x[2] - U_t))

i = 0
while i < min(n, len(V)):
    R_1, R_2, u = V[i]
    
    print("{}Ohm, {}Ohm = {}V {}V".format( \
        R.prefix(R_1), R.prefix(R_2), u, R.prefix(U_t - u, 6)))

    i += 1
