#!/bin/env python

import R

R_vals = R.R(R.E12, {1, 2, 3})

expr = lambda R_1, R_2, R_3: R_1/((7881299347898368*R_2)/(224181193982707*R_3) - 1/2)

res = []

def closest(val):
    current = None

    for R_val in R_vals:
        if current == None:
            current = R_val
        else:
            if abs(R_val - val) < abs(current - val):
                current = R_val

    return current

for R_1 in R_vals:
    for R_2 in R_vals:
        for R_3 in R_vals:
            R_g = expr(R_1, R_2, R_3)
            
            R_gp = closest(R_g)

            diff = abs(R_g - R_gp)

            res.append((R_1, R_2, R_3, R_gp, diff))

res.sort(key=lambda x: x[4])

i = 0
while i < min(32, len(res)):
    R_1, R_2, R_3, R_g, diff = res[i]
    print("{} {} {} = {} {}".format(
        R.prefix(R_1), R.prefix(R_2), R.prefix(R_3), R.prefix(R_g), diff))
    i += 1
