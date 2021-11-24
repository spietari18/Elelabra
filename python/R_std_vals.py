#!/bin/env python

import R

R_vals = R.R(R.E12, {1, 2, 3})

expr_1 = lambda R_5: (43809536554452018*R_5)/612987373269875
expr_2 = lambda R_6: (394064967394918400*R_6)/5737747954578637 

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
        R_3 = expr_1(R_1)
        R_4 = expr_2(R_2)

        R_3p = closest(R_3)
        R_4p = closest(R_4)

        diff_1 = abs(R_3 - R_3p)
        diff_2 = abs(R_4 - R_4p)        

        res.append((R_1, R_2, R_3p, R_4p, diff_1, diff_2))

res.sort(key=lambda x: x[4] + x[5])

i = 0
while i < min(32, len(res)):
    print("{} {} = {} {} {} {}".format(*res[i]))
    i += 1
