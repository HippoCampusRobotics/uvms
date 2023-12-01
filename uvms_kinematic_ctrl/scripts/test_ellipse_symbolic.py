import numpy as np
import sympy as sym
from sympy import solve

x, z = sym.symbols("x, z", real=True)
a_x, a_z = sym.symbols("a_x, a_z", real=True)
c_x, c_z = sym.symbols("c_x, c_z", real=True)
s = sym.symbols("s", real=True)

t = sym.symbols("t", real=True)
x_shifted, z_shifted = sym.symbols("x_shifted, z_shifted", real=True)


f = sym.Poly((t + a_x**2)**2 * (t + a_z**2)**2 - a_x**2 * x_shifted**2 * (t + a_z**2)**2 - a_z**2 * z_shifted**2 * (t + a_x**2)**2, t)
print(f)
print(f.coeffs)



"""
# in cartesian coordinates:
e_x, e_z = sym.symbols("e_x, e_z", real=True)
expr_1 = (e_x - c_x)**2 / a_x**2 + (e_z - c_z)**2 / a_z**2 - 1
expr_2 = a_z**2 * (z - e_z)*2*(e_x - c_x) - (a_x**2 * (x - e_x) * 2 * (e_z - c_z))
out = solve([expr_1, expr_2], [e_x, e_z], dict=True)
print(out)
#for key in out[0].keys():
    #out[0][key] = sym.simplify(out[0][key])
#print(out)
"""
"""
# in cartesian coordinates:
e_x, e_z = sym.symbols("e_x, e_z", real=True)
expr_1 = (e_x - c_x)**2 / a_x**2 + (e_z - c_z)**2 / a_z**2 - 1
expr_2 = e_x + s * 2 * (e_x - c_x) / a_x**2 - x
expr_3 = e_z + s * 2 * (e_z - c_z) / a_z**2 - z
out = solve([expr_1, expr_2, expr_3], [e_x, e_z, s], dict=True)
print(out)
for key in out[0].keys():
    out[0][key] = sym.simplify(out[0][key])
print(out)

# in polar coordinates
theta = sym.symbols("theta", real=True)
expr_1 = c_x + (a_x + s*a_z)*sym.cos(theta) - x
expr_2 = c_z + (a_z + s * a_x)*sym.sin(theta) -z
out = solve([expr_1, expr_2], [s, theta], dict=True)
for key in out[0].keys():
    out[0][key] = sym.simplify(out[0][key])
print(out)
"""
