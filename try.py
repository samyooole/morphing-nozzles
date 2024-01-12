gamma = 1.5
P_c = 100000



from sympy import symbols, Eq, diff, solve
import sympy as sp

# Define the variables
x, y = symbols('x y')

# Define the equation (example: x^2 + y^2 = 1)
equation = Eq(x**2 + y**2, 1)

# Differentiate both sides implicitly with respect to x
derivative = diff(equation, x)

# Solve for dy/dx
derivative_dy_dx = solve(derivative, diff(y, x))[0]


h_c = 5
l = 10

import numpy as np

d_t, theta_c = symbols('d_t theta_c')
llambda = symbols('llambda')

Eq(d_t, h_c - 2 * llambda * l * sp.cos(theta_c))
