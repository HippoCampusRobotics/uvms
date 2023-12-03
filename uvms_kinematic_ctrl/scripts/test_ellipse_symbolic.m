clear all close all clc

    syms x z real;
syms a_x a_z real positive;
syms c_x c_z real;
syms s real positive;

% in cartesian coordinates : syms e_x e_z real;
% {
  expr_1 = (e_x - c_x) ^ 2 / (a_x ^ 2) + (e_z - c_z) ^ 2 / (a_z ^ 2) == 1;
  expr_2 = e_x + s * 2 * (e_x - c_x) / (a_x ^ 2) == x;
  expr_3 = e_z + s * 2 * (e_z - c_z) / (a_z ^ 2) == z;
  out = solve([ expr_1, expr_2, expr_3 ], [e_x e_z s]) %
}

expr_1 = (e_x - c_x) ^ 2 / (a_x ^ 2) + (e_z - c_z) ^ 2 / (a_z ^ 2) == 1;
expr_2 = a_z ^ 2 * (z - e_z) * 2 * (e_x - c_x) == a_x ^
         2 * (x - e_x) * 2 * (e_z - c_z);
out = solve([ expr_1, expr_2 ], [e_x e_z], 'ReturnConditions', true)

      % in polar coordinates theta = sym("theta", 'real');
expr_1 = c_x + (a_x + s * a_z) * cos(theta) - x == 0;
expr_2 = c_z + (a_z + s * a_x) * sin(theta) - z == 0;
out = solve([ expr_1, expr_2 ], [s theta], 'ReturnConditions', true)