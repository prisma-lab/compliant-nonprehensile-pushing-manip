zita = 0.7;
omega_n = 30;
syms s;
pol = (s^2 + 2*zita*omega_n*s + omega_n^2) * (s + 30);
pol_epx = expand(pol)
