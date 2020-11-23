mu = 45;
x = 0;
y = 0;

rotorZB = rotz(mu) * rotz(90);
rotorXp = rotx(x);
rotorYpp = roty(y);

R_BR = rotorZB * rotorXp * rotorYpp;

