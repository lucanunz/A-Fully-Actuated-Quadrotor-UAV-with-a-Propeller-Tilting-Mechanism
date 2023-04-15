function A=matrix_A(u)
phib=u(1); thetab=u(2);

A=[1, sin(phib)*tan(thetab), cos(phib)*tan(thetab);
   0,             cos(phib), -sin(phib);
   0, sin(phib)/cos(thetab), cos(phib)/cos(thetab)];
end