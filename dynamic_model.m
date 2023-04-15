clear variables
close all
clc
syms phim thetam dphim dthetam kf km w1 w2 w3 w4 d...
    m Ibxx Ibyy Ibzz px py pz dpx dpy dpz phib thetab psib wbx wby wbz...
    ddpx ddpy ddpz dwbx dwby dwbz g0 real
%declaring vectors for convenience
wm=[dphim dthetam]';
w=[w1 w2 w3 w4]'; %signed squared spinning rotor velocities
g=[0 0 -g0]';
Ib=diag([Ibxx Ibyy Ibzz]);
p=[px py pz]';
dp=[dpx dpy dpz]';
ddp=[ddpx ddpy ddpz]';
wb=[wbx wby wbz]'; %velocity of the UAV in B
dwb=[dwbx dwby dwbz]';
%pose of the UAV in the world frame
Rb=rpy_rotation('xyz',[phib,thetab,psib]);
% pose of the propeller pi in the frame B
Rpi=elem_rot_mat('y',thetam)*elem_rot_mat('x',phim);
Opi=sym(zeros(3,1,4));
%thrust and reaction moments of the propeller pi in its own frame
fpi=sym(zeros(3,1,4));
mpi=sym(zeros(3,1,4));
T=0; %Thrust on the CoM
tau=0; %torque on the CoM
for i=1:4
    Opi(:,:,i)=elem_rot_mat('z',(i-1)*sym(pi)/2)*[d 0 0]';
    fpi(:,:,i)=[0 0 kf*w(i)]';
    mpi(:,:,i)=(-1)^(i)*[0 0 km*w(i)]';
    T=simplify(T+Rpi*fpi(:,:,i));
    tau=simplify(tau+Rpi*mpi(:,:,i)+cross(Opi(:,:,i),Rpi*fpi(:,:,i)));
end
%% NE equations
c=simplify(-inv(Ib)*(cross(wb,Ib*wb)));
Jr=sym(zeros(6,6));
Jr(1:3,1:3)=1/m*Rb;
Jr(4:6,4:6)=inv(Ib);
Fm=simplify(jacobian(T,w)); taum=simplify(jacobian(tau,w)); %T,tau linear in w. They do not depend on wm, so their derivatives wrt wm is a 0 matrix
%taum of the paper:
% taum(1,:)=[-km*cos(thetam)*sin(phim),d*kf*cos(thetam)*cos(phim)+km*cos(thetam)*sin(phim), -km*cos(thetam)*sin(phim),-d*kf*cos(thetam)*cos(phim)+km*cos(thetam)*sin(phim)];
% taum(2,:)=[km*sin(phim) - d*kf*cos(phim)*cos(thetam), -km*sin(phim), km*sin(phim) + d*kf*cos(phim)*cos(thetam), -km*sin(phim)];
% taum(3,:)=[- d*kf*sin(phim) - km*cos(phim)*cos(thetam), -cos(phim)*(-km*cos(thetam) + d*kf*sin(thetam)), d*kf*sin(phim) - km*cos(phim)*cos(thetam), -cos(phim)*(-km*cos(thetam) - d*kf*sin(thetam))];
Jm=[Fm;taum];

%jacobian matrix and drift
J=simplify(Jr*[Jm zeros(6,2)]);
f=[g;c];
%% inputs for hovering, used for simulation
Rb_h=subs(Rb,[phib thetab psib]',zeros(3,1)); Fm_h=subs(Fm,[phim,thetam]',zeros(2,1));
w_h=pinv(Rb_h*Fm_h)*([0;0;9.81]);
%% after differentiation
c_dot=jacobian(c,wb)*dwb;
drift=[1/m*Rb*Smtrx(wb)*Fm*w;c_dot];
scol=[diff(Fm,phim)*w;diff(taum,phim)*w];
tcol=[diff(Fm,thetam)*w;diff(taum,thetam)*w];
jac=simplify(horzcat([Fm;taum],scol,tcol));
J_ast=simplify(Jr*jac);
determinant=simplify(det(J_ast));
%% simulation data
m=1; Ibxx=0.015; Ibyy=0.015; Ibzz=0.025; kf=1.6e-5; km=2.5e-7; d=0.3; g0=9.81;
Kp1=30*eye(3); Kw1=Kp1;
Kp2=600*eye(3); Kw2=Kp2;
Kp3=900*eye(3); Kw3=Kp3;
Fm=simplify(eval(Fm)); taum=simplify(eval(taum));
f=simplify(eval(f)); J=simplify(eval(J));
c_dot=simplify(eval(c_dot)); J_ast=simplify(eval(J_ast));
drift=simplify(eval(drift));
%% generating function for simulation
output=f+J*[w;wm];
matlabFunction(output,'File','quadrotor_dynamics.m','Vars',[phib,thetab,psib,wbx,wby,wbz,phim,thetam,w',wm'])