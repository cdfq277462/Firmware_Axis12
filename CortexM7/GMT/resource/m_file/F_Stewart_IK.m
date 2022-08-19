function  L = F_Stewart_IK(P,pp,bb)

%---Example--------------------------------------------------
% %----pre-Test Bounds for given pose--------------------
% abs(Xm)=19;
% abs(Ym)=19;
% abs(Zm)=9;
% abs(Phi_m)=13*pi/180;
% abs(Theta_m)=6*pi/180;
% abs(Psi_m)=6*pi/180;
%------------------------------------------
%-------------------------------------------------------------

format long
% global bb pp
% Position and orientation vector
bd = P(1:3);
phi = P(4);
theta = P(5);
psi = P(6);

% transformation matrix %
%---------R(phi)-P(theta)-Y(psi)------
r11=cos(theta)*cos(phi);
r12=cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
r13=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
r21=sin(phi)*cos(theta);
r22=cos(phi)*cos(psi)+sin(theta)*sin(phi)*sin(psi);
r23=-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
r31=-sin(theta);
r32=cos(theta)*sin(psi);
r33=cos(theta)*cos(psi);

%--------Z(alpha)-Y(beta)-Z(gamma)-------------------------
% r11=cos(alpha)*cos(beta)*cos(gamma)-sin(alpha)*sin(gamma);
% r21=cos(alpha)*sin(gamma)+cos(beta)*cos(gamma)*sin(alpha);
% r31=-cos(gamma)*sin(beta);
% r12=-cos(gamma)*sin(alpha)-cos(alpha)*cos(beta)*sin(gamma);
% r22=cos(alpha)*cos(gamma)-cos(beta)*sin(alpha)*sin(gamma);
% r32=sin(beta)*sin(gamma);
% r13=cos(alpha)*sin(beta);
% r23=sin(alpha)*sin(beta);
% r33=cos(beta);

%----------------------------------------------------------
R=[r11 r12 r13; r21 r22 r23; r31 r32 r33];

%bbi fixed platform global coordinates
bb1 = bb(:,1);
bb2 = bb(:,2);
bb3 = bb(:,3);
bb4 = bb(:,4);
bb5 = bb(:,5);
bb6 = bb(:,6);

% ppi moving platform coordinates
pp1 = pp(:,1);
pp2 = pp(:,2);
pp3 = pp(:,3);
pp4 = pp(:,4);
pp5 = pp(:,5);
pp6 = pp(:,6);

%inverse kinematics for lengths of the actuators
Bx1= bd-bb1;
Bx2= bd-bb2;
Bx3= bd-bb3;
Bx4= bd-bb4;
Bx5= bd-bb5;
Bx6= bd-bb6;
%Bpi
BP1= R*pp1;
BP2= R*pp2;
BP3= R*pp3;
BP4= R*pp4;
BP5= R*pp5;
BP6= R*pp6;

%lenght of the actuators
l1=sqrt((abs(Bx1(1)+BP1(1)))^2+(abs(Bx1(2)+BP1(2)))^2+(abs(Bx1(3)+BP1(3)))^2);
l2=sqrt((abs(Bx2(1)+BP2(1)))^2+(abs(Bx2(2)+BP2(2)))^2+(abs(Bx2(3)+BP2(3)))^2);
l3=sqrt((abs(Bx3(1)+BP3(1)))^2+(abs(Bx3(2)+BP3(2)))^2+(abs(Bx3(3)+BP3(3)))^2);
l4=sqrt((abs(Bx4(1)+BP4(1)))^2+(abs(Bx4(2)+BP4(2)))^2+(abs(Bx4(3)+BP4(3)))^2);
l5=sqrt((abs(Bx5(1)+BP5(1)))^2+(abs(Bx5(2)+BP5(2)))^2+(abs(Bx5(3)+BP5(3)))^2);
l6=sqrt((abs(Bx6(1)+BP6(1)))^2+(abs(Bx6(2)+BP6(2)))^2+(abs(Bx6(3)+BP6(3)))^2);
L=[l1;l2;l3;l4;l5;l6];

end
