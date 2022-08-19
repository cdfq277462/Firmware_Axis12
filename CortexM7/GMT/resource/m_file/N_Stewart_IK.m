function  [Ls,alpha] = N_Stewart_IK(P,pp,bb)

%---Example--------------------------------------------------
% P = [-5.3156484;  -0.26133255; 259.96659;...
%       2.3451623*pi/180;-4.8837413*pi/180;-3.1559683*pi/180]
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
Bp1= R*pp1;
Bp2= R*pp2;
Bp3= R*pp3;
Bp4= R*pp4;
Bp5= R*pp5;
Bp6= R*pp6;

% L-vector
Lv1 = Bx1+Bp1;
Lv2 = Bx2+Bp2;
Lv3 = Bx3+Bp3;
Lv4 = Bx4+Bp4;
Lv5 = Bx5+Bp5;
Lv6 = Bx6+Bp6;

% % L-z component
lz1 = Lv1(3);
lz2 = Lv2(3);
lz3 = Lv3(3);
lz4 = Lv4(3);
lz5 = Lv5(3);
lz6 = Lv6(3);

alpha = [lz1;lz2;lz3;lz4;lz5;lz6];

%lenght of the L-vector
L1=sqrt((abs(Bx1(1)+Bp1(1)))^2+(abs(Bx1(2)+Bp1(2)))^2+(abs(Bx1(3)+Bp1(3)))^2);
L2=sqrt((abs(Bx2(1)+Bp2(1)))^2+(abs(Bx2(2)+Bp2(2)))^2+(abs(Bx2(3)+Bp2(3)))^2);
L3=sqrt((abs(Bx3(1)+Bp3(1)))^2+(abs(Bx3(2)+Bp3(2)))^2+(abs(Bx3(3)+Bp3(3)))^2);
L4=sqrt((abs(Bx4(1)+Bp4(1)))^2+(abs(Bx4(2)+Bp4(2)))^2+(abs(Bx4(3)+Bp4(3)))^2);
L5=sqrt((abs(Bx5(1)+Bp5(1)))^2+(abs(Bx5(2)+Bp5(2)))^2+(abs(Bx5(3)+Bp5(3)))^2);
L6=sqrt((abs(Bx6(1)+Bp6(1)))^2+(abs(Bx6(2)+Bp6(2)))^2+(abs(Bx6(3)+Bp6(3)))^2);

Ls = [L1;L2;L3;L4;L5;L6];

% % length of the g-actuators
% if sum((alpha.^2 + h0^2 - Ls.^2)<0)>0 
%     error('Pose cannot be reached')
% end
% g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
% dg = g-g0;
end

% l1 = lz1 - sqrt(lz1^2 + h0^2 -L1^2);
% l2 = lz2 - sqrt(lz2^2 + h0^2 -L2^2);
% l3 = lz3 - sqrt(lz3^2 + h0^2 -L3^2);
% l4 = lz4 - sqrt(lz4^2 + h0^2 -L4^2);
% l5 = lz5 - sqrt(lz5^2 + h0^2 -L5^2);
% l6 = lz6 - sqrt(lz6^2 + h0^2 -L6^2);

% l = [l1;l2;l3;l4;l5;l6];


