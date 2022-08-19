function  S=F_Stewart_FK(Pi)
%-------------------------------------
% Stewart 物理最低點 Length 162.5mm
% Stewart 物理最高點 Length 182.5mm
% 
% Stewart Low(Zero) Point Length 163.5mm
% Stewart High Point Length 181.5mm
% Stewart Home Point Length 172.5mm. In this case, the pose of the Home
% point is
% Phome =[0;0;155.0851057814131;0;0;0];
%----------------------------------------
format long
% options = optimoptions('fsolve','Display','iter');
if nargin < 1
   ini_pose =  [0; 0; 155.0851057814131; 0; 0; 0];
else
    ini_pose = Pi;
end
x = fsolve(@myfun, ini_pose); % function calls3
S = x;
end
 
function f=myfun(x)
syms x1 x2 x3 a b g
global pp bb L
X=[x1;x2;x3];

% Rotation matrix--RPY--
Rza=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
Ryb=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rxg=[1 0 0;0 cos(g) -sin(g);0 sin(g) cos(g)];
R=Rza*Ryb*Rxg;

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
% li Lengths of the actuators
% L = F_Stewart_IK(P,pp,bb);

%--Length of actuators----------
l1=L(1);
l2=L(2);
l3=L(3);
l4=L(4);
l5=L(5);
l6=L(6);

% funtion of the lengths of the actuators
f1=(norm(X+R*pp1-bb1))^2-l1^2;
f2=(norm(X+R*pp2-bb2))^2-l2^2;
f3=(norm(X+R*pp3-bb3))^2-l3^2;
f4=(norm(X+R*pp4-bb4))^2-l4^2;
f5=(norm(X+R*pp5-bb5))^2-l5^2;
f6=(norm(X+R*pp6-bb6))^2-l6^2;
f=[f1;f2;f3;f4;f5;f6];
x1=x(1);
x2=x(2);
x3=x(3);
a=x(4);
b=x(5);
g=x(6);
f=eval(f);
end
