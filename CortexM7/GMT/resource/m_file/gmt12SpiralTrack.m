function traj = gmt12SpiralTrack(Tstart,ScanRange,LineSpace,velocity)

% --Given ----------------------------------
%  ---poseRPY--------
% Phome = [0;0;0;0;0;0];
%----pre-Test Bounds---------------------
% Xmin=-2.72; Xmax=2.34
% Ymin=-2.22; Ymax=2.22
% Zmin=-3.92; Zmax=4.09
% Phi_min=-5.145161000274793*pi/180; Phi_max=5.145161000274793*pi/180;
% Theta_min=-9.173054300044482*pi/180; Theta_max=9.092840208726166*pi/180
% Psi_min=-8.468316212033569*pi/180; Psi_max=8.468316212033569*pi/180;
%------------------------------------------

%%------Example Inputs--------------------------------
% clear;clc;close all;
% ScanRange = 1.0;  % ScanRange(max) = Zmax;
% LineSpace = 0.1;    % ScanRange*2/#(circles)
% velocity=1;   % 1<=v<=10;
%----------------------------------------------

format long

TPhome = eye(4);
%------------Parameters---------------------
xmin=-2.72; xmax=2.34;
ymin=-2.22; ymax=2.22;
zmin=-3.92; zmax=4.09;
rzmin= (-8)*pi/180; rzmax= (8)*pi/180;
rymin= (-8)*pi/180; rymax= (8)*pi/180;
rxmin= (-6)*pi/180; rxmax= (6)*pi/180;
% %------- Tstart----------------------------
Rstart = Tstart(1:3,1:3);
Pstart = Tstart(1:3,4);
[phi,theta,psi]=SO3ToRPY(Rstart);
ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);

Pdx = xmax-ax; Mdx= xmin-ax;
Pdy = ymax-ay; Mdy= ymin-ay;
Pdz = zmax-az; Mdz= zmin-az;
Pdrz = rzmax-phi; Mdrz = rzmin-phi;
Pdry = rymax-theta; Mdry = rymin-theta;
Pdrx = rxmax-psi; Mdrx = rxmin-psi;

dx= min(abs(Pdx),abs(Mdx)); 
dy= min(abs(Pdy),abs(Mdy)); 
dz= min(abs(Pdz),abs(Mdz)); 

ToolTranslx=Rstart(:,1);
ToolTransly=Rstart(:,2);
ToolTranslz=Rstart(:,3);

%------------------------------------
b=LineSpace/(2*pi);
if (ScanRange<= min(dy,dz))
    Scan = ScanRange;
else
    Scan = min(dy,dz);
end
v=velocity/100;
Tf=(2*pi)*Scan/LineSpace;
K=ceil(Tf/(0.0001))+1;
Trans=[];
traj=[];
for k=1:K 
    F= @(T) (b/2)*(log((T*k+sqrt(T^2*k^2+1))/(T*(k-1)+sqrt(T^2*(k-1)^2+1)))...
        +T*k*sqrt(T^2*k^2+1)-T*(k-1)*sqrt(T^2*(k-1)^2+1))-v*T;  
    x0 = 0.1;  % TolX = 1e-5; MaxIter = 50; %with initial guess 1.8
    T = newton(F,x0,1e-5,50); %1st order derivative
    %--------------------------------------      
    y=(b*(k-1)*T)*cos((k-1)*T);
    z=(b*(k-1)*T)*sin((k-1)*T);
    x= 0.0;
    Y=y*ToolTransly;
    Z=z*ToolTranslz;
    X=x*ToolTranslx;
    Trans(:,k)=[ax;ay;az]+X+Y+Z; 
    delta = Scan < min(abs(y),abs(z));    
    if (delta)
        M=k-1;
        break;
    end
    traj(:,k)=[Trans(:,k);phi;theta;psi];
    M=k;
end
traj(:,M)=[Trans(:,M);phi;theta;psi];     

plot3(Trans(1,1:M)',Trans(2,1:M)',Trans(3,1:M)','ro'); hold on;
%--------
plotH(TPhome);hold on;
plotH(Tstart);
xlabel('x');ylabel('y');zlabel('z');
grid on;
% count = count+1;
end