Pstart=[1;1;66;0.1;0.1;0.1];
Pend=[-1;-1;67;-0.1;-0.1;-0.1];
TimeScaling = '3'; 
velocity = 2*pi/3; 
Ts = 0.01;   % sample time

traj = N_moveL_PP(Pstart, Pend, TimeScaling, velocity, Ts);