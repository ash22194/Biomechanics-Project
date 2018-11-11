
l0 = 0.99; %[m]
ls = 0.5; %[m]

y0 = l0+0.2; %[m]
dy0 = 0; %[m/s]

m = 80; %[kg]
g = 9.81; %[m/s^2]


loptVAS = 0.1;
lslackVAS = 0.4;
FmaxVAS = 22000;
vmaxVAS = 12;

% **************** %
% MUSCLE MECHANICS %
% **************** %

% -------------------------------
% Shared Muscle Tendon Parameters
% -------------------------------

% excitation-contraction coupling
preA =  0.01; %[] preactivation
tau  =  0.01; %[s] delay time constant

% contractile element (CE) force-length relationship
w    =   0.4; %[lopt] width
c    =   0.05; %[]; remaining force at +/- width**

% CE force-velocity relationship
N    =   1.5; %[Fmax] eccentric force enhancement
K    =     5; %[] shape factor

% Series elastic element (SE) force-length relationship
eref =  0.04; %[lslack] tendon reference strain


lVAS0 = loptVAS+lslackVAS;
r0 = 0.05;%**

thetak0 = 110*pi/180;



% --------------
% Neural Control
% --------------

G_F = 1/FmaxVAS;
% G_v = -1/(vmaxVAS*loptVAS);
G_v = 0;
% G_l = 1/loptVAS;
G_l = 0;
a = 1;
tauE = 0.01;
tau_E = 12;
bE = 0;
thetaE = 0;
tauW = 1;
theta_u = zeros(3,1);
