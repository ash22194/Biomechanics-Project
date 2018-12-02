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
r0 = 0.05;

thetak0 = 110*pi/180;



% --------------
% Neural Control
% --------------

G_F = 1/(N*FmaxVAS);
G_v = -1/(vmaxVAS*loptVAS);
%G_v = 0;
G_l = 1/loptVAS;
%G_l = 0;
%a = 1;
tauE = 0.01;
tau_E = 12;
bE = 2.5;
thetaE = 0;
tauW = 50;
theta_u = zeros(3,1);

% w_F0=1;
% w_l0=1;
% w_v0=1;

% for i=1:4
%     sim('neural_hopper_trial_2')
%     w_F0=w_F;
%     w_l0=w_l;
%     w_v0=w_v;
% end

w_l_0 = 1;
w_v_0 = 1;
w_F_0 = 1;

numTrials = 15;
heightsReached = zeros(numTrials,1);
weights = zeros(numTrials+1,3); % l, v, F
weights(1,:) = [w_l_0, w_v_0, w_F_0];
for i=1:1:numTrials
    sim('neural_hopper_diff_hebb');
    w_l_0 = w_f(1);
    w_v_0 = w_f(2);
    w_F_0 = w_f(3);
    heightsReached(i,1) = y_f - l0;
    weights(i+1,:) = [w_l_0, w_v_0, w_F_0];
end

plot(1:1:numTrials,heightsReached);
title('Jump height vs Trials');
xlabel('Trial');
ylabel('Jump height');

figure;
plot(1:1:numTrials+1,weights(:,1),'DisplayName','Length');hold on;
plot(1:1:numTrials+1,weights(:,2),'DisplayName','Velocity');
plot(1:1:numTrials+1,weights(:,3),'DisplayName','Force');
title('Synaptic weights vs Trials');
xlabel('Trial');
ylabel('w');
legend();
hold off;



