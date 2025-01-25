clc; close all; clear all;

% Physical parameters
mb = 300;    % Car body mass [kg]
mw = 60;     % Wheel mass [kg]
bs = 1000;   % Suspension damping [N/m/s]
ks = 16000 ; % Suspension stiffness [N/m]
kt = 190000; % Tire stiffness [N/m]

% State matrices
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw];
B = [ 0 0; 0 1e3/mb ; 0 0 ; [kt -1e3]/mw];
C = [1 0 0 0; 1 0 -1 0; A(2,:)];
D = [0 0; 0 0; B(2,:)];

qcar = ss(A,B,C,D);
qcar.StateName = {'body travel (m)';'body vel (m/s)';...
          'wheel travel (m)';'wheel vel (m/s)'};
qcar.InputName = {'r';'fs'};
qcar.OutputName = {'xb';'sd';'ab'};

ActNom = tf(1,[1/60 1]);
ActNom.InputName = 'u';
ActNom.OutputName = 'fs';

Wroad = ss(0.07);  Wroad.u = 'd1';   Wroad.y = 'r';
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1';

HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);
ComfortTarget = 0.4 * tf([1/0.45 1],[1/150 1]);

beta = reshape([0.01 0.5 0.99],[1 1 3]);
Wsd = beta / HandlingTarget;
Wsd.u = 'sd';  Wsd.y = 'e3';
Wab = (1-beta) / ComfortTarget;
Wab.u = 'ab';  Wab.y = 'e2';

sdmeas  = sumblk('y1 = sd');
abmeas = sumblk('y2 = ab');
ICinputs = {'d1';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};
qcaric = connect(qcar(2:3,:),ActNom,Wroad,Wact,Wab,Wsd,sdmeas,abmeas,ICinputs,ICoutputs);

ncont = 1; % one control signal, u
nmeas = 2; % two measurement signals, sd and ab
K = ss(zeros(ncont,nmeas,3));
gamma = zeros(3,1);
for i=1:3
   [K(:,:,i),~,gamma(i)] = hinfsyn(qcaric(:,:,i),nmeas,ncont);
end

K.u = {'sd','ab'};  K.y = 'u';
CL = connect(qcar,ActNom,K,'r',{'xb';'sd';'ab'});

% Road disturbance
t = 0:0.0025:4;
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% Simulate
p1 = lsim(qcar(:,1),roaddist,t);
y1 = lsim(CL(1:3,1,1),roaddist,t);
y2 = lsim(CL(1:3,1,2),roaddist,t);
y3 = lsim(CL(1:3,1,3),roaddist,t);

% Bucla deschisa
figure('Position', [550, 550, 800, 500]);
subplot(311)
plot(t,p1(:,1),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcarea șasiului'); xlabel('Timp (s)'); ylabel('Mișcarea șasiului (m)');
legend('Mișcarea șasiului','Perturbația');
subplot(312)
plot(t,p1(:,3),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Accelerația șasiului'); xlabel('Timp (s)'); ylabel('Accelerația șasiului (m/s^2)');
legend('Accelerația șasiului','Perturbația');
subplot(313)
plot(t,p1(:,2),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcare suspensiei'); xlabel('Timp (s)'); ylabel('Mișcarea suspensiei (m)');
legend('Mișcarea suspensiei','Perturbația');

% Mod Comfort
figure('Position', [550, 550, 800, 500]);
subplot(311)
plot(t,y1(:,1),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcarea șasiului'); xlabel('Timp (s)'); ylabel('Mișcarea șasiului (m)');
legend('Mișcarea șasiului','Perturbația');
subplot(312)
plot(t,y1(:,3),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Accelerația șasiului'); xlabel('Timp (s)'); ylabel('Accelerația șasiului (m/s^2)');
legend('Accelerația șasiului','Perturbația');
subplot(313)
plot(t,y1(:,2),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcare suspensiei'); xlabel('Timp (s)'); ylabel('Mișcarea suspensiei (m)');
legend('Mișcarea suspensiei','Perturbația');

% Mod Standard
figure('Position', [550, 550, 800, 500]);
subplot(311)
plot(t,y2(:,1),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcarea șasiului'); xlabel('Timp (s)'); ylabel('Mișcarea șasiului (m)');
legend('Mișcarea șasiului','Perturbația');
subplot(312)
plot(t,y2(:,3),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Accelerația șasiului'); xlabel('Timp (s)'); ylabel('Accelerația șasiului (m/s^2)');
legend('Accelerația șasiului','Perturbația');
subplot(313)
plot(t,y2(:,2),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcare suspensiei'); xlabel('Timp (s)'); ylabel('Mișcarea suspensiei (m)');
legend('Mișcarea suspensiei','Perturbația');

% Mod Sport
figure('Position', [550, 550, 800, 500]);
subplot(311)
plot(t,y3(:,1),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcarea șasiului'); xlabel('Timp (s)'); ylabel('Mișcarea șasiului (m)');
legend('Mișcarea șasiului','Perturbația');
subplot(312)
plot(t,y3(:,3),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Accelerația șasiului'); xlabel('Timp (s)'); ylabel('Accelerația șasiului (m/s^2)');
legend('Accelerația șasiului','Perturbația');
subplot(313)
plot(t,y3(:,2),'Color',"#0072BD",'LineWidth',1.5); grid; hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcare suspensiei'); xlabel('Timp (s)'); ylabel('Mișcarea suspensiei (m)');
legend('Mișcarea suspensiei','Perturbația');
%%
% Plot results
figure('Position', [550, 550, 900, 600]);
plot(t,p1(:,1),'Color',"#0072BD",'LineWidth',1.5);hold on;
plot(t,y1(:,1),'Color','#D95319','LineWidth',1.5);hold on;
plot(t,y2(:,1),'Color','#7E2F8E','LineWidth',1.5);hold on;
plot(t,y3(:,1),'Color','#A2142F','LineWidth',1.5);hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5)
title('Mișcarea șasiului'), ylabel('Mișcarea șasiului (m)'); xlabel('Timp (s)'); grid;
legend('Buclă deschisă','Mod Comfort','Mod Standard','Mod Sport','Perturbație drum')

figure('Position', [550, 550, 900, 600]);
plot(t,p1(:,3),'Color',"#0072BD",'LineWidth',1.5); hold on;
plot(t,y1(:,3),'Color','#D95319','LineWidth',1.5); hold on;
plot(t,y2(:,3),'Color','#7E2F8E','LineWidth',1.5); hold on;
plot(t,y3(:,3),'Color','#A2142F','LineWidth',1.5); hold on;
plot(t,roaddist,'Color','#77AC30','LineWidth',1.5);
title('Accelerația șasiului'), ylabel('Accelerația șasiului (m/s^2)'); xlabel('Timp (s)'); grid;
legend('Buclă deschisă','Mod Comfort','Mod Standard','Mod Sport','Perturbație drum')

figure('Position', [550, 550, 900, 600]);
plot(t,p1(:,2),'Color',"#0072BD",'LineWidth',1.5); hold on;
plot(t,y1(:,2),'Color',"#D95319",'LineWidth',1.5); hold on;
plot(t,y2(:,2),'Color',"#7E2F8E",'LineWidth',1.5); hold on;
plot(t,y3(:,2),'Color',"#A2142F",'LineWidth',1.5); hold on;
plot(t,roaddist,'Color',"#77AC30",'LineWidth',1.5);
title('Mișcare suspensiei'), xlabel('Timp (s)'), ylabel('Mișcarea suspensiei (m)'); grid;
legend('Buclă deschisă','Mod Comfort','Mod Standard','Mod Sport','Perturbație drum')

% figure('Position', [550, 550, 900, 600]);
% plot(t,zeros(size(t)),'Color',"#0072BD",'LineWidth',1.5); hold on;
% plot(t,y1(:,4),'Color',"#D95319",'LineWidth',1.5); hold on;
% plot(t,y2(:,4),'Color',"#7E2F8E",'LineWidth',1.5); hold on;
% plot(t,y3(:,4),'Color',"#A2142F",'LineWidth',1.5); hold on
% plot(t,roaddist,'Color',"#77AC30",'LineWidth',1.5);
% title('Forța actuatorului'), xlabel('Timp (s)'), ylabel('Forța actuatorului (kN)'); grid;
% legend('Buclă deschisă','Mod Comfort','Mod Standard','Mod Sport','Perturbație drum')

Wunc = makeweight(0.80,15,3);
unc = ultidyn('unc',[1 1],'SampleStateDim',5);
ActUnc = ActNom*(1 + Wunc*unc);
ActUnc.InputName = 'u';
ActUnc.OutputName = 'fs';

qcaric = connect(qcar(2:3,:),ActUnc,Wroad,Wact,Wab,Wsd,sdmeas,abmeas,ICinputs,ICoutputs);

rng('default'), nsamp = 100;  clf

t = 0:0.0025:2;
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% body position
% suspension travel
% body acceleration
%% ROBUST STANDARD
figure('Position', [550, 550, 800, 500]);
CLU = connect(qcar,ActUnc,K(:,:,2),'r',{'xb','sd','ab'});
lsim(usample(CLU,nsamp),'b',roaddist,t); grid;
title('Poziția șasiului');

[Krob,rpMU] = musyn(qcaric(:,:,2),nmeas,ncont);

figure('Position', [550, 550, 800, 500]);
Krob.u = {'sd','ab'};
Krob.y = 'u';
CLUR = connect(qcar,ActUnc,Krob,'r',{'xb','sd','ab'});
lsim(usample(CLUR,nsamp),'b',roaddist,t); grid;
% title('Mod Standard Robust')

figure('Position', [550, 550, 800, 500]);
lsim(usample(CLU,nsamp),'b',roaddist,t); grid; hold on;
lsim(usample(CLUR,nsamp),'r',roaddist,t); grid;
% title('Comparatie robust si normal (Mod Standard)')
%% ROBUST COMFORT
figure('Position', [550, 550, 800, 500]);
CLU = connect(qcar,ActUnc,K(:,:,1),'r',{'xb','sd','ab'});
lsim(usample(CLU,nsamp),'b',roaddist,t); grid;
title('Mod Comfort')

[Krob,rpMU] = musyn(qcaric(:,:,1),nmeas,ncont);

figure('Position', [550, 550, 800, 500]);
Krob.u = {'sd','ab'};
Krob.y = 'u';
CLUR = connect(qcar,ActUnc,Krob,'r',{'xb','sd','ab'});
lsim(usample(CLUR,nsamp),'b',roaddist,t); grid;
title('Mod Comfort Robust')

figure('Position', [550, 550, 800, 500]);
lsim(usample(CLU,nsamp),'b',roaddist,t); grid; hold on;
lsim(usample(CLUR,nsamp),'r',roaddist,t); grid;
title('Comparatie robust si normal (Mod Comfort)')

%% ROBUST SPORT
figure('Position', [550, 550, 900, 600]);
CLU = connect(qcar,ActUnc,K(:,:,3),'r',{'xb','sd','ab'});
lsim(usample(CLU,nsamp),'b',roaddist,t); grid;
title('Nominal "sport" design')
legend('Perturbed','Nominal','location','SouthEast')

[Krob,rpMU] = musyn(qcaric(:,:,3),nmeas,ncont);

figure('Position', [550, 550, 900, 600]);
Krob.u = {'sd','ab'};
Krob.y = 'u';
CLUR = connect(qcar,ActUnc,Krob,'r',{'xb','sd','ab'});
lsim(usample(CLUR,nsamp),'b',roaddist,t); grid;
title('Mod Sport Robust')

figure('Position', [550, 550, 900, 600]);
lsim(usample(CLU,nsamp),'b',roaddist,t); grid; hold on;
lsim(usample(CLUR,nsamp),'r',roaddist,t); grid;
title('Comparatie robust si normal (Mod Sport)')