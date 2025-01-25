X=[];
Y=[];
for w=[0.1 0.5 1 5 10]; % frecventa de interes

 x=(3*(1-17*(w.^2)))/((1-17*(w.^2))^2+(8*w.^1-10*w.^3)^2); % partea reala
 X=[X x]
 y=(3*(-8*w.^1+10*w.^3))/((1-17*(w.^2))^2+(8*w.^1-10*w.^3)^2);  % partea imaginara
 Y=[Y y]
end
% M=abs(sqrt((x^2+y^2)/[(1+x^2)^2+y^2])); % modulul sistemului in bucla inchisa la pulsatia w

%Verificare
num=3;
den=[10 17 8 1];
% bucla directa
hd=tf(num,den); 
% bucla inchisa
hcl=feedback(hd,1);

w=[0.1 0.5 1 5 10];

figure(1)
nyquist(hd);
grid;
[mag,phase] = bode(hcl,w)
hold on
nyquist(hd,w)

figure(2)
nichols(hd,w);
grid;
[magN,phaseN] = nichols(hcl,w)


