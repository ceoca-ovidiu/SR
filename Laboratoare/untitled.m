numx =[2];
denx = [0.1 1 ];
Hx = tf(numx,denx);
numy = [3];
deny = [0.75 1 ];
Hy = tf(numy,deny);
numc = [0.1 1];
denc =[0.01 1];
Hc= tf(numc,denc);
numc1 = [0.4 1];
denc1 =[0.01 1];
Hc1= tf(numc1,denc1);
%%
n1 =[10 9 5 7];
suma1 = sum(n1);
media1 = suma1/length(n1)

m1 = (10+9)/2;
m2 = (5+7)/2;
m = m1/2+m2/2