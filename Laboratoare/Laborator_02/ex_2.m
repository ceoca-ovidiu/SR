Mp=1.3;
ts=2;

xi=sqrt(log(Mp)^2/(log(Mp)^2+pi^2))
wn=3/ts/xi
B_sup=tf([wn^2],[1 2*xi*wn wn^2])
s1=-xi*wn+wn*sqrt(xi^2-1);
s2=-xi*wn-wn*sqrt(xi^2-1);
B_inf=tf(s1*s2,conv([1 -s1],[1 -s2]))