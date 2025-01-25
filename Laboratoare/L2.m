%syms a0 a1 a2
a3= 1;
a2_range =[2, 3];
a1_range =[5,8];
a0_range =[3,6];

a2 = ureal('a2',2.5,'Range', a2_range)
a1 = ureal('a1',6.5,'Range', a1_range)
a0 = ureal('a0',(3+6)/2,'Range', a0_range)

syms s
k1 = [max(a3) max(a2_range) min(a1_range) min(a0_range) ]
k2 = [min(a3) max(a2_range) max(a1_range) min(a0_range) ]
k4 = [min(a3) min(a2_range) max(a1_range) max(a0_range) ]
k3 = [max(a3) min(a2_range) min(a1_range) max(a0_range) ]
%p = [1 a2 a1 a0]
roots(k1)
roots(k2)
roots(k3)
roots(k4)
%%
%syms a0 a1 a2
a3= 1;
a2_range =[2, 3];
a1_range =[5,8];
a0_range =[10,20];

a2 = ureal('a2',2.5,'Range', a2_range)
a1 = ureal('a1',6.5,'Range', a1_range)
a0 = ureal('a0',(3+6)/2,'Range', a0_range)

syms s
k1 = [max(a3) max(a2_range) min(a1_range) min(a0_range) ]
k2 = [min(a3) max(a2_range) max(a1_range) min(a0_range) ]
k4 = [min(a3) min(a2_range) max(a1_range) max(a0_range) ]
k3 = [max(a3) min(a2_range) min(a1_range) max(a0_range) ]
%p = [1 a2 a1 a0]
roots(k1)
roots(k2)
roots(k3)
roots(k4)