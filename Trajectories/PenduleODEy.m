function y=PenduleODEy(t,x)
global Zc g Pys

y(1)=x(2);
y(2)=(g/Zc)*(x(1)-Pys);
y=y';
end