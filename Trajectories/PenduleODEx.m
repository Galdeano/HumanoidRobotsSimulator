function y=PenduleODEx(t,x)
global Zc g Pxs

y(1)=x(2);
y(2)=(g/Zc)*(x(1)-Pxs);
y=y';
end