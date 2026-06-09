function y=PenduleODEx(t,x)
global z g Pxm

y(1)=x(2);
y(2)=(g/z)*(x(1)-Pxm);
y=y';
end
