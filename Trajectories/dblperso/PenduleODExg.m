function y=PenduleODExg(t,x)
global z g Pxm1

y(1)=x(2);
y(2)=(g/z)*(x(1)-Pxm1);
y=y';
end
