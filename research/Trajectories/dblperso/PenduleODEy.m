function y=PenduleODEy(t,x)
global z g Pym

y(1)=x(2);
y(2)=(g/z)*(x(1)-Pym);
y=y';
end
