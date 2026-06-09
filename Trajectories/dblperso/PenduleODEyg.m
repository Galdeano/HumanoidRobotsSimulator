function y=PenduleODEyg(t,x)
global z g Pym1

y(1)=x(2);
y(2)=(g/z)*(x(1)-Pym1);
y=y';
end
