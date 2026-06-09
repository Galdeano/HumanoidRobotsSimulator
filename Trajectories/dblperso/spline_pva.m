function [ S ] = spline_pva( t,p1,v1,a1,p2,v2,a2 )
%SPLINE_PVA splines avec definition de du point, de la dérivée premiere et la dérivée seconde
%   splines avec definition de du point, de la dérivée premiere et la dérivée seconde

A=[[0,0,0,0,0,1];[0,0,0,0,1,0];[0,0,0,2,0,0];[1,1,1,1,1,1];[5,4,3,2,1,0];[20,12,6,2,0,0]];
r=[[1,0,0,0,0,0];[0,1,0,0,0,0];[0,0,1,0,0,0];[0,0,0,1,0,0];[0,0,0,0,1,0];[0,0,0,0,0,1]];

coeff=inv(A)*r;

t=t-t(1,1);
scale=t(end,end);
t=t./t(end,end);

coeff(6,1);

for i=1:6
    h(i,:)=coeff(1,i)*t(:).*t(:).*t(:).*t(:).*t(:)+coeff(2,i)*t(:).*t(:).*t(:).*t(:)+coeff(3,i)*t(:).*t(:).*t(:)+coeff(4,i)*t(:).*t(:)+coeff(5,i)*t(:)+coeff(6,i);
end

S(1,:)=p1*h(1,:)+v1*scale*h(2,:)+a1*scale*scale*h(3,:)+p2*h(4,:)+v2*scale*h(5,:)+a2*scale*scale*h(6,:);


end

