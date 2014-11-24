%% This code is licensed under Creative Commons Attribution Share-Alike 3.0
% for the details about this license please go to
% http://creativecommons.org/licenses/by-sa/3.0/

%% Author: Javier V. Gómez  -  www.javiervgomez.com jvgomez _at_ ing.uc3m.es
% Date:  06/02/2013

function A=sum_in(A,B,x)
% sums the small matrix B to the big matrix A in position [x(1),x(2)] (center of the square matrix B).
x=round(x);
a=size(A);
b=size(B);
r2=floor(b(1)/2);
for i=max(1,x(1)-r2):min(a(1),x(1)+r2)
    for j=max(1,x(2)-r2):min(a(2),x(2)+r2)
        A(i,j)=A(i,j)+B(i-x(1)+r2+1,j-x(2)+r2+1);
    end
end
