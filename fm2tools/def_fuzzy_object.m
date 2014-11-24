%% This code is licensed under Creative Commons Attribution Share-Alike 3.0
% for the details about this license please go to
% http://creativecommons.org/licenses/by-sa/3.0/

%% Author: Javier V. Gómez  -  www.javiervgomez.com jvgomez _at_ ing.uc3m.es
% Date:  06/02/2013


function zfuzzy = def_fuzzy_object(length, width, uncert)

% The way a obstacle with uncertainty is created is to set the lenght and
% width of the zone it is 100% probably to find the obstacle. A 10 times
% bigger matrix is created and then the obstacle is placed on the middle.

% The parameters length and width have to be odd numbers when multiplied
% by 2 (i.e. 2,5).

% To make it fuzzy, the fast marching method is applied.

z=ones(10*length,10*width);
z(round(4.5*length):round(5.5*length),round(4.5*width):round(5.5*width))=0;

[row,col]=find(z<0.5);
start_points=[row,col]';
I=find(z<0.5);
z=z-uncert;
z(I)=z(I)+uncert;
options.nb_iter_max = Inf;
options.Tmax = sum(size(z));
[zfuzzy,S] = perform_fast_marching(z, start_points, options);
zfuzzy=min(zfuzzy,1);
