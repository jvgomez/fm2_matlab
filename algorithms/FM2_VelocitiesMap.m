%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fast Marching Square algorithm.
% Javier V. Gómez - www.javiervgomez.com
% Carlos III University of Madrid
% http://roboticslab.uc3m.es/
% 22/10/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [F, time] = FM2_VelocitiesMap(map, sat)
%
% Inputs:
%   + map: MxN logical matrix representing the environment map as a binary 
%   occupancy gridmap, 0 (black) are obstacles and 1 (white) free space.
%
%   + sat: double scalar for the saturation value.
%
% Outputs:
%   o F: MxN double matrix representing the velocities map normalized.
%
%   o time: the time elapsed in seconds for computing F.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [F, time] = FM2_VelocitiesMap(map, sat)

tic();
if sat > 0
    F = rescale( double(map) );
    [row,col]=find(F<0.5); %Getting obstacles.
    start_points = [row,col]';

    options.nb_iter_max = Inf;
    options.Tmax = sum(size(F));
    [F,~] = perform_fast_marching(F, start_points, options);

    F = rescale(min(rescale(F), sat));
else
    F = double(map);
end
time = toc();

end

