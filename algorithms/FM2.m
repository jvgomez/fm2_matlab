%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main function for the Fast Marching Square algorithm.
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
%   + start: 2x1 integer representing the start point (given as [x,y]).
%
%   + goal: 2x1 integer representing the goal point (given as [x,y]).
%
% Outputs:
%   o F: MxN double matrix representing the velocities map normalized.
%
%   o T: MxN double matrix representing the times-of-arrival map normalized.
%
%   o path: 2xP double matrix containing the path.
%
%   o vels: 2x100 vector containing relative velocity for 100 waypoints of
%   the original path and the index of the waypoint within the initial path.
%
%   o times:  double scalar, fm2times(1) is the time employed to compute fm2F,
%                            fm2times(2) is the time employed to compute fm2T and path.
%
%
%   This function depends on Gabriel Peyré's Fast Marching Toolbox and 
%   other functions such as trimpath and FM2_VelocitiesMap.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [F, T, path, vels, times] = FM2(map, sat, start, goal)

 % Changing order of start and goal because of the (x,y) - (row,col)
 % correspondence.
start = [start(2); start(1)];
goal = [goal(2); goal(1)];

tic();
F = FM2_VelocitiesMap(map, sat);
times(1) = toc();

 % Parameters setting
options.nb_iter_max = Inf;
options.Tmax        = sum(size(F));
options.end_points  = start;

 % Second wave and gradient descent.
tic();
[T,~] = perform_fast_marching(F, goal, options);
path = compute_geodesic(T,start, options);
times(2) = toc();

 % Obtaining the velocities profile.
aux_path = trimpath(path, 100);

vels = [];
for i=1:size(aux_path,2)
    vels(:,i) = [F(round(aux_path(1,i)),round(aux_path(2,i))) ; aux_path(3,i)]; 
end

 % Giving the path in the expected way (x,y).
path = [path(2,:); path(1,:)]; 
