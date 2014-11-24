%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main function for the Fast Marching Learning algorithm.
% Javier V. Gómez - www.javiervgomez.com
% Carlos III University of Madrid
% http://roboticslab.uc3m.es/
% 22/11/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [F, T, end_point] = FML (map, demos, sat, aoi_size)
% 
% Inputs:
%   + map: MxN logical matrix representing the environment map as a binary 
%   occupancy gridmap, 0 (black) are obstacles and 1 (white) free space.
%
%   + demos: cell of size n containing demonstrations as a points array with
%   format [x1....xm; y1....ym]. Points are given in pixels within map.
%
%   + sat: double scalar for the saturation value.
%
%   + aoi_size: double scalar defining the area of interest size.
%
% Las salidas de la función son:
%
%   o F: MxN double matrix representing the velocities map normalized.
%
%   o T: MxN double matrix representing the times-of-arrival map
%   normalized. In this case it is the reproductions field. The wave source
%   is end_point.
% 
%   o end_point: mean of the last point of all the demonstrations given in
%   pixels.
%
%   o dx, dy: MxN double matrix containing gradients in x and y directions
%   with discontinuities removed.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [F, T, end_point, dx, dy] = FML (map, demos, sat, aoi_size)
% Preparing variables.
global fmLF_axes
[F_nosat, t]= FM2_VelocitiesMap(map',sat); % Initial velocities map normalized.

F_init  = min(sat, F_nosat);      % Velocities map saturated.



F = F_init;                     % Final velocities map.
Fpo = ones(size(F));            % Map with reconstructed paths.
zero_index = F_init==0;         % Indices of the map representing obstacles.

n = length(demos);

% Fast Marching options.
options.nb_iter_max = Inf;
options.Tmax        = sum(size(F));

% Teaching the system modifying F.
for k = 1:n
    % Trajectory reconstruction from the points in demos. It is done by
    % computing the FM2 path between two consecutive points of demos.
    for i = 1:length(demos{k}(1,:))-1;
        options.end_points  = demos{k}(:,i+1);
        [T,~] = perform_fast_marching(F, demos{k}(:,i), options);
        path = compute_geodesic(T,demos{k}(:,i+1), options);

        Wpoi = ones(size(F));
            for j=1:length(path)
                Wpoi(round(path(1,j)),round(path(2,j))) = 0;
            end
        Fpo = min(Fpo,Wpoi);    
    end
end



% Dilating and "fuzzying" the learned path.
SE = strel('disk', aoi_size);
Fpo = imdilate(~Fpo, SE);
[Wp, t2] = FM2_VelocitiesMap(Fpo, sat);%1-sat

F = F + Wp;



% Obstacle reposition. It can happen that learning removes some of this
% info, restored here.
F(zero_index) = 0;



% Computing T, the reproductions field.
end_point = zeros(2,1);
for k = 1:n
    end_point_aux = demos{k}(:,end);
    end_point = end_point + end_point_aux;
end
end_point = end_point ./n;

options.end_points  = [];
[T,~] = perform_fast_marching(F, end_point, options);



% Removing discontinuities of FML.
[dx,dy] = gradient(T', 1,1);
dy(dy < -1) = 0;
dy(dy >  9999) = 0;
dx(dx < -1) = 0;





