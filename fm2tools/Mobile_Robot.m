%% This code is licensed under Creative Commons Attribution Share-Alike 3.0
% for the details about this license please go to
% http://creativecommons.org/licenses/by-sa/3.0/

%% Author: Javier V. Gómez  -  www.javiervgomez.com jvgomez _at_ ing.uc3m.es
% Date:  06/02/2013

classdef Mobile_Robot
    
    properties
        position;  %[x;y]
        end_point; %[x;y]
        path;      %[x1, y1; x2,y2... ]
        Wo_init = [];
        Wo = [];
        W=[];         %[xx,yy] double map, first potential.
        D=[];         %[xx,yy] double map, second potential.     
        path_done=[]; %[x1, y1; x2,y2... ]
        
        % Properties to make the code faster.
        m;
        n;
        W1;
    end
    
    methods
        function mr = Mobile_Robot(position, Wo)
            mr.Wo_init = Wo;
            mr.Wo = Wo;
            mr.position=position;
            [mr.m mr.n] = size(Wo');
            mr.W1 = ones(mr.m, mr.n);
            mr.path_done=[];

        end %Mobile_Robot
        function mr = setW(mr,first_pot)
            mr.W = first_pot;
        end %setW
        function mr = addBinaryObstacle(mr, size, pos) 
            %This function adds and obstacle to the INITIAL MAP. Works
            %without saturation.
            Woo = ones(mr.n, mr.m);
            z=ones(10*size,10*size);
            z(round(4.5*size):round(5.5*size),round(4.5*size):round(5.5*size))=0;
            Woo = sum_in(Woo,z-1,[pos(2) pos(1)]); %Pos(2) pos(1) is because Wo is not transposed.
            mr.Wo = min(double(mr.Wo), Woo);
            mr.Wo = uint8(mr.Wo);
        end
        function mr = calcW(mr) %Use together with addBinaryObstacle
            mr.W  = FMdist(mr.Wo');
            mr.W  = log(1+mr.W); % Like 2 dimensiones electric field.
            mr.W  = rescale( double(mr.W) ); 
            mr.Wo = mr.Wo_init; %Restarting the Wo map for the robot once W is computed.
        end %calcW
        function mr = addObstacle(mr, obs, pos) 
            %This function adds and obstacle to the FIRST POTENTIAL.
            W11 = sum_in(mr.W1, obs, pos); %Funciones_Propias
            mr.W = min(W11, mr.W);    
        end %addObstacle
        function mr = findPath(mr, endpoint, options)
            %if endpoint==mr.position
            %    disp('Robot is actually at the end_position');
           % else
                mr.end_point = endpoint;
                %These functions are in the folder Funciones_Propias and
                %the Fast Marching Toolbox.
                [mr.D,S] = perform_fast_marching(mr.W, mr.end_point, options);
                mr.path = compute_geodesic(mr.D,mr.position, options)';
            %end
        end %calculate_path
        function mr = move(mr, steps)
            steps = min(steps, length(mr.path));
            mr.position = mr.path(steps,:)';
            mr.path_done = [mr.path_done; mr.position'];
        end %move    
        function plot(mr,steps)
            plot(mr.end_point(1), mr.end_point(2), 'o', 'LineWidth', 1)
            if ~isempty(mr.path)
                plot(mr.path(:,1),mr.path(:,2), 'b-', 'LineWidth', 1);
                %plot(mr.path(1:steps,1),mr.path(1:steps,2), 'b-', 'LineWidth', 4);
            end
            
            %Robot shape plot
            Xr = [mr.path(1,1), mr.path(1,2), atan2(mr.path(2,2)-mr.path(1,2),mr.path(2,1)-mr.path(1,1))];
            escala = 15;
            color = 'b';
            P_ini=[ 0 1 1 0 2 3 3 4 4 5 5 4 4 3 3 4 4 5 5 4 4 3 3 2 -2 -3 -3 -4 -4 -5 -5 -4 -4 -3 -3 -4 -4 -5 -5 -4 -4 -3 -3 -2 0 -1 -1 0 ;
            0 0 6 6 6 5 4 4 6 6 2 2 4 4 -4 -4 -2 -2 -6 -6 -4 -4 -5 -6 -6 -5 -4 -4 -6 -6 -2 -2 -4 -4 4 4 2 2 6 6 4 4 5 6 6 6 0 0];
            P_ini = P_ini/6*escala;
            theta = Xr(3)-pi/2;% rotate
            c=cos(theta);
            s=sin(theta);
            P=[c -s; s c]*P_ini; % rotate
            P(1,:) = P(1,:) + Xr(1); % translate to x
            P(2,:) = P(2,:) + Xr(2);
            plot(P(1,:),P(2,:),color,'LineWidth',1.5);
            plot(Xr(1),Xr(2),sprintf('%s+',color));
            
        end %plot
    end
    
end