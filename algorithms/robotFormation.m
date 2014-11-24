function [F,G,fm2fmap_res,dist_foll1,dist_foll2,Bleader,Bfoll1,Bfoll2,jj] = robotFormation(fm2fmap,fm2fanim,fm2fveloc,fm2fdist,uncertainty,uvedoble,uvedobleo,steps_data,dist_data)



fm2fmap_axes = fm2fmap;
fm2fanim_axes = fm2fanim;
fm2fveloc_axes = fm2fveloc;
fm2fdist_axes = fm2fdist;
W = uvedoble;
Wo = uvedobleo;
steps = steps_data;
dist = dist_data;


%% Initializing, loading the map and creating the first potential with FMM.

Bleader=[]; Bfoll1=[]; Bfoll2=[];
dist_foll1=[]; dist_foll2=[];

W = W';


%% Picking start and end points.

[start_point,end_point] = pick_start_end_point(W);

%% Creating leader robot and finding the leader's path.
options.nb_iter_max = Inf;
options.Tmax = sum(size(W));

leader = Mobile_Robot(start_point, Wo);
leader = leader.setW(W);
leader = leader.findPath(end_point, options);

%% Defining the robots formation, mobile objects and the initial partial objectives.

% Here the orientation of the formation is calculated. Actually, the leader
% doesn't have partial objectives here, but it is necessary to calculate a
% small increment of the trajectory to find out the direction.
next_point = leader.path(steps,:)';
prev_point = leader.path(1,:)';
direc = (next_point-prev_point)/norm(next_point-prev_point);
perp_direc = [direc(2,1); -direc(1,1)];

partial_obj = [next_point, next_point-2*dist*direc+2*dist*perp_direc, next_point-2*dist*direc-2*dist*perp_direc];

foll1 = Mobile_Robot(partial_obj(:,2),Wo);
foll2 = Mobile_Robot(partial_obj(:,3),Wo);


% Setting the other robots as obstacles. 
robot_length = 10.5; % These two numbers always have to be odd when multiplied by 2.
robot_width  = 10.5;

%uncertainty  = 0.99;
rob = def_fuzzy_object(robot_length, robot_width, uncertainty)-1;
[m, n] = size(W);

%% Setting options to start the simulation. 
path_width =1; path_done_width =4; jj=1;

%% Starting the simulation.
while euc_dist(next_point,end_point)>10
    disp(euc_dist(next_point,end_point));
    leader = leader.setW(W);
    foll1 = foll1.setW(W);
    foll2 = foll2.setW(W);

    % Adding other robots as obstacles.
    leader = leader.addObstacle(rob, foll1.position);
    leader = leader.addObstacle(rob, foll2.position);

    foll1 = foll1.addObstacle(rob, leader.position);
    foll1 = foll1.addObstacle(rob, foll2.position);

    foll2 = foll2.addObstacle(rob, leader.position);
    foll2 = foll2.addObstacle(rob, foll1.position);

    % Calculating the next partial objectives.
    B1 = leader.W(round(leader.position(1)),round(leader.position(2)));
    B2 = foll1.W(round(foll1.position(1)),round(foll1.position(2)));
    B3 = foll2.W(round(foll2.position(1)),round(foll2.position(2)));
    
    partial_obj = [next_point, next_point-2*dist*direc+2*dist*B2*perp_direc, next_point-(3-B3)*dist*direc-2*dist*B3*perp_direc];
   
    % Saving in a vector the velocities and distances to the leader of the
    % robots.
    Bleader = [Bleader,B1];
    Bfoll1  = [Bfoll1,B2];
    Bfoll2  = [Bfoll2,B3];
    
    dist_foll1 = [ dist_foll1, euc_dist(foll1.position,leader.position)];
    dist_foll2 = [ dist_foll2, euc_dist(foll2.position,leader.position)];
    
    % Calculating the path for each robot.
    leader = leader.findPath(end_point, options);
    foll1 = foll1.findPath(partial_obj(:,2), options);
    foll2 = foll2.findPath(partial_obj(:,3), options);

    %% Plotting
    
     
   
    figure(FM2app2);
    axes(fm2fmap_axes);
    hold on;
    imagesc(Wo); axis image; axis off; set(gca,'box','on');
    
    
    leader.plot(steps);
    foll1.plot(round(length(foll1.path)/2));
    foll2.plot(round(length(foll2.path)/2));
   
    % Triangle of current formation
    line([leader.position(1),foll1.position(1)],[leader.position(2),foll1.position(2)],'Color','r');
    line([leader.position(1),foll2.position(1)],[leader.position(2),foll2.position(2)],'Color','r');
    line([foll1.position(1) ,foll2.position(1)],[foll1.position(2) ,foll2.position(2)],'Color','r');
    
    %Triangle of desired formation
    line([leader.position(1), partial_obj(1,2)],[leader.position(2),partial_obj(2,2)],'Color','g');
    line([leader.position(1), partial_obj(1,3)],[leader.position(2),partial_obj(2,3)],'Color','g');
    line([partial_obj(1,2) , partial_obj(1,3)],[partial_obj(2,2) ,partial_obj(2,3)],'Color','g');
    
    figure(FM2app2);
    colormap gray(256);
    hold off;
    drawnow
    axes(fm2fanim_axes); %%figure(2)
    imagesc(leader.W');
    axis image; axis xy; axis off;
    colormap gray(256);
    title('Velocities Map');
   
    
    if(jj>4)
        figure(FM2app2);
        axes(fm2fmap_axes); 
        G(jj-4) = getframe;  
        title('Formations Map');
        
        figure(FM2app2);
        axes(fm2fanim_axes); 
        F(jj-4) = getframe;
        title('Velocities Map');
     
        
    end
    % Frame counting. 
    jj=jj+1;
    
    
    
    %% Moving the robots to the next position.
    prev_point = next_point;
    next_point = leader.path(steps+1,:)';
    direc = (next_point-prev_point)/norm(next_point-prev_point);
    perp_direc = [direc(2,1); -direc(1,1)];
    
    leader = leader.move(steps);
    foll1 = foll1.move(round(length(foll1.path)/2));
    foll2 = foll2.move(round(length(foll2.path)/2));

end  
 
 %% Creating the final animation and the velocities plot.

axes(fm2fmap_axes);
movie(G,3);



figure(FM2app2);
axes(fm2fveloc_axes); %%figure;

m = min(min(min(Bleader),min(Bfoll1)),min(Bfoll2));
M = max(max(max(Bleader),max(Bfoll1)),max(Bfoll2));
axis on;
axis([1 jj-1 m-0.1, M+0.1]); %0.1

hold on;


time=1:length(Bleader);
plot(time,Bleader,'k');
plot(time,Bfoll1,'--b');
plot(time,Bfoll2,':r');

xlabel('Iteration');
ylabel('Speed (m/s)');

legend('Leader', 'Follower 1', 'Follower 2','location','Best');



%figure(FM2app2);
axes(fm2fdist_axes); %%figure; 

s = min(min(dist_foll1),min(dist_foll2));
S = max(max(dist_foll1),max(dist_foll2));


axis on;
axis([1 jj-1 s-1, S+1]); %1

hold on;

plot(time,dist_foll1,'--b');
plot(time,dist_foll2,':r');

legend('Follower 1' , 'Follower 2','location','Best');

xlabel('Iteration');
ylabel('% Perp_direct');


fm2fmap_res = Wo;


    
