function predator_prey
 close all
 g = 9.81;
 mr = 100; % Mass of predator, in kg
 my = 10.; % Mass of prey, in kg
 Frmax = 1.3*mr*g; % Max force on predator, in Newtons
 Fymax = 1.4*my*g; % Max force on prey, in Newtons
 c = 0.2; % Drag coeft, in N s/m
 initial_w = [150,1000,0,1000,0,0,0,0]; % Initial position/velocity
 force_table_predator = rand(51,2)-0.5;
 force_table_prey = rand(51,2)-0.5;
 options = odeset('Events',@event,'RelTol',0.01);
 [time_vals,sol_vals] = ode45(@(t,w) ...
  eom(t,w,mr,my,Frmax,Fymax,c,force_table_predator,force_table_prey), ...
 [0:1:250],initial_w,options);
 animate_projectiles(time_vals,sol_vals);
 figure
 plot(time_vals, (sol_vals(:,6).^2+sol_vals(:,5).^2).^0.5)
 ylabel("speed")
 hold on 
  plot(time_vals, (sol_vals(:,7).^2+sol_vals(:,8).^2).^0.5)

end
function dwdt = eom(t,w,mr,my,Frmax,Fymax,c,forcetable_r,forcetable_y)
% Extract the position and velocity variables from the vector w
% Note that this assumes the variables are stored in a particular order in w.
 pr=w(1:2); vr=w(5:6); py=w(3:4); vy=w(7:8);
 g = 9.81;
% Compute all the forces on the predator
 amiapredator = true;
 Fr = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy);
% The force table varies between +/- 0.5 so this makes the random force
% vary between +/- 0.2*mr*g
 Frrand = 0.4*mr*g*compute_random_force(t,forcetable_r);
 Frdrag = -vr*norm(vr)*c;
 Frgrav = -mr*g*[0;1];
 Frtotal = Fr+Frrand+Frdrag+Frgrav;
 %Test force - Use this for testing only
 %Frtotal = Fr;
 amiapredator = false;
% Write similar code below to call your compute_f_groupname function to
% compute the force on the prey, determine the random forces on the prey,
% and determine the drag forces on the prey
 Fy = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy);
 Fyrand = 0.4*my*g*compute_random_force(t,forcetable_r);
 Fydrag = -vy*norm(vy)*c;
 Fygrav = -my*g*[0;1];
 Fytotal = Fy+Fyrand+Fydrag+Fygrav;
 t    %Uncomment this line to display time in the command window as it runs
 %Test force - Use this for testing only
 %Fytotal = Fy;
 dwdt = [vr;vy;Frtotal/mr;Fytotal/my]; 
 
 end
 function [event,stop,direction] = event(t,w)
% Event function to stop calculation when predator catches prey
% Write your code here? For the event variable, use the distance between
% predator and prey. You could add other events to detect when predator/prey hit
% the ground as well. See the MATLAB manual for how to detect and
% distinguish between multiple events if you want to do this
pr=w(1:2); vr=w(5:6); py=w(3:4); vy=w(7:8);
dist = norm(pr-py); %addcomment
event = dist - 1;
stop = 1;
direction = -1; %Could also be 0
 end
 
 %The actual code where we write the behavior
 function F = compute_f_groupname(t,Frmax,Fymax,amiapredator,pr,vr,py,vy)
 mp = 100;
% Write your code to compute the forces below. This will be the function
% that you must submit to the competition. You don?t need to submit the rest
% of the code ? that?s just for test purposes
 
 
%Test code, does nothing meaningful currently
%Predator code
if (amiapredator)
    distance = sqrt((pr(1)-py(1))^2 + (pr(2)-py(2))^2);
    velocity_relative = vr - vy;
    distance_apart = pr - py;
    time_to_catch = norm(distance_apart)/norm(velocity_relative);
    predicted_py = py + vy*(time_to_catch/3);
    dist_between = predicted_py - pr;
    angle = dist_between/norm(dist_between); 
    if t == 0
        F = Frmax*[1;0];
    elseif pr(2) < 30
        if distance < 40
            F = Frmax*[angle(1);angle(2)];
        elseif norm(vr(2)) > 10
            F = Frmax*-vr; 
        elseif pr(2) < 6
            F = Frmax*[0;1];
        else
            F = Frmax * [0;1];
        end
    elseif distance > 60
        F = Frmax*[angle(1);angle(2)];
    elseif distance < 60
        if distance < 30
            F = Frmax*((py-pr)/norm(py-pr));
        elseif norm(vr) > 25
            F = Frmax*-vr;
        else
            F = Frmax*[angle(1);angle(2)];
        end
    elseif pr(2) < 100 && distance < 100
        F = Frmax*[angle(1);angle(2)];
        %For loop to break for 2 seconds then accelerate toward prey
    end

    %F = Fscaled * 
%Prey code
else
    %Apply force perpendicular to predator velocity
    if t==0
       F = Fymax*[0;-1]; 
    else
        %max upward acceleration
        c = 0.2; my = 10; g = 9.81;
        Fydrag = -vy*norm(vy)*c;
        Fygrav = -my*g*[0;1];
        %assume random force acts in largest downward (opposing) magnitude
        Fyrand = -0.2*my*g*[0;1];
        Fytotal = Fymax+Fyrand+Fydrag+Fygrav;
        max_upward_a = Fytotal/my;
        %height prediction - prevent prey from hitting the ground
        syms t_unknown y0
        eq2 = y0 == -vy*t_unknown + 0.5*max_upward_a*t_unknown^2;
        solution = solve([eq2], [y0, t_unknown]);
        if py(2) < solution.y0 + 20
            py
            F = Fymax*[0;1];
        else  
            perp_vector = [-vr(2); vr(1)]/norm(vr);
            direction_vector = [perp_vector(1), perp_vector(2)*50/py(2)]
            F = Fymax*direction_vector/norm(direction_vector);
        end
    end
end
 
end
 
%Random Force function
function F = compute_random_force(t,force_table)
% Computes value of fluctuating random force at time t, where 0<t<250.
% The variable force_table is a 51x2 matrix of pseudo-random
% numbers between -0.5 and 0.5, computed using
% force_table = rand(51,2)-0.5;
% NB ? THE FORCE TABLE MUST BE DEFINED OUTSIDE THIS FUNCTION
% If you define it in here it fries the ode45 function
F = [interp1([0:5:250],force_table(:,1),t);...
 interp1([0:5:250],force_table(:,2),t)];
end
 %change
%Animation function
function animate_projectiles(t,sols)
figure
xmax = max(max(sols(:,3)),max(sols(:,1)));
xmin = min(min(sols(:,3)),min(sols(:,1)));
ymax = max(max(sols(:,4)),max(sols(:,2)));
ymin = min(min(sols(:,4)),min(sols(:,2)));
dx = 0.1*(xmax-xmin)+0.5;
dy = 0.1*(ymax-ymin)+0.5;
for i = 1:length(t)
 clf
 plot(sols(1:i,3),sols(1:i,4),'LineWidth',2,'LineStyle',...
 ':','Color',[0 0 1]);
 ylim([ymin-dy ymax+dy]);
 xlim([xmin-dx xmax+dx]);
 hold on
 plot(sols(1:i,1),sols(1:i,2),'LineWidth',2,'LineStyle',':',...
 'Color',[1 0 0]);
 plot(sols(i,1),sols(i,2),'ro','MarkerSize',11,'MarkerFaceColor','r');
 plot(sols(i,3),sols(i,4),'ro','MarkerSize',5,'MarkerFaceColor','g');
 pause(0.1);
end
end

