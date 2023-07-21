%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title: Project1.mlx
% Course: Introduction to Intelligent Robots I (CPET - 371)
% Developer: Josh Skotarczak (CPET)
% Date: 09/17/2022
% Description: 2 Link Manipulator Arm Main Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clear All Output Before Running
clc;clear all;clf; %Clear Of Command Window Clear All Varibles,Clear Figure

% Insert Object Obstacle
ObstacleOn = 0;

%  Length of Arms
Arm1 = 2; % Length Link 1
Arm2 = 2; % Length Link 2

% Animation Smoothness
Frames = 100;

%Debug
Debug = 0;

%% Initally Plot Robot Arm In Orginal Position And Obstacle Object
%Plot Functions
grid on
title('2 Link Manipulator Arm')
axis equal
axis([-6 6 0 8])
xticks(-6:1:6)
xlabel('X Coordinates')
ylabel('Y Coordinates')
hold on

% Plot Arm 1
a1= plot([0, Arm1],[0, 0],'b','LineWidth',1.5);
% Plot Arm 2
a2= plot([Arm1, Arm1 + Arm2],[0, 0],'g','LineWidth',1.5);

% Obstacle Placement (X,Y) coordinates
if ObstacleOn == 1
    % Rectangular Obect 1 Placement
    obstacle = polyshape([-1 -1 1 1], [1 2 2 1]);
    plot(obstacle)
elseif ObstacleOn == 2
    obstacle = polyshape([-3 -3 -1 -1], [2 3 3 2]);
    plot(obstacle) 
end

%Initialization Of  Variables
Solution1 = 0;
Solution2 = 0;
Arm1Collison = 0;
Arm2Collison = 0;

if (Debug == 1)
    %Plot Degree of Reachability Sphere (Only For
    th = 0:pi/50:2*pi;
    xunit = (Arm1+Arm2) * cos(th) + 0;
    yunit = (Arm1+Arm2) * sin(th) + 0;
    h = plot(xunit, yunit);
end

%% LOOP
while(1)
    %Assign Coordinates Based On Current Position of Links Within Graph
    a1x1Current=a1.XData(2);
    a1y1Current=a1.YData(2);
    a2x2Current=a2.XData(2);
    a2y2Current=a2.YData(2);
    %Find Current Theta Values
    [theta11Prev, theta12Prev, theta21Prev, theta22Prev] = ...
    InverseKinematicsFunction(Arm1 ,Arm2, a2x2Current, a2y2Current);
    % If Previous Solution Was Picked
    if(Solution1 == 1)
        theta1Current = theta11Prev;
        theta2Current = theta21Prev;
    elseif (Solution2 == 1)
        theta1Current = theta12Prev;
        theta2Current = theta22Prev;
    else %If No Solution Prev Picked : Both Theta Solutions are the Same
        theta1Current = theta11Prev;
        theta2Current = theta21Prev;
    end
    
    % Display Current Theta And Link Postion Values And Collison
    clc;
    if Arm1Collison == 1
        disp('Object Collison Detected At Arm 1')
    elseif Arm2Collison == 1
        disp('Object Collison Detected At Arm 2')
    end
    disp("Current Theta Values")
    Theta1Show = ['Theta 1: ',num2str(theta1Current),' Degrees'];
    Theta2Show = ['Theta 2: ', num2str(theta2Current),' Degrees'];
    disp(Theta1Show)
    disp(Theta2Show)
    disp("Current End Of Link Position")
    px2Show = ['X Value: ',num2str(a2.XData(2))];
    py2Show = ['Y Value: ',num2str(a2.YData(2))];
    disp(px2Show)
    disp(py2Show)
    
    % Reset Varibles
    Solution1 = 0;
    Solution2 = 0;
    Arm1Collison = 0;
    Arm2Collison = 0;
    % Prompt User ForInputs Find These Cordinates
    prompt = {'Enter X Coordinate:','Enter Y Coordinate:'};
    dlgtitle = ('Move Robot');
    dims = [1 50]; %Input Box Size
    definput = {num2str(a2.XData(2)),num2str(a2.YData(2))};
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    %Check If User Cancels Input
    if isempty(answer)
        break % Exit Program Without Error
    end
    % Assign New px2 and py2 from user Input
    px2= str2double(answer{1});
    py2= str2double(answer{2});

    % Check if Given Value Is Not Within Quad I & II
    if (px2<=0) && (py2<=0)
        % If Selected Coordintes Are in Quadrant III Go Flat In Quadrant II
        theta11 = 180;
        theta21 = 0;
        theta12 = 180;
        theta22 = 0;
    elseif(px2>=0) && (py2<=0)
        % If Selected Coordintes Are in Quadrant IV Go Flat in Quadrant I
        theta11 = 0;
        theta21 = 0;
        theta12 = 0;
        theta22 = 0;
    else
        % Else ~ Proceed With Inverse Kinematics Function Calculations 
        [theta11, theta12, theta21, theta22] = ...
        InverseKinematicsFunction(Arm1 ,Arm2, px2, py2);
        %Define Where Middle Point should be for both solutions
        px1Sol1 = Arm1*cosd(theta11);
        py1Sol1 = Arm2*sind(theta11);
        % Soltion 2
        px1Sol2 = Arm1*cosd(theta12);
        py1Sol2 = Arm2*sind(theta12);
        % Check if Middle Point would be out of Range %%No Quad III & IV%%
        if (px1Sol1<0) && (py1Sol1<0)||(px1Sol1>0) && (py1Sol1<0)
            %If Solution 1 is Out of Range then Choose Solution 2
            px1Final = px1Sol2;
            py1Final = py1Sol2;
            Solution2 = 1;
        elseif (px1Sol2<0) && (py1Sol2<0)||(px1Sol2>0) && (py1Sol2<0)
            %If Solution 2 is Out of Range then Choose Solution 1
            px1Final = px1Sol1;
            py1Final = py1Sol1;
            Solution1 = 1;
        else
            %If Both Are Within Range Find Which One Occurs Least Amount Of
            %Movement from Current Position
            d1 = sqrt((px1Sol1-a1x1Current)^2 + (py1Sol1 - a1y1Current)^2);
            d2 = sqrt((px1Sol2-a1x1Current)^2 + (py1Sol2 - a1y1Current)^2);
            if (d1<d2)
                px1Final = px1Sol1;
                py1Final = py1Sol1;
                Solution1 = 1;
            elseif (d1>d2)
                px1Final = px1Sol2;
                py1Final = py1Sol2;
                Solution2 = 1;
            elseif (d1==d2)
                px1Final = px1Sol2;
                py1Final = py1Sol2;
            end
        end
    end

    % Set Final "Destination" Theta Values Based On Solution Is Choosen
    if (Solution1 == 1)
        theta1Final = theta11;
        theta2Final = theta21;
    elseif (Solution2 == 1)
        theta1Final = theta12;
        theta2Final = theta22;
    else
        % (Arm is Streched Out) Both Theta Solutions are the Same
        % Or Given Coordinates are in Quad III or IV Go Flat
        theta1Final = theta11;
        theta2Final = theta21;
    end

    % Create An Array of Values
    theta1Array=linspace(theta1Current,theta1Final,Frames);
    theta2Array=linspace(theta2Current,theta2Final,Frames);

    %% Iterate Through Theta Arrays
    for i=1:Frames
        % If Obstacle Is Present
        if ObstacleOn > 0
        %Collison Detection
        %Define Matrix of Arm 1 Line
        a1Matrix = [a1.XData(1) a1.YData(1);a1.XData(2), a1.YData(2)];
        %Find If Current Postion of Arm 1 Intersects obstacle via Matrices
        [a1Collison, a1NoCollison] = intersect(obstacle,a1Matrix);

        if isempty(a1Collison)
            %disp('No Object Collison With Arm 1')
        else
           % Display That there was Obejct Collison
           disp('Object Collison Detected At Arm 1')
           Arm1Collison = 1;
           %Rebound ~ Back Arm Off from Object Colliosn Via 2 Degrees
           % If Arm is Moving Left Minus 2 Degrees
           if theta1Array(i) < theta1Array(i + 1)
           a1.XData(2) = Arm1*cosd(theta1Array(i)-2);
           a1.YData(2) = Arm2*sind(theta1Array(i)-2);
           a2.XData(1) = Arm1*cosd(theta1Array(i)-2);
           a2.YData(1) = Arm2*sind(theta1Array(i)-2);
           a2.XData(2) = Arm1*cosd(theta1Array(i)-2) + ...
           Arm2*cosd(theta1Array(i)-2 + theta2Array(i)-2);
           a2.YData(2) = Arm2*sind(theta1Array(i)-2) + ...
           Arm2*sind(theta1Array(i)-2 + theta2Array(i)-2);
           % If Arm is Moving Right Add 2 Degrees
           elseif theta1Array(i) > theta1Array(i + 1)
           a1.XData(2) = Arm1*cosd(theta1Array(i)+2);
           a1.YData(2) = Arm2*sind(theta1Array(i)+2);
           a2.XData(1) = Arm1*cosd(theta1Array(i)+2);
           a2.YData(1) = Arm2*sind(theta1Array(i)+2);
           a2.XData(2) = Arm1*cosd(theta1Array(i)+2) + ...
           Arm2*cosd(theta1Array(i)+2 + theta2Array(i)+2);
           a2.YData(2) = Arm2*sind(theta1Array(i)-2) + ...
           Arm2*sind(theta1Array(i)+2 + theta2Array(i)+2);
           end
           break
        end
        
        %Collison Detection
        %Define Matrix of Arm 2 Line
        a2Matrix = [a2.XData(1) a2.YData(1);a2.XData(2), a2.YData(2)];
        %Find If Current Postion of Arm 2 Intersects obstacle via Matrices
        [a2Collison, a2NoCollison] = intersect(obstacle,a2Matrix);
        
        if isempty(a2Collison)
            %disp('No Object Collison With Arm 2')
        else
           % Display That there was Obejct Collison
           %disp('Object Collison Detected At Arm 2')
           Arm2Collison = 1;
           %Rebound ~ Back Arm Off from Object Colliosn
           % If Arm is Moving Left Minus 2 Degrees
           if theta2Array(i) < theta2Array(i + 1)
           a1.XData(2) = Arm1*cosd(theta1Array(i)-2);
           a1.YData(2) = Arm2*sind(theta1Array(i)-2);
           a2.XData(1) = Arm1*cosd(theta1Array(i)-2);
           a2.YData(1) = Arm2*sind(theta1Array(i)-2);
           a2.XData(2) = Arm1*cosd(theta1Array(i)-2) + ...
           Arm2*cosd(theta1Array(i)-2 + theta2Array(i)-2);
           a2.YData(2) = Arm2*sind(theta1Array(i)-2) + ...
           Arm2*sind(theta1Array(i)-2 + theta2Array(i)-2);
           % If Arm is Moving Right Add 2 Degrees
           elseif theta2Array(i) > theta2Array(i + 1)
           a1.XData(2) = Arm1*cosd(theta1Array(i)+2);
           a1.YData(2) = Arm2*sind(theta1Array(i)+2);
           a2.XData(1) = Arm1*cosd(theta1Array(i)+2);
           a2.YData(1) = Arm2*sind(theta1Array(i)+2);
           a2.XData(2) = Arm1*cosd(theta1Array(i)+2) + ...
           Arm2*cosd(theta1Array(i)+2 + theta2Array(i)+2);
           a2.YData(2) = Arm2*sind(theta1Array(i)-2) + ...
           Arm2*sind(theta1Array(i)+2 + theta2Array(i)+2);
           end
           break
        end
        end
        
        % Update Coordinates Instead Of Plotting Over and Over
        a1.XData(2) = Arm1*cosd(theta1Array(i));
        a1.YData(2) = Arm2*sind(theta1Array(i));
        a2.XData(1) = Arm1*cosd(theta1Array(i));
        a2.YData(1) = Arm2*sind(theta1Array(i));
        a2.XData(2) = Arm1*cosd(theta1Array(i)) + ...
        Arm2*cosd(theta1Array(i) + theta2Array(i));
        a2.YData(2) = Arm2*sind(theta1Array(i)) + ...
        Arm2*sind(theta1Array(i) + theta2Array(i));
        pause(0.05);
    end

end %While Loop

