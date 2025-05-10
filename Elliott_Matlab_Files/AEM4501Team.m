clc; 
clear; close all;

%% Start

a = 5;
b = 8;
% drone mass = 2000 kg and acceleration on the positive x-axis
P = 2000; % N


% titanium alloy
MatsSets(1).E = 125e9;
MatsSets(1).A = 1.96e-3; 
MatsSets(1).rho = 4500; 
yield_stress = 600e6; 
r_outer = 0.025; % Outer radius in meters (2.5 cm)
I = (pi/4) * r_outer^4; % Moment of inertia (m^4)

% %% Boron Composite
%     r_outer = 0.025;
%     r_inner = 0.023;
%     MatsSets(1).E = 200e9;
%     MatsSets(1).A = pi*(r_outer^2-r_inner^2); 
%     MatsSets(1).rho = 4500; 
%     yield_stress = 1000e6;
%     I = pi/4*(r_outer^4-r_inner^4);
%

% 2D truss
PD.N = 6;
PD.NodePos = [0,   0,   0; % node 1
              5,   0,   0; % node 2
              10.65,  5.65,   0; % node 3
              -5.65, 5.65, 0; % node 4 
              -5.65, 13.65, 0; % node 5
              10.65, 13.65, 0]; % node 6
              

PD.NE = 10;
PD.ElmConnect = [1 2;
                 1 4;
                 2 4;
                 2 3; 
                 3 4;
                 3 5;
                 3 6;
                 4 5;
                 5 6;
                 1 6];
%% --- DISTANCE BETWEEN NODES ---
fprintf('\n=== Element Lengths (Distance Between Nodes) ===\n');
fprintf('Element\tNode1\tNode2\tLength (m)\n');

PD.ElmLength = zeros(PD.NE, 1); % store lengths for reuse

for i = 1:PD.NE
    n1 = PD.ElmConnect(i,1);
    n2 = PD.ElmConnect(i,2);
    L = norm(PD.NodePos(n2,:) - PD.NodePos(n1,:)); % Euclidean distance
    PD.ElmLength(i) = L; % store it
    fprintf('%d\t%d\t%d\t%.4f\n', i, n1, n2, L);
end



PD.NM = 1;
PD.MatsSets = MatsSets;
PD.ElmMats = [1;
              1;
              1;
              1;
              1;
              1;
              1;
              1;
              1;
              1];
              
PD.BCType = [1, 0, 1;
             1, 0, 1;
             0, 1, 1;
             0, 1, 1;
             0, 1, 1;
             0, 1, 1];
             


% Note z displacement fixed for all since we're in 2D

PD.BCVal = [0, 0, 0;
            0, 0, 0;
            -P, 0, 0;
            -P, 0, 0;
            -P, 0, 0;
            -P, 0, 0];
            
           
PDans = PD_truss_static(PD);
PlotTruss(PDans,1000,'y','y');


PDans.SF_yield = zeros(PD.NE,1);
PDans.SF_buckling = NaN(PD.NE,1); % initialize with NaN
PDans.Pcr = NaN(PD.NE,1); % initialize Pcr

fprintf('\n=== Member Force, Stress, Yield SF, Buckling Load, and Buckling SF ===\n');
fprintf('Mem\tForce (N)\tStress (Pa)\tYield SF\tPcr (N)\t\tBuckling SF\n');
threshold_yield_SF = 1.2;

Ltot = 0;
for i = 1:PD.NE
    % Member ends
    n1 = PD.ElmConnect(i,1);
    n2 = PD.ElmConnect(i,2);
    
    % Member length
    
    L = norm(PD.NodePos(n2,:) - PD.NodePos(n1,:));
    Ltot = Ltot + L;

    % Force and stress
    Force = PDans.ElmForce(i); % (N)
    Stress = PDans.ElmStress(i); % (Pa)

    % % Yield safety factor
    SF_yield = yield_stress / abs(Stress);
    PDans.SF_yield(i) = SF_yield; % Store in PDans

 
    
    % Buckling load and safety factor (only if in compression)
    if Force < 0
        P_cr = (pi^2 * MatsSets(1).E * I) / (L^2); % Euler buckling load
        SF_buckling = P_cr / abs(Force);
        PDans.SF_buckling(i) = SF_buckling; % Store in PDans
        PDans.Pcr(i) = P_cr;
    end

    status = 'PASS'; % default

if SF_yield < threshold_yield_SF
    status = 'FAIL';
end

if Force < 0 && PDans.SF_buckling(i) < 1.0
    status = 'FAIL';
end

    % if Stress > 0
    % SF_yield = yield_stress / abs(Stress); % Use tensile yield strength
    % else
    % SF_yield = comp_strength / abs(Stress); % Use compressive yield strength
    % end
    % PDans.SF_yield(i) = SF_yield;


    % Print results
    if Force < 0
        fprintf('%d\t%.2f\t\t%.2e\t%.2f\t\t%.2f\t\t%.2f\t\t%s\n', ...
            i, Force, Stress, SF_yield, PDans.Pcr(i), PDans.SF_buckling(i), status);
    else
        fprintf('%d\t%.2f\t\t%.2e\t%.2f\t\t-\t\t-\t\t%s\n', ...
            i, Force, Stress, SF_yield, status);
    end

end

% MASS AND COST CALCULATION

total_mass = 0;
total_cost = 0;
cost_per_meter = 150; % $/m for titanium
%cost_per_meter = 800; % boron composite

for i = 1:PD.NE
    n1 = PD.ElmConnect(i,1);
    n2 = PD.ElmConnect(i,2);
    L = norm(PD.NodePos(n2,:) - PD.NodePos(n1,:)); % element length

    mass_i = L * MatsSets(1).rho * MatsSets(1).A;         % kg
    cost_i = L * cost_per_meter;          % $

    total_mass = total_mass + mass_i;
    total_cost = total_cost + cost_i;
end

% Store in PDans
PDans.mass = total_mass;
PDans.cost = total_cost;

% Print results
fprintf('\n=== Total Truss Mass and Cost ===\n');
fprintf('Total mass = %.2f kg\n', PDans.mass);
fprintf('Total cost = $%.2f\n', PDans.cost);


% DEFLECTION CHECK

fprintf('\n=== Node Deflection Check (Drone Docking Points) ===\n');
fprintf('Node\tDeflection (m)\tPass/Fail\n');

max_deflection_allowed = 0.10; % 10 cm = 0.10 m

% List of nodes to check
dock_nodes = [1, 2, 3, 4, 5, 6];

PDans.deflection = zeros(length(dock_nodes),1); % Store deflections

for i = 1:length(dock_nodes)
    node = dock_nodes(i);

    % Displacement vector in x and y (ignore z)
    Ux = PDans.U(node,1); % x-direction displacement (m)
    Uy = PDans.U(node,2); % y-direction displacement (m)

    % Store x and y separately
    PDans.deflection_x(i) = Ux;
    PDans.deflection_y(i) = Uy;
    
    % Check both directions
    if abs(Ux) <= max_deflection_allowed && abs(Uy) <= max_deflection_allowed
        status = 'Pass';
    else
        status = 'Fail';
    end

    % Print result
    fprintf('%d\tUx = %.4f m\tUy = %.4f m\t%s\n', node, Ux, Uy, status);
end

%% Anything below this this one is broken because the distance between drones was not 8 meters

% PD.N = 7;
% PD.NodePos = [1.5,   0,   0; % node 1
%               6.5,   0,   0; % node 2
%               -2.77,   4.27,   0; % node 3
%               10.77, 4.27, 0; % node 4 
%               4, 8.54, 0; % node 5
%               2.4, 3.22, 0; % node 6
%               4.2, 2.8, 0]; % node 7
% 
% 
% 
% PD.NE = 10;
% PD.ElmConnect = [1 6;
%                  6 5;
%                  6 7;
%                  5 7;
%                  7 2;
%                  2 4;
%                  4 7;
%                  6 3;
%                  3 1;
%                  7 1];
% 
% 
% 
% 
% PD.NM = 1;
% PD.MatsSets = MatsSets;
% PD.ElmMats = [1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1];
% 
% 
% PD.BCType = [1, 1, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1];
% 
% 
% % Note z displacement fixed for all since we're in 2D
% 
% PD.BCVal = [0, 0 0;
%             0, 0 0;
%             -P, 0 0;
%             -P, 0, 0;
%             -P, 0, 0;
%             0, 0, 0;
%             0, 0, 0];

%% 61.94 

% PD.N = 7;
% PD.NodePos = [1.5,   0,   0; % node 1
%               6.5,   0,   0; % node 2
%               -2.5,   4,   0; % node 3
%               10.5, 4, 0; % node 4 
%               4, 8, 0; % node 5
%               2.4, 3.22, 0; % node 6
%               4.2, 2.8, 0]; % node 7
% 
% 
% 
% PD.NE = 10;
% PD.ElmConnect = [1 6;
%                  6 5;
%                  6 7;
%                  5 7;
%                  7 2;
%                  2 4;
%                  4 7;
%                  6 3;
%                  3 1;
%                  7 1];
% 
% 
% 
% 
% PD.NM = 1;
% PD.MatsSets = MatsSets;
% PD.ElmMats = [1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1];
% 
% 
% PD.BCType = [1, 1, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1];
% 
% 
% % Note z displacement fixed for all since we're in 2D
% 
% PD.BCVal = [0, 0 0;
%             0, 0 0;
%             -P, 0 0;
%             -P, 0, 0;
%             -P, 0, 0;
%             0, 0, 0;
%             0, 0, 0];
% 


%% 62.22 Goated

% PD.N = 7;
% PD.NodePos = [1.5,   0,   0; % node 1
%               6.5,   0,   0; % node 2
%               -2.5,   4,   0; % node 3
%               10.5, 4, 0; % node 4 
%               4, 8, 0; % node 5
%               2.7, 3.65, 0; % node 6
%               4.25, 3.48, 0] % node 7
% 
% 
% 
% PD.NE = 10;
% PD.ElmConnect = [1 6;
%                  6 5;
%                  6 7;
%                  5 7;
%                  7 2;
%                  2 4;
%                  4 7;
%                  6 3;
%                  3 1;
%                  7 1];
% 
% 
% 
% 
% PD.NM = 1;
% PD.MatsSets = MatsSets;
% PD.ElmMats = [1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1];
% 
% 
% PD.BCType = [1, 1, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1];
% 
% 
% % Note z displacement fixed for all since we're in 2D
% 
% PD.BCVal = [0, 0 0;
%             0, 0 0;
%             -P, 0 0;
%             -P, 0, 0;
%             -P, 0, 0;
%             0, 0, 0;
%             0, 0, 0];
%% 65.36 kg

% PD.N = 7;
% PD.NodePos = [1.5,   0,   0; % node 1
%               6.5,   0,   0; % node 2
%               -2.5,   4,   0; % node 3
%               10.5, 4, 0; % node 4 
%               4, 8, 0; % node 5
%               1.25, 4, 0; % node 6
%               5.24, 4, 0]; % node 7
% 
% 
% PD.NE = 10;
% PD.ElmConnect = [1 6;
%                  6 5;
%                  6 7;
%                  5 7;
%                  7 2;
%                  2 4;
%                  4 7;
%                  6 3;
%                  3 1;
%                  6 2 ];
% 
% 
% 
% 
% PD.NM = 1;
% PD.MatsSets = MatsSets;
% PD.ElmMats = [1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1];
% 
% 
% PD.BCType = [1, 1, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1];
% 
% 
% % Note z displacement fixed for all since we're in 2D
% 
% PD.BCVal = [0, 0 0;
%             0, 0 0;
%             -P, 0 0;
%             -P, 0, 0;
%             -P, 0, 0;
%             0, 0, 0;
%             0, 0, 0];


%% STUPID 73.89 but could have been lower maybe

% PD.N = 12;
% PD.NodePos = [1.5,   0,   0; % node 1
%               6.5,   0,   0; % node 2
%               -2.5,   4,   0; % node 3
%               10.5, 4, 0; % node 4 
%               4, 8, 0; % node 5
%               1.25, 4, 0; % node 6
%               5.25, 4, 0  % node 7
%               1.7217, h, 0 % node 8
%               6.278, h, 0; % node 9
%               4, 0, 0; % node 10
%               1.3 h 0; % node 11
%               6.7 h 0]; % node 12
% 
% 
% PD.NE = 18;
% PD.ElmConnect = [1  8;
%                  8 6;
%                  7 9;
%                  2 9;
%                  6 5;
%                  6 7;
%                  5 7;
%                  4 7;
%                  6 3;
%                  8 10;
%                  9 10;
%                  6 2;
%                  1 11;
%                  11 3;
%                  2 12;
%                  12 4;
%                  8 11;
%                  9 12];
% 
% 
% 
% 
% PD.NM = 1;
% PD.MatsSets = MatsSets;
% PD.ElmMats = [1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1;
%               1];
% 
% 
% PD.BCType = [1, 1, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              0, 0, 1;
%              1, 1, 1;
%              0, 0, 1;
%              0, 0, 1];
% 
% 
% % Note z displacement fixed for all since we're in 2D
% 
% PD.BCVal = [0, 0 0;
%             0, 0 0;
%             -P, 0 0;
%             -P, 0, 0;
%             -P, 0, 0;
%             0, 0, 0;
%             0, 0, 0;
%             0, 0, 0;
%             0, 0, 0;
%             0, 0, 0;
%             0, 0, 0;
%             0, 0, 0];
% 
% PDans = PD_truss_static(PD);
% PlotTruss(PDans,1000,'y','y');
