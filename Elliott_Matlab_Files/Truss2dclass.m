%% --- SETUP ---
a = 5;   % Distance between A and B (horizontal)
b = 8;   % Platform height (not used as max height here)
P = 2000; % N force for each drone (standard case)

% Material: Titanium Alloy
MatsSets(1).E = 125e9;       % Young's Modulus (Pa)
MatsSets(1).A = 1.96e-3;     % Cross-sectional Area (m^2)
MatsSets(1).rho = 4500;      % Density (kg/m^3)
yield_stress = 600e6;        % Yield Strength (Pa)
r_outer = 0.025;             % Outer radius (m)
I = (pi/4) * r_outer^4;      % Moment of Inertia (m^4)

%% --- NODE POSITIONS ---
PD.N = 6;
PD.NodePos = [1.5, 0, 0;   % Node 5 (Support A)
              6.5, 0, 0;   % Node 6 (Support B)
              1,   3, 0;   % Node 1 (Drone Node)
              6,   3, 0;   % Node 2 (Drone Node)
              2.75, 4, 0;  % Node 3 (Drone Node)
              4.75, 4, 0]; % Node 4 (Drone Node)

% Nodes 1-4 are docking points.
% Nodes 5-6 are fixed supports.

%% --- ELEMENT CONNECTIONS ---
PD.NE = 9;
PD.ElmConnect = [5, 1;
                 5, 3;
                 6, 2;
                 6, 4;
                 1, 2;
                 1, 3;
                 2, 4;
                 3, 4;
                 3, 2];
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

%% --- MATERIALS ---
PD.NM = 1;
PD.MatsSets = MatsSets;
PD.ElmMats = ones(PD.NE,1);

%% --- BOUNDARY CONDITIONS ---
PD.BCType = [1, 1, 1;    % Node 5 (Support)
             1, 1, 1;    % Node 6 (Support)
             0, 1, 1;    % Node 1 (Drone)
             0, 1, 1;    % Node 2 (Drone)
             0, 1, 1;    % Node 3 (Drone)
             0, 1, 1];   % Node 4 (Drone)

PD.BCVal = [0, 0, 0;
            0, 0, 0;
           -P, 0, 0;
           -P, 0, 0;
           -P, 0, 0;
           -P, 0, 0];

%% --- SOLVE STATIC PROBLEM ---
PDans = PD_truss_static(PD);
PlotTruss(PDans,1000,'y','y');

%% --- FORCE, STRESS, BUCKLING ANALYSIS ---
PDans.SF_yield = zeros(PD.NE,1);
PDans.SF_buckling = NaN(PD.NE,1);
PDans.Pcr = NaN(PD.NE,1);

fprintf('\n=== Member Force, Stress, Yield SF, Buckling Load, and Buckling SF ===\n');
fprintf('Mem\tForce (N)\tStress (Pa)\tYield SF\tPcr (N)\tBuckling SF\n');

for i = 1:PD.NE
    n1 = PD.ElmConnect(i,1);
    n2 = PD.ElmConnect(i,2);
    L = norm(PD.NodePos(n2,:) - PD.NodePos(n1,:));
    
    Force = PDans.ElmForce(i);
    Stress = PDans.ElmStress(i);

    SF_yield = yield_stress / abs(Stress);
    PDans.SF_yield(i) = SF_yield;

    if Force < 0
        P_cr = (pi^2 * MatsSets(1).E * I) / (L^2);
        SF_buckling = P_cr / abs(Force);
        PDans.SF_buckling(i) = SF_buckling;
        PDans.Pcr(i) = P_cr;
    end

    if Force < 0
        fprintf('%d\t%.2f\t%.2e\t%.2f\t%.2f\t%.2f\n',i, Force, Stress, SF_yield, PDans.Pcr(i), PDans.SF_buckling(i));
    else
        fprintf('%d\t%.2f\t%.2e\t%.2f\t-\t-\n',i, Force, Stress, SF_yield);
    end
end

%% --- MASS AND COST CALCULATION ---
total_mass = 0;
total_cost = 0;
cost_per_meter = 150; % $/m for titanium

for i = 1:PD.NE
    n1 = PD.ElmConnect(i,1);
    n2 = PD.ElmConnect(i,2);
    L = norm(PD.NodePos(n2,:) - PD.NodePos(n1,:));

    mass_i = L * MatsSets(1).rho * MatsSets(1).A;
    cost_i = L * cost_per_meter;

    total_mass = total_mass + mass_i;
    total_cost = total_cost + cost_i;
end

PDans.mass = total_mass;
PDans.cost = total_cost;

fprintf('\n=== Total Truss Mass and Cost ===\n');
fprintf('Total mass = %.2f kg\n', PDans.mass);
fprintf('Total cost = $%.2f\n', PDans.cost);

%% --- DEFLECTION CHECK ---

fprintf('\n=== Node Deflection Check (Drone Docking Points) ===\n');
fprintf('Node\tUx (m)\tUy (m)\tPass/Fail\n');

max_deflection_allowed = 0.10; % 10 cm

dock_nodes = [3, 4, 5, 6]; % Node numbers for docking points (Nodes 1-4)

for i = 1:length(dock_nodes)
    node = dock_nodes(i);
    
    Ux = PDans.U(node,1);
    Uy = PDans.U(node,2);
    
    if abs(Ux) <= max_deflection_allowed && abs(Uy) <= max_deflection_allowed
        status = 'Pass';
    else
        status = 'Fail';
    end

    fprintf('%d\t%.4f\t%.4f\t%s\n', node, Ux, Uy, status);
end
