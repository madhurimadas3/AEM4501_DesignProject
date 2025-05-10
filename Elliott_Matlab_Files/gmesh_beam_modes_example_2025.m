E = 200e9;
rho = 8050;
L = 2;
b = 10/100;
h = 15/100;
A = b*h;
Ix = (1/12)*b*h^3;

ZmodePos = cell(2,1);
Modez = cell(2,1);
Freqz = cell(2,1);
n = [5 20];

for k = 1:1
        N = n(k);
        NodePos = linspace(0, L, N)';
        NE = N-1;
        ElmConnect = zeros(NE,2);
        for i = 1:NE
            ElmConnect(i,:) = [i, i+1];
        end
        
        E_matrix = E*ones(NE,1);
        Ix_matrix = Ix*ones(NE,1);
        rho_matrix = rho*A*ones(NE,1);
        
        LEndNode = 1;
        REndNode = N;
        
        Nmodes = 1;
        
        [Freq, RawModes, M, K, InterpModes, Zpos] = gmesh_beam_modes(N,...
                NodePos, NE, ElmConnect, E_matrix, Ix_matrix,rho_matrix,...
                LEndNode,REndNode,Nmodes);
            
            %Freqz{k,1}=Freq;
            %ZModePos{k,1} = Zpos;
            %Modez{k,1} = InterpModes;
end

fprintf('Natural frequencies with 5 nodes are:\n')
%disp(Freqz{1,1});Zm
disp(Freq);
%fprintf('Natural frequencies with 20 nodes are:\n')
%disp(Freqz{2,1});

%figure(2)
%plot(ZModePos{2,1}, Modez{2,1}, 'LineWidth',2 );
%xlabel('Z postion (m)');
%ylabel('Mode Shape');
%title('First 3 Mode Shapes using 20 Nodes');
%legend('3rd NF', '2nd NF', '1st NF', 'Location', 'Best');
        