%% SDPSO is based on standard PSO and a paper: https://ieeexplore.ieee.org/abstract/document/9795684
function [vxn1, vyn1, vxn2, vyn2] = SDPSO(xc1,yc1,xs1,ys1,xg1,yg1,l1_max,vx1,vy1,h1,h1_old,xc2,yc2,xs2,ys2,xg2,yg2,l2_max,vx2,vy2,h2,h2_old,ds)
%% Problem Definition
nVar = 4;             % Number of Decision Variables, (x1, y1, x2, y2)
VarSize = [1 nVar];   % Size of Decision Variables Matrix
D = 3;                % Dimension (Search space)
VarMin = -D;          % Lower Bound of Variables (iteration lower range)
VarMax =  D;          % Upper Bound of Variables (iteration upper range)

%% SDPSO Parameters
MaxIt=100;       % Maximum Number of Iterations
nPop_max=150;    % Population Size (Swarm Size), # of particles
nPop_min=100;     % Population Size (Swarm Size), # of particles
w=1;             % Inertia Weight
w1=2;          % relative weight factor of cost funtion 1
w2=1;          % relative weight factor of cost funtion 2
w3=1;          % relative weight factor of cost funtion 3
wdamp=0.99;      % Inertia Weight Damping Ratio
c1=1.5;          % Personal Learning Coefficient
c2=2.0;          % Global Learning Coefficient
pre_GlobalBest = 0;

% simulated annealing algorithm (SA)
p = 0;           % initial probability of a suboptimal solution being accepted
T = 90; % temperature of current system (T=1)

% Acceleration of Convergence based on DLS
count = 0;        % nonupdating number
m = 2;          % Random threshold fo the acceleration of convergence

% Velocity Limits
% VelMax=0.1*(VarMax-VarMin);
VelMax=3;
VelMin=-VelMax;

% j=0; % check the number of local update 
% z=0; % check the number of global update

%% Initialization
empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
particle=repmat(empty_particle,nPop_max,1);
GlobalBest.Cost=inf;

for i=1:nPop_max
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize); % 1*4
    
    % Initialize Velocity
    particle(i).Velocity=unifrnd(VelMin,VelMax,VarSize); % 1*4
    
    % Evaluation 
    ax1 = particle(i).Position(1,1); % body frame
    ay1 = particle(i).Position(1,2); % body frame
    ax2 = particle(i).Position(1,3); % body frame
    ay2 = particle(i).Position(1,4); % body frame
    vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
    vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
    vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
    vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
    wvx1 = cos(h1)*vxn1-sin(h1)*vyn1; wvy1 = sin(h1)*vxn1+cos(h1)*vyn1;
    wvx2 = cos(h2)*vxn2-sin(h2)*vyn2; wvy2 = sin(h2)*vxn2+cos(h2)*vyn2;
    xn1 = xc1 + wvx1; yn1 = yc1 + wvy1; % world frame
    xn2 = xc2 + wvx2; yn2 = yc2 + wvy2; % world frame
    dn1 = ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5);
    dn2 = ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);
    F1 = OF1(dn1,dn2,l1_max,l2_max);
    F2 = OF2(xn1,yn1,xn2,yn2,xs1,ys1,xs2,ys2,ds);
    F3 = OF3(h1, h1_old, h2, h2_old);
    particle(i).Cost = w1*F1 + w2*F2 + w3*F3;
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
  
        GlobalBest=particle(i).Best;
        
    end
    
    
end

%% SDPSO Main Loop
if (((xc1 - xc2)^2 + (yc1 - yc2)^2)^(0.5)) < ds
    for it=1:MaxIt
        
        for i=1:nPop_max
            
            % Update Velocity
            particle(i).Velocity = w*particle(i).Velocity ...
                +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
                +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
            
            % Apply Velocity Limits
            particle(i).Velocity = max(particle(i).Velocity,VelMin);
            particle(i).Velocity = min(particle(i).Velocity,VelMax);
            
            % Update Position
            particle(i).Position = particle(i).Position + particle(i).Velocity;
            
%             % Velocity Mirror Effect
%             IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
%             particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
            
%             % Apply Position Limits
%             particle(i).Position = max(particle(i).Position,VarMin);
%             particle(i).Position = min(particle(i).Position,VarMax);
            
            % Evaluation
            ax1 = particle(i).Position(1,1);
            ay1 = particle(i).Position(1,2);
            ax2 = particle(i).Position(1,3);
            ay2 = particle(i).Position(1,4);
            vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
            vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
            vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
            vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
            wvx1 = cos(h1)*vxn1-sin(h1)*vyn1; wvy1 = sin(h1)*vxn1+cos(h1)*vyn1;
            wvx2 = cos(h2)*vxn2-sin(h2)*vyn2; wvy2 = sin(h2)*vxn2+cos(h2)*vyn2;
            xn1 = xc1 + wvx1; yn1 = yc1 + wvy1;
            xn2 = xc2 + wvx2; yn2 = yc2 + wvy2;
            dn1 = ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5);
            dn2 = ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);
            F1 = OF1(dn1,dn2,l1_max,l2_max);
            F2 = OF2(xn1,yn1,xn2,yn2,xs1,ys1,xs2,ys2,ds);
            F3 = OF3(h1, h1_old, h2, h2_old);
            particle(i).Cost = w1*F1 + w2*F2 + w3*F3;
    
            % Update Personal Best
            if count > m
                if particle(i).Cost < particle(i).Best.Cost
                    
                    particle(i).Best.Position = particle(i).Position;
                    particle(i).Best.Cost = particle(i).Cost;
                
                end
                count = 0;
            end
            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                if p > rand
                    
                    pre_GlobalBest = GlobalBest.Cost;
                    GlobalBest = particle(i).Best;
                    
                end
            end
        end
        
        % Update parameters
        if it > 1
            count = count + 1;
            w=w*wdamp;
            r = (MaxIt - it)/MaxIt;
            T = T * r;
            p = exp(-(GlobalBest.Cost - pre_GlobalBest)/T);
%             disp(['Global best: ' num2str(GlobalBest.Cost) ' previous Global best: ' num2str(pre_GlobalBest) ' p = ' num2str(p)])
            m = (it/MaxIt)*(VarMax - VarMin);
        end
    end
else
    for it=1:MaxIt
        
        for i=1:nPop_min
            
            % Update Velocity
            particle(i).Velocity = w*particle(i).Velocity ...
                +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
                +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
            
            % Apply Velocity Limits
            particle(i).Velocity = max(particle(i).Velocity,VelMin);
            particle(i).Velocity = min(particle(i).Velocity,VelMax);
            
            % Update Position
            particle(i).Position = particle(i).Position + particle(i).Velocity;
            
%             % Velocity Mirror Effect
%             IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
%             particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
%             
%             % Apply Position Limits
%             particle(i).Position = max(particle(i).Position,VarMin);
%             particle(i).Position = min(particle(i).Position,VarMax);
            
            % Evaluation
            ax1 = particle(i).Position(1,1);
            ay1 = particle(i).Position(1,2);
            ax2 = particle(i).Position(1,3);
            ay2 = particle(i).Position(1,4);
            vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
            vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
            vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
            vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
            wvx1 = cos(h1)*vxn1-sin(h1)*vyn1; wvy1 = sin(h1)*vxn1+cos(h1)*vyn1;
            wvx2 = cos(h2)*vxn2-sin(h2)*vyn2; wvy2 = sin(h2)*vxn2+cos(h2)*vyn2;
            xn1 = xc1 + wvx1; yn1 = yc1 + wvy1;
            xn2 = xc2 + wvx2; yn2 = yc2 + wvy2;
            dn1 = ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5);
            dn2 = ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);
            F1 = OF1(dn1,dn2,l1_max,l2_max);
            F2 = OF2(xn1,yn1,xn2,yn2,xs1,ys1,xs2,ys2,ds);
            F3 = OF3(h1, h1_old, h2, h2_old);
            particle(i).Cost = w1*F1 + w2*F2 + w3*F3;
    
            % Update Personal Best
            if count > m
                if particle(i).Cost < particle(i).Best.Cost
                    
                    particle(i).Best.Position = particle(i).Position;
                    particle(i).Best.Cost = particle(i).Cost;

                        % Update Global Best
                        if particle(i).Best.Cost < GlobalBest.Cost
                          
                            GlobalBest = particle(i).Best;

                        end
                end
                count = 0;
            end
        end
        
        % Update parameters
        if it > 1
            count = count + 1;
            w=w*wdamp;
            m = (it/MaxIt)*(VarMax - VarMin);
        end
    end
end

ax1 = GlobalBest.Position(1,1);
ay1 = GlobalBest.Position(1,2);
ax2 = GlobalBest.Position(1,3);
ay2 = GlobalBest.Position(1,4);
vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);

end