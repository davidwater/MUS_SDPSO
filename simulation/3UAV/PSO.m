function [vxn1, vyn1, vxn2, vyn2, vxn3, vyn3] = PSO(xc1,yc1,xs1,ys1,xg1,yg1,l1_max,vx1,vy1,h1,h1_old, ...
            xc2,yc2,xs2,ys2,xg2,yg2,l2_max,vx2,vy2,h2,h2_old, ...
            xc3,yc3,xs3,ys3,xg3,yg3,l3_max,vx3,vy3,h3,h3_old, ...
            ds)
%% Problem Definition
nVar = 6;             % Number of Decision Variables
VarSize = [1 nVar];   % Size of Decision Variables Matrix
D = 3;               % Dimension (Search space)
VarMin = -D;          % Lower Bound of Variables (iteration lower range)
VarMax =  D;          % Upper Bound of Variables (iteration upper range)

%% PSO Parameters
MaxIt=100;       % Maximum Number of Iterations
nPop=100;        % Population Size (Swarm Size)
w=1;             % Inertia Weight
wdamp=0.99;      % Inertia Weight Damping Ratio
c1=1.5;          % Personal Learning Coefficient
c2=2.0;          % Global Learning Coefficient
w1=0.33;         % relative weight factor of cost funtion 1
w2=0.33;         % relative weight factor of cost funtion 2
w3=0.33;         % relative weight factor of cost funtion 3

% Velocity Limits
%VelMax=0.1*(VarMax-VarMin);
VelMax = 3;
VelMin=-VelMax;

%% Initialization
empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
particle=repmat(empty_particle,nPop,1);
GlobalBest.Cost=inf;

for i=1:nPop
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize);
    
    % Initialize Velocity
    particle(i).Velocity=unifrnd(VelMin,VelMax,VarSize);
    
    % Evaluation
    ax1 = particle(i).Position(1,1);
    ay1 = particle(i).Position(1,2);
    ax2 = particle(i).Position(1,3);
    ay2 = particle(i).Position(1,4);
    ax3 = particle(i).Position(1,5);
    ay3 = particle(i).Position(1,6);
    vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
    vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
    vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
    vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
    vxn3 = vx3 + ax3; vxn3 = max(vxn3,-3); vxn3 = min(vxn3,3);
    vyn3 = vy3 + ay3; vyn3 = max(vyn3,-3); vyn3 = min(vyn3,3);
    wvx1 = cos(h1)*vxn1+sin(h1)*vyn1; wvy1 = sin(h1)*vxn1-cos(h1)*vyn1;
    wvx2 = cos(h2)*vxn2+sin(h2)*vyn2; wvy2 = sin(h2)*vxn2-cos(h2)*vyn2;
    wvx3 = cos(h3)*vxn3+sin(h3)*vyn3; wvy3 = sin(h3)*vxn3-cos(h3)*vyn3;
    xn1 = xc1 + wvx1; yn1 = yc1 + wvy1;
    xn2 = xc2 + wvx2; yn2 = yc2 + wvy2;
    xn3 = xc3 + wvx3; yn3 = yc3 + wvy3;
    dn1 = ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5);
    dn2 = ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);
    dn3 = ((xn3 - xg3)^2 + (yn3 - yg3)^2)^(0.5);
    F1 = OF1(dn1,dn2,dn3,l1_max,l2_max,l3_max);
    F2 = OF2(xn1,yn1,xn2,yn2,xn3,yn3,xs1,ys1,xs2,ys2,xs3,ys3,ds);
    F3 = OF3(h1, h1_old, h2, h2_old, h3, h3_old);
    particle(i).Cost = w1*F1 + w2*F2 + w3*F3;
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        
    end
    
end

%% PSO Main Loop
for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Velocity Mirror Effect
        IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
        particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Evaluation
        ax1 = particle(i).Position(1,1);
        ay1 = particle(i).Position(1,2);
        ax2 = particle(i).Position(1,3);
        ay2 = particle(i).Position(1,4);
        ax3 = particle(i).Position(1,5);
        ay3 = particle(i).Position(1,6);
        vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
        vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
        vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
        vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
        vxn3 = vx3 + ax3; vxn3 = max(vxn3,-3); vxn3 = min(vxn3,3);
        vyn3 = vy3 + ay3; vyn3 = max(vyn3,-3); vyn3 = min(vyn3,3);
        wvx1 = cos(h1)*vxn1+sin(h1)*vyn1; wvy1 = sin(h1)*vxn1-cos(h1)*vyn1;
        wvx2 = cos(h2)*vxn2+sin(h2)*vyn2; wvy2 = sin(h2)*vxn2-cos(h2)*vyn2;
        wvx3 = cos(h3)*vxn3+sin(h3)*vyn3; wvy3 = sin(h3)*vxn3-cos(h3)*vyn3;
        xn1 = xc1 + wvx1; yn1 = yc1 + wvy1;
        xn2 = xc2 + wvx2; yn2 = yc2 + wvy2;
        xn3 = xc3 + wvx3; yn3 = yc3 + wvy3;
        dn1 = ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5);
        dn2 = ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);
        dn3 = ((xn3 - xg3)^2 + (yn3 - yg3)^2)^(0.5);
        F1 = OF1(dn1,dn2,dn3,l1_max,l2_max,l3_max);
        F2 = OF2(xn1,yn1,xn2,yn2,xn3,yn3,xs1,ys1,xs2,ys2,xs3,ys3,ds);
        F3 = OF3(h1, h1_old, h2, h2_old, h3, h3_old);
        particle(i).Cost = w1*F1 + w2*F2 + w3*F3;
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest=particle(i).Best;
                
            end
            
        end
        
    end
    
    w=w*wdamp;
    
end

ax1 = GlobalBest.Position(1,1);
ay1 = GlobalBest.Position(1,2);
ax2 = GlobalBest.Position(1,3);
ay2 = GlobalBest.Position(1,4);
ax3 = GlobalBest.Position(1,5);
ay3 = GlobalBest.Position(1,6);
vxn1 = vx1 + ax1; vxn1 = max(vxn1,-3); vxn1 = min(vxn1,3);
vyn1 = vy1 + ay1; vyn1 = max(vyn1,-3); vyn1 = min(vyn1,3);
vxn2 = vx2 + ax2; vxn2 = max(vxn2,-3); vxn2 = min(vxn2,3);
vyn2 = vy2 + ay2; vyn2 = max(vyn2,-3); vyn2 = min(vyn2,3);
vxn3 = vx3 + ax3; vxn3 = max(vxn3,-3); vxn3 = min(vxn3,3);
vyn3 = vy3 + ay3; vyn3 = max(vyn3,-3); vyn3 = min(vyn3,3);

end