function plot_simout(simout, act, misc)
% SIM_PLOT.M PLOTS THE OUTPUTS FROM THE SIMULINK SIM
%   plots states and control effort from linear and nonlinear models

% EXTRACT DATA
t    = simout.tout;           % continuous time values
tk1  = simout.u.Time;         % controller time values
tk2  = simout.y.Time;         % measurement time values
tk3  = simout.x_cmd.Time;       % guidance time values
x_ref  = simout.x_cmd.Data;   % inner loop reference data
x    = simout.x.Data;         % x data
y    = simout.y.Data;         % y data
u    = simout.u.Data';        % u_cmd data
xh   = simout.xh.Data;        % x_plus data
p    = simout.p.Data;         % position data
if strcmp(misc.type,"outer")
    cmd  = simout.p_cmd.Data;     % trajectory command data
    p_ref  = simout.p_ref.Data;   % trajectory reference data
    ph  = simout.ph.Data;         % position estimate
end

% CONVERT FROM RAD TO DEG
x(1:3,:) = x(1:3,:)*180/pi;
x(7:9,:) = x(7:9,:)*180/pi;
y(1:3,:) = y(1:3,:)*180/pi;
y(7:9,:) = y(7:9,:)*180/pi;
xh(1:3,:) = xh(1:3,:)*180/pi;
xh(7:9,:) = xh(7:9,:)*180/pi;

% PLOT TITLES 
states = {'$\phi$', '$\theta$', '$\psi$', ...
          '$u$',   '$v$',     '$w$',   ...
          '$p$',   '$q$' ,    '$r$'};
rcs = act.rcs;

% PLOT STATES
figure(); clf;
tiledlayout(3,3, 'Padding', 'none', 'TileSpacing', 'compact')
for i = 1:9
    nexttile; hold on;
    line1 = stairs(tk2,y(i,:),'color','#80B3FF','linewidth',1,'DisplayName','sensed');
    line2 = stairs(tk2,xh(i,:),'r','linewidth',1,'DisplayName','estimate');
    line3 = plot(t,x(i,:),'b','linewidth',1,'DisplayName','true');
    line4 = stairs(tk3,x_ref(i,:),'k--','Linewidth',1,'DisplayName','command');
    title(states(i), Interpreter="latex");  
    xlabel("t (s)"); 
    if ismember(i, [1 2 3])
        ylabel("Angle (deg)");
    elseif ismember(i, [4 5 6])
        ylabel("Velocity (m/s)");
    else
        ylabel("Angular Velocity (deg/s)");
    end
end
hl = legend([line1 line2 line3 line4], 'NumColumns',4);
hl.Layout.Tile = 'south';

% PLOT ENGINE USAGE
figure(); clf;
tiledlayout(1, 1, 'Padding', 'none', 'TileSpacing', 'compact')
nexttile; hold on;
line1 = stairs(tk1,u(1,:),'color','#80B3FF','linewidth',1,'DisplayName','command');
line2 = stairs(tk2,xh(10,:),'r','linewidth',1,'DisplayName','estimate');
line3 = plot(t,x(10,:),'b','linewidth',1,'DisplayName','true');
title("Engine Usage"); 
legend(Location="best");
xlabel("t (s)"); 
ylabel("Thrust (N)");
hl = legend([line1 line2 line3], 'NumColumns',3);
hl.Layout.Tile = 'south';

% PLOT RCS THRUSTER USAGE
figure(); clf;
tiledlayout(length(rcs), 1, 'Padding', 'none', 'TileSpacing', 'compact')
for i = 2:height(u)
    nexttile; hold on;
    line1 = stairs(tk1,u(i,:),'color','#80B3FF','linewidth',1,'DisplayName','command');
    line2 = stairs(tk2,xh(9+i,:),'r','linewidth',1,'DisplayName','estimate');
    line3 = plot(t,x(9+i,:),'b','linewidth',1,'DisplayName','true');
    title(strcat("RCS Pair ", num2str(i - 1)));
    xlabel("t (s)"); 
    ylabel("Thrust (N)");
end
hl = legend([line1 line2 line3], 'NumColumns',3);
hl.Layout.Tile = 'south';

% PLOT ENGINE USAGE PERCENT
x(10, :) = x(10,:)/act.engine_max*100;          % convert to percent
xh(10, :) = xh(10,:)/act.engine_max*100;          % convert to percent
u(1,:) = u(1,:)/act.engine_max*100;             % convert to percent

figure(); clf;
tiledlayout(1, 1, 'Padding', 'none', 'TileSpacing', 'compact')
nexttile; hold on;
line1 = stairs(tk1,u(1,:),'color','#80B3FF','linewidth',1,'DisplayName','command');
line2 = stairs(tk2,xh(10,:),'r','linewidth',1,'DisplayName','estimate');
line3 = plot(t,x(10,:),'b','linewidth',1,'DisplayName','true');
title("Engine Usage"); 
legend(Location="best");
xlabel("t (s)"); 
ylabel("Thrust (% Max Thrust)");
hl = legend([line1 line2 line3], 'NumColumns',3);
hl.Layout.Tile = 'south';

% PLOT RCS THRUSTER USAGE PERCENT
x= x/act.rcs_max*100;                           % convert to percent
xh= xh/act.rcs_max*100;                           % convert to percent
u = u/act.rcs_max*100;                          % convert to percent

figure(); clf;
tiledlayout(length(rcs), 1, 'Padding', 'none', 'TileSpacing', 'compact')
for i = 2:height(u)
    nexttile; hold on;
    line1 = stairs(tk1,u(i,:),'color','#80B3FF','linewidth',1,'DisplayName','command');
    line2 = stairs(tk2,xh(9+i,:),'r','linewidth',1,'DisplayName','estimate');
    line3 = plot(t,x(9+i,:),'b','linewidth',1,'DisplayName','true');
    title(strcat("RCS Pair ", num2str(i - 1)));
    xlabel("t (s)"); 
    ylabel("Thrust (% Max Thrust)");
end
hl = legend([line1 line2 line3], 'NumColumns',3);
hl.Layout.Tile = 'south';

% PLOT POSITION DATA
positions = 'xyz';
figure(); clf;
tiledlayout(3, 1, 'Padding', 'none', 'TileSpacing', 'compact')
for i = 1:3
    nexttile; hold on;
    if strcmp(misc.type, "outer")
        line1 = stairs(tk3,ph(i,:),'r','linewidth',1,'DisplayName','estimate');
        line2 = stairs(tk3,cmd(i,:),'k--','linewidth',1,'DisplayName','command');
        line3 = stairs(tk3,p_ref(:,i),'k-','linewidth',1,'DisplayName','reference');
    end
    line4 = plot(t,p(i,:),'b','linewidth',1,'DisplayName','true');
    title(['$r_' positions(i) '$'], Interpreter="latex");
    xlabel("t (s)"); 
    ylabel("Position (m)");
end
hl = legend([line1 line2 line3 line4], 'NumColumns',4);
hl.Layout.Tile = 'south';
end