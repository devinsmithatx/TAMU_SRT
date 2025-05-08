function [actuator_upper, actuator_lower] = actuator_limits(act)
% ACTUATOR_LIMITS.M OUTPUTS ACTUATOR LIMITS AS VECTORS
%   actuator_upper = [engine_max rcs_1_max rcs_2_max ... rcs_m_max]
%   actuator_lower = [engine_min rcs_1_min rcs_2_min ... rcs_m_min]

% USER INPUT
engine_max = act.engine_max;
rcs_max = act.rcs_max;

% DIMENSIONS
m = length(act.rcs) + 1;                                % number actuators

% UPPER LIMITS
actuator_upper = [engine_max; ones(m-1,1)*rcs_max];
actuator_upper(1) = engine_max - act.u_bar(1);

% LOWER LIMITS
actuator_lower = -actuator_upper;             % rcs min
actuator_lower(1) = -act.u_bar(1);
end

