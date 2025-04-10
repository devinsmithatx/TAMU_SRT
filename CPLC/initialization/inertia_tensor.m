function I = inertia_tensor(body)
% INERTIA_TENSOR.M CREATES INERTIA TENSOR FOR THE VEHICLE
    % I = [ Ixx -Ixy -Ixz; -Ixy  Iyy -Iyz; -Ixz -Iyz  Izz];

% USER INPUT
Ixx = body.Ixx;
Iyy = body.Iyy;
Izz = body.Izz;
Ixy = body.Ixy;
Iyz = body.Iyz;
Ixz = body.Ixz;

% INERTIA TENSOR
I = [ Ixx -Ixy -Ixz
     -Ixy  Iyy -Iyz  
     -Ixz -Iyz  Izz ];
end

