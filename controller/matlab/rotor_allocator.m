function [omega, wrenchAchieved, saturated] = rotor_allocator(wrenchRequested)
%ROTOR_ALLOCATOR Allocate thrust and moments to four rotor-speed commands.
%
% Input:
%   wrenchRequested = [T; L; M; N]
%       T: total thrust [N]
%       L: roll moment [N*m]
%       M: pitch moment [N*m]
%       N: yaw moment [N*m]
%
% Outputs:
%   omega           : 4x1 rotor-speed commands [rad/s]
%   wrenchAchieved  : achieved wrench [T; L; M; N]
%   saturated       : true if the unconstrained allocation exceeds limits

m = 1.2;
g = 9.80665;
kT = 8.0e-6;
kQ = 1.5e-7;
d = 0.141421356;
wMax = 1200;

if ~isnumeric(wrenchRequested) || ...
        numel(wrenchRequested) ~= 4 || ...
        any(~isfinite(wrenchRequested(:)))
    error('wrenchRequested must contain four finite numeric values.');
end

wrenchRequested = wrenchRequested(:);

if wrenchRequested(1) < 0
    error('Requested total thrust cannot be negative.');
end

A = [kT,    kT,    kT,    kT;
    d*kT, -d*kT, -d*kT,  d*kT;
    d*kT,  d*kT, -d*kT, -d*kT;
    kQ,   -kQ,    kQ,   -kQ];

% Allocate in squared rotor speeds.
s = A \ wrenchRequested;

% Flag infeasible squared speeds before limiting.
saturated = any(s < 0) || any(s > wMax^2);

% Enforce actuator bounds.
s = min(max(s, 0), wMax^2);
omega = sqrt(s);

% Report what the constrained commands actually produce.
wrenchAchieved = A * s;
end