function z = generateSpaElevation(s)
%GENERATESPAELEVATION Generate realistic elevation profile for Spa-Francorchamps.

if nargin < 1 || isempty(s)
    error('Input track distance vector s is required.');
end

L = max(s);
if L <= 0
    z = zeros(size(s));
    return;
end

% Key landmarks along the ~7004 m lap (normalised station s/L and altitude [m])
% Smooth periodic profile ensuring start and finish elevations match
stations_norm = [ ...
    0.000; ... % Start/Finish line
    0.040; ... % Turn 1 (La Source hairpin)
    0.085; ... % Downhill approach to Eau Rouge
    0.115; ... % Bottom of Eau Rouge dip
    0.145; ... % Raidillon crest
    0.200; ... % Kemmel straight climb
    0.260; ... % Turn 5 & 6 (Les Combes chicane)
    0.285; ... % Turn 7 (Malmedy - highest point)
    0.340; ... % Turn 8 (Bruxelles hairpin descent)
    0.390; ... % Turn 9 (Speaker's Corner)
    0.450; ... % Turn 10 & 11 (Pouhon double-left)
    0.530; ... % Turn 12 & 13 (Fagnes chicane)
    0.600; ... % Turn 14 & 15 (Campus / Stavelot)
    0.680; ... % Turn 16 (Courbe Paul Frère)
    0.780; ... % Turn 17 (Blanchimont sweeping left)
    0.880; ... % Approach to Bus Stop
    0.950; ... % Turn 18 & 19 (Bus Stop chicane)
    1.000  ... % Back to Start/Finish line
    ];

altitudes_m = [ ...
    395.0; ... % Start/Finish
    398.0; ... % La Source
    384.0; ... % Eau Rouge approach
    372.0; ... % Eau Rouge bottom (dip)
    414.0; ... % Raidillon crest (+42 m climb)
    448.0; ... % Kemmel straight
    468.0; ... % Les Combes
    473.2; ... % Malmedy peak (+101.2 m above lowest)
    446.0; ... % Bruxelles
    428.0; ... % Speaker's Corner
    392.0; ... % Pouhon
    386.0; ... % Fagnes
    376.0; ... % Stavelot
    373.5; ... % Paul Frere
    378.0; ... % Blanchimont
    386.0; ... % Bus Stop approach
    392.0; ... % Bus Stop chicane
    395.0   ... % Start/Finish (matches start for seamless loop)
    ];

% Interpolate using shape-preserving piecewise cubic Hermite (pchip)
s_norm = s ./ L;
z = pchip(stations_norm, altitudes_m, s_norm);
end
