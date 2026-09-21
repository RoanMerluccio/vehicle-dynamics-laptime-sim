function [Fy, mu_eff] = calculatePacejkaTire(Fz, alpha_deg, coeffs)
%CALCULATEPACEJKATIRE Pacejka Magic Formula 94 lateral tire force model.
%
%   [Fy, mu_eff] = calculatePacejkaTire(Fz, alpha_deg, coeffs)
%
%   Implements the Pacejka 94 "Magic Formula" for lateral (cornering) force:
%       Fy = D * sin(C * atan(B*alpha - E*(B*alpha - atan(B*alpha))))
%
%   The peak lateral friction coefficient mu_eff = Fy_peak / Fz is returned
%   for use as a drop-in replacement for the linear friction coefficient mu
%   in the friction-circle solver.
%
%   Inputs:
%     Fz        - Normal (vertical) load on tire, scalar [N]
%     alpha_deg - Slip angle vector [degrees] to evaluate; if empty, the
%                 function returns mu_eff at peak only (no Fy curve).
%     coeffs    - Struct with Pacejka shape parameters:
%                   .B  - Stiffness factor         (default 10.0)
%                   .C  - Shape factor             (default 1.9 )
%                   .D  - Peak factor = peak Fy/Fz (default 1.6 )
%                   .E  - Curvature factor         (default 0.97)
%
%   Defaults match a typical FSAE dry-slick 13" tire (Hoosier R25B).
%   Real tire coefficients can be fitted from TTC (Tire Testing Consortium) data.
%
%   References:
%     Pacejka, H.B. (2002). Tyre and Vehicle Dynamics. Butterworth-Heinemann.
%     SAE 940198: Magic Formula Tyre Model.
%
%   Usage examples:
%     % Peak friction coefficient only (for use in calculateVehicleForces):
%     [~, mu] = calculatePacejkaTire(Fz, [], []);
%
%     % Full Fy(alpha) curve (for plotting):
%     alpha = -20:0.1:20;
%     [Fy, ~] = calculatePacejkaTire(2500, alpha, []);

%% Default coefficients: FSAE dry slick (Hoosier 20.5x7.0-13 R25B typical)
if nargin < 3 || isempty(coeffs)
    coeffs = struct();
end
if ~isfield(coeffs, 'B'); coeffs.B = 10.0; end
if ~isfield(coeffs, 'C'); coeffs.C = 1.9;  end
if ~isfield(coeffs, 'D'); coeffs.D = 1.6;  end
if ~isfield(coeffs, 'E'); coeffs.E = 0.97; end

%% Load sensitivity: peak friction decreases slightly at high vertical load
%   D_eff = D * (1 - 0.06 * (Fz - Fz_ref) / 1000)
%   This approximates the degressive load-sensitivity seen in real tire data.
Fz_ref = 1500;   % Reference corner weight [N] (~340 N per corner for FSAE car)
D_eff  = coeffs.D * (1.0 - 0.06 * (Fz - Fz_ref) / 1000.0);
D_eff  = max(D_eff, 0.5);

%% Magic Formula evaluation
B = coeffs.B;
C = coeffs.C;
D = D_eff;
E = coeffs.E;

if isempty(alpha_deg)
    % Compute only the peak friction coefficient (no full curve needed)
    % Evaluate on a dense grid and take peak
    alpha_dense = linspace(-20, 20, 500);
    alpha_rad   = deg2rad(alpha_dense);
    phi         = B * alpha_rad - E * (B * alpha_rad - atan(B * alpha_rad));
    Fy_curve    = Fz * D * sin(C * atan(phi));
    mu_eff      = max(abs(Fy_curve)) / Fz;
    Fy          = [];
else
    % Full Fy(alpha) curve
    alpha_rad = deg2rad(alpha_deg(:));
    phi       = B * alpha_rad - E * (B * alpha_rad - atan(B * alpha_rad));
    Fy        = Fz * D * sin(C * atan(phi));
    mu_eff    = max(abs(Fy)) / Fz;
end
end
