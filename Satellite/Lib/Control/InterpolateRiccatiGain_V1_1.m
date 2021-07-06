function [Kc, Gc] = InterpolateRiccatiGain_V1_1(t, Ndeputy, params)

%   InterpolateRiccatiGain_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the gain matrices Kc and Gc at the desired instant "t" by interpolating the gain matrices solution of the riccati differential equation.
%
%   INPUT
%   t: time instant in which the control gain matrices must be evaluated
%   Ndeputy : Number of deputy satellites
%   params: structure containing the following fields
%       - params.tRiccati = Riccati differential equation integration time step [s]
%       - params.K = feedback gain matrix obtained as output of the Ricati differrential equation integration
%       - params.G = feedforward gain matrix obtained as output of the Ricati differrential equation integration
%
%   OUTPUT
%   Kc : Array (6 x 6 x N) containing the feedback control gain matrix at time "t" for each deputy. N is the number of deputies.
%   Gc : Array (6 x N) containing the feedforward control term at time "t" for each deputy. N is the number of deputies.
%
%   VERSION
%   20190412 V1_1:
%   -  First Release

% Compute the previous and successive indices
RiccatiStep = params.tRiccati;
PrevIndex = floor(t/RiccatiStep) + 1;
SucIndex = floor(t/RiccatiStep) + 2;

% Extract gain matrices from the "params" structure (they don't need to be calculated at each time step)
K = params.K;
G = params.G;

% Initialize output arrays
Kc = zeros(6, 6, Ndeputy);
Gc = zeros(6, Ndeputy);

% Interpolation process over time for each deputy
for j = 1:Ndeputy
    
    if (SucIndex < max(size(K)))
        
        t_prec = (PrevIndex - 1)*RiccatiStep;
        t_suc = (SucIndex - 1)*RiccatiStep;
        Kvet = K(PrevIndex,:,j);
        K_prec = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
        Kvet = K(SucIndex,:,j);
        K_suc = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
        Kc(:,:,j) = (K_suc - K_prec)/(t_suc - t_prec)*t + (K_prec*t_suc - K_suc*t_prec)/(t_suc - t_prec);
        G_prec = G(PrevIndex,:,j)';
        G_suc = G(SucIndex,:,j)';
        Gc(:,j) = (G_suc - G_prec)/(t_suc - t_prec)*t + (G_prec*t_suc - G_suc*t_prec)/(t_suc - t_prec);
        
    else
        
        Kvet = K(PrevIndex,:,j);
        Kc(:,:,j) = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
        Gc(:,j) = G(PrevIndex,:,j)';
        
    end
    
end

end