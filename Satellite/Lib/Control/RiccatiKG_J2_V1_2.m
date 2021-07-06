function KGp = RiccatiKG_J2_V1_2(tau, KGvet, params)

%   RiccatiKG_J2_V1_2.m
%   Made by Sapienza Gn Lab
%
%   Integrates Riccati Differential Equation for the Augmented LQR control strategy and returns control matrices K (feedback) and G (feedforward).
%   Here the system's state matrix takes into account linearized J2 effects.
%
%   INPUT
%   tau: integration time vector
%   KGvet: feedback (K) and feedforward (G) gain in vector form to be integrated
%   params: structure containing the following fields
%       - params.R = LQR control gain matrix
%       - params.Q = LQR state gain matrix
%       - params.B = control mapping matrix
%       - params.tfinalRiccati = Riccati differential equation integration final time [s]
%       - params.chief_OOE = array containing chief's osculating orbital elements
%       - params.ReferenceTrajectory = structure containing all the parameters required for correct reference trajectory computation
%       - params.ReferenceTrajectory.TrajectoryProfile = handle to the function which computes the reference trajectory at time "t"
%
%   OUTPUT
%   KGp: Control gain derivatives
%
%   VERSION
%   20190326 V1_1:
%   - First release
%
%   20190405 V1_2:
%   - Modification in reference trajectory computation procedure

% Extraction of needed quantities from "params" structure
R = params.R;
Q = params.Q;
B = params.B;
tfinalRIC = params.tfinalRiccati;
chief_OOE = params.chief_OOE;


% Extraction of K matrix and reshaping into column vector
K = [KGvet(1:6)'; KGvet(7:12)'; KGvet(13:18)'; KGvet(19:24)'; KGvet(25:30)'; KGvet(31:36)']';

% Reverting time to be backwards, as Riccati diferential equation is given to final conditions ("tau" is used to make the problem "as if" it was with
% given initial conditions)
t = tfinalRIC - tau;

% State matrix including linearized J2 effects
A = AJ2_V1_1(t, chief_OOE, params);
   
% Feedback Gain derivatives
KPRIME = -K*B*inv(R)*B'*K + K*A + A'*K + Q;

Kp(1:36,1) = 0;
i = 1;

for h = 1:6
    
    for j = 1:6
        
        Kp(i,1) = KPRIME(j,h);
        i = i+1;
        
    end
    
end

% Reference Trajectory computation at time t
x_ref = params.ReferenceTrajectory.TrajectoryProfile(t, params.ReferenceTrajectory);

% Feedforward Gain derivatives
Gp = -(K*B*inv(R)*B' - A')*KGvet(37:42) + Q*x_ref;

% Allocation of gain matrices derivatives
KGp = [Kp; Gp];

end