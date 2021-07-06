function q = ConvertQuat_V2_1(q0, MODE)

%   ConvertQuat_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Swaps the position of the scalar component in a quaternion vector.
%
%   INPUT
%   q0: Input 1x4 quaternion vector
%   MODE: string selection either one between two modes:
%         - 'ScalarTo1': Gives as output a quaternion having the scalar component placed in the first component
%         - 'ScalarTo4': Gives as output a quaternion having the scalar component placed in the fourth component
%
%   OUTPUT
%   q: Output 1x4 quaternion vector
%
%   VERSION
%   20200519 V2_1:
%   - First Release

if strcmpi(MODE, 'ScalarTo1')
    
    q = [q0(4), q0(1:3)];
    
elseif strcmpi(MODE, 'ScalarTo4')
    
    q = [q0(2:4), q0(1)];
    
end

end
