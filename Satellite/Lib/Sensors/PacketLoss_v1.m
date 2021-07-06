function ReadOrNot = PacketLoss_v1(pastReading,p,r)
% SendMessages: a valueequal to 1 if it was sent last time and 0 if not
% p: probability of losing the i-th message if the previous one was send
% r: probability of starting to send the i-th message if the previous one
%was lost
% GILBERT-ELLIOT MODEL (loss bursts)
if(pastReading==1) 
    if(rand(1) <= p)
        ReadOrNot = 0;
    else
        ReadOrNot = 1;
    end
else
    if(rand(1) <= r)
        ReadOrNot = 1;
    else
        ReadOrNot = 0;
    end
end

