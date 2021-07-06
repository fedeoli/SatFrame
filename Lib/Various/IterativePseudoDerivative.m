function [buffer_out, dy] = IterativePseudoDerivative(T,y,c,d,mediana,buffer)

%T: sampling time (equally spaced samples)
%y: actual data
%c  the length of the two subwindows where the mean/median is evaluated
%d  is total number of data considered to evaluate the pseudo-derivative, d>=2c
%reset: =1 if the data buffer has to be discarded, 0 otherwise
%mediana = 1 the median is evaluated on the burst of the c samples,
%otherwise the mean
% buffer = 1xd vector

% (------d------)
% (-c-)-----(-c-)
% [---]-----[---]

%FUNZIONE PER IL CALCOLO DELLA PSEUDODERIVATA
%----------------------------------

% global DynOpt

%update the buffer
for k=1:d-1
    buffer(k) = buffer(k+1);
end
buffer(d) = y;

    
if(mediana==1)
    temp1 = median(buffer(1:c));
    temp2 = median(buffer(d-c+1:d));
else
    temp1 = 0;
    for k=1:c
        temp1 = temp1 + buffer(k);
    end
    temp1 = temp1/c;

    temp2 = 0;
    for k=d-c+1:d
        temp2 = temp2 + buffer(k);
    end
    temp2 = temp2/c;
end

dy = (temp2-temp1) / (T*(d-c));
buffer_out = buffer;

end

%-------------------------------------
