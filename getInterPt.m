function [ thisAngle ] = getInterPt( coeff_solns, thisTime )
%GETINTERPT get an interpolated point (in ticks)
%   consumes a set of coeff solutions and time (in seconds)
%   produces a number of encoder ticks the axis should be at at this time
%   note: coeff_solns are the return value of cubicTraj

    a0 = coeff_solns(1);
    a1 = coeff_solns(2);
    a2 = coeff_solns(3);
    a3 = coeff_solns(4);
    
    tx = thisTime;
    
    
    thisAngle = a0 + a1*tx + a2*tx^2 + a3*tx^3;
end

