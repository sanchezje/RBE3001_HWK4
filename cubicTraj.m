function [coeffSolns] = cubicTraj(t0, tf, v0, vf, q0, qf)
%cubicTraj produces a trajectory using a solution to a cubic polynomial
%   consumes start time (seconds) end time (seconds),
%   start velocity (deg/s) and end velocity (deg/s)
%   start position (deg) and end position (deg)
coeffSolns = zeros(4,1,'single');

syms a0 a1 a2 a3;

eqn1 = a0 + a1*t0 + a2*t0^2 + a3*t0^3 == q0;
eqn2 = a1 + 2*a2*t0 + 3*a3*t0^2 == v0;
eqn3 = a0 + a1*tf + a2*tf^2 + a3*tf^3 == qf;
eqn4 = a1 + 2*a2*tf + 3*a3*tf^2 == vf;

[A,B] = equationsToMatrix([eqn1 eqn2 eqn3 eqn4],[a0 a1 a2 a3]);
coeffSolns = linsolve(A,B);
end

