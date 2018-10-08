%% HOMEWORK 2
%  Jonathan Sanchez
%  RBE3001
%  9/7/18

%% cleanup
clear
clc
close all

%% 1.a

% <<intermediate_transforms.jpg>>

%% 1.b
%define our symbolic DH params
syms A B C q1 q2 q3

%put them in a matrix to be retrieved and modified easily
DHsym = [A q1 + pi/2 0 pi/2;...
       0 q2 B 0;...
       0 q3 C 0]
   
%% 1.c
% see end of file to see the function definition
% (matlab needs function definitions at the end of a file)
   
%% 1.d   
% define each of the transformation matrices   
T01sym = makeDHtrans(DHsym(1,1), DHsym(1,2), DHsym(1,3), DHsym(1,4))
T12sym = makeDHtrans(DHsym(2,1), DHsym(2,2), DHsym(2,3), DHsym(2,4))
T23sym = makeDHtrans(DHsym(3,1), DHsym(3,2), DHsym(3,3), DHsym(3,4))
%% 1.e
% calculate the forward transformation from base to tip
T03sym = T01sym * T12sym * T23sym

%% 1.f
% solve numerically for each of the new matrices
% substituting theta1 = 15deg, theta2 = 30deg, theta3 = -15deg, A=60, B=40,
% C = 20
% 15 = pi/12, 30 = pi/6
DHval = subs(DHsym, [q1, q2, q3, A, B, C], [pi/12, pi/6, -pi/12, 30, 20, 20]);
T01val = makeDHtrans(DHval(1,1), DHval(1,2), DHval(1,3), DHval(1,4))
T12val = makeDHtrans(DHval(2,1), DHval(2,2), DHval(2,3), DHval(2,4))
T23val = makeDHtrans(DHval(3,1), DHval(3,2), DHval(3,3), DHval(3,4))

%% 1.g
% determine the approach vector along the x axis relative to frame 3
% re-define T03 with the newly substituted values
T03val = T01val * T12val * T23val;
% extract the x-dir vector (which is the approach vector)
approachX = T03val(1:3,1)

%% 1.h
% take the time derivative of the 3x1 posn vector from 1.e
% to determine fwd velocity kinematics

xt = T03sym(1:3, 4) % rows 1-3 of column 4
diffP03 = diff(xt, q1) % this is nightmarishly long


%% 1.i
% determine the robot's full velocity kinematics
% aka solve for the 6x3 jacobian

% define the blank 6x3 jacobian
syms J;
% complete the 'top half'
J(1:3, 1) = diff(xt, q1);
J(1:3, 2) = diff(xt, q2);
J(1:3, 3) = diff(xt, q3);
% we know the next part will always be zeros
J(4:6, 1) = [0 0 1];

% this is the part with the switch statement and extracting rotation
% vectors for the axis where rotation happens
% zeta n * z n-1 (q)
J(4:6, 2) = 1 * T01sym(1:3, 3);

T02sym = T01sym * T12sym;

J(4:6, 3) = 1 * T02sym(1:3, 3);

disp(J);
%% 1.j
% i. numerically solve for the jacobian if it is in 1.f

% ii. what is the instantaneous 6x1 velocity vector if all joints have an
% instantaneous angular velocity of positive 30deg/s?

Jval = subs(J, [q1, q2, q3, A, B, C], [pi/12, pi/6, -pi/12, 30, 20, 20])

qdot = [pi/6; pi/6; pi/6];

instVel = Jval * qdot 

% intVel(1:3) is in cm/s
% intVel(4:6) in rad/s

%% 1.c
function [transMatrix] = makeDHtrans(d, theta, a, alpha)
%MAKEDHTRANS return symbolic representation of trans matrix
%            given symbolic DH inputs
%   either accepts numeric values or symbolic vars
%   then returns corrosponding numeric or symbolic matrix

d = sym(d);
theta = sym(theta);
a = sym(a);
alpha = sym(alpha);

cosT = cos(theta);
sinT = sin(theta);

cosA = cos(alpha);
sinA = sin(alpha);

transMatrix = ...
    [cosT    -sinT*cosA    sinT*sinA    a*cosT;...
    sinT     cosT*cosA     -cosT*sinA   a*sinT;...
    0        sinA          cosA         d;...
    0        0             0            1];


end
