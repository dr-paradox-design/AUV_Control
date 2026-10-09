function f = manoeuvre()
% MANOEUVRE  A generalised force that surges, sways, heaves and turns, so that every
% degree of freedom and the rotation of the body frame are exercised.
f = @(t) [10; 2 * sin(0.5 * t); 1.0 * sin(0.3 * t); 0.5 * sin(0.7 * t); 0.4 * sin(0.6 * t); 1.5 * sin(0.8 * t) + 0.5];
end
