function velocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startVelocity, endVelocity, maxSpeed)
%helperGenerateVelocityProfile Generate a velocity profile for a reference path

% Copyright 2017-2019 The MathWorks, Inc.

velocities = driving.internal.planning.generateVelocityProfile( ...
    directions, cumLengths, curvatures, startVelocity, endVelocity, ...
    'MaxSpeed', maxSpeed, 'MaxLateralAccel', 2);
end