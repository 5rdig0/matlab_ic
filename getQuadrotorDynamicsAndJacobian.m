% Create symbolic functions for time-dependent angles
% phi: roll angle
% theta: pitch angle
% psi: yaw angle
syms phi(t) theta(t) psi(t)

% Transformation matrix for angular velocities from inertial frame
% to body frame
W = [ 1,  0,        -sin(theta);
      0,  cos(phi),  cos(theta)*sin(phi);
      0, -sin(phi),  cos(theta)*cos(phi) ];

% Rotation matrix R_ZYX from body frame to inertial frame
R = rotationMatrixEulerZYX(phi,theta,psi);

% Create symbolic variables for diagonal elements of inertia matrix
syms Ixx Iyy Izz

% Jacobian that relates body frame to inertial frame velocities
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;

% Coriolis matrix
dJ_dt = diff(J);
h_dot_J = [diff(phi,t), diff(theta,t), diff(psi,t)]*J;
grad_temp_h = transpose(jacobian(h_dot_J,[phi theta psi]));
C = dJ_dt - 1/2*grad_temp_h;
C = subsStateVars(C,t);

% Evaluate C times etadot using symbolic definition
phidot   = subsStateVars(diff(phi,t),t);
thetadot = subsStateVars(diff(theta,t),t);
psidot   = subsStateVars(diff(psi,t),t);
Csym_etadot = C*[phidot; thetadot; psidot];


% Evaluate C times etadot using reference paper definition
C11 = 0;
C12 = (Iyy - Izz)*(thetadot*cos(phi)*sin(phi) + psidot*sin(phi)^2*cos(theta)) ...
      + (Izz - Iyy)*(psidot*cos(phi)^2*cos(theta)) - Ixx*psidot*cos(theta);
C13 = (Izz - Iyy)*psidot*cos(phi)*sin(phi)*cos(theta)^2;
C21 = (Izz - Iyy)*(thetadot*cos(phi)*sin(phi) + psidot*sin(phi)^2*cos(theta)) ...
      + (Iyy - Izz)*psidot*cos(phi)^2*cos(theta) + Ixx*psidot*cos(theta);
C22 = (Izz - Iyy)*phidot*cos(phi)*sin(phi);
C23 = - Ixx*psidot*sin(theta)*cos(theta) + Iyy*psidot*sin(phi)^2*sin(theta)*cos(theta) ...
      + Izz*psidot*cos(phi)^2*sin(theta)*cos(theta);
C31 = (Iyy - Izz)*psidot*cos(theta)^2*sin(phi)*cos(phi) - Ixx*thetadot*cos(theta);
C32 = (Izz - Iyy)*(thetadot*cos(phi)*sin(phi)*sin(theta) + phidot*sin(phi)^2*cos(theta)) ...
      + (Iyy - Izz)*phidot*cos(phi)^2*cos(theta) + Ixx*psidot*sin(theta)*cos(theta) ...
      - Iyy*psidot*sin(phi)^2*sin(theta)*cos(theta) - Izz*psidot*cos(phi)^2*sin(theta)*cos(theta);
C33 = (Iyy - Izz)*phidot*cos(phi)*sin(phi)*cos(theta)^2 ...
      - Iyy*thetadot*sin(phi)^2*cos(theta)*sin(theta) ...
      - Izz*thetadot*cos(phi)^2*cos(theta)*sin(theta) + Ixx*thetadot*cos(theta)*sin(theta);
Cpaper = [C11, C12, C13; C21, C22, C23; C31 C32 C33];
Cpaper_etadot = Cpaper*[phidot; thetadot; psidot];
Cpaper_etadot = subsStateVars(Cpaper_etadot,t);

tf_Cdefinition = isAlways(Cpaper_etadot == Csym_etadot);

% Define fixed parameters and control input
% k: lift constant
% l: distance between rotor and center of mass
% m: quadrotor mass
% b: drag constant
% g: gravity
% ui: squared angular velocity of rotor i as control input
syms k l m b g u1 u2 u3 u4

% Torques in the direction of phi, theta, psi
tau_beta = [l*k*(-u2+u4); l*k*(-u1+u3); b*(-u1+u2-u3+u4)];

% Total thrust
T = k*(u1+u2+u3+u4);

% Create symbolic functions for time-dependent positions
syms x(t) y(t) z(t)

% Create state variables consisting of positions, angles,
% and their derivatives
state = [x; y; z; phi; theta; psi; diff(x,t); diff(y,t); ...
    diff(z,t); diff(phi,t); diff(theta,t); diff(psi,t)];
state = subsStateVars(state,t);

f = [ % Set time-derivative of the positions and angles
      state(7:12);

      % Equations for linear accelerations of the center of mass
      -g*[0;0;1] + R*[0;0;T]/m;

      % Euler–Lagrange equations for angular dynamics
      inv(J)*(tau_beta - C*state(10:12))
];

f = subsStateVars(f,t);

% Replace fixed parameters with given values here
IxxVal = 1.2;
IyyVal = 1.2;
IzzVal = 2.3;
kVal = 1;
lVal = 0.25;
mVal = 2;
bVal = 0.2;
gVal = 9.81;

f = subs(f, [Ixx Iyy Izz k l m b g], ...
    [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal]);
f = simplify(f);
% Calculate Jacobians for nonlinear prediction model
A = jacobian(f,state);
control = [u1; u2; u3; u4];
B = jacobian(f,control);

% Create QuadrotorStateFcn.m with current state and control
% vectors as inputs and the state time-derivative as outputs
matlabFunction(f,'File','QuadrotorStateFcn', ...
    'Vars',{state,control});

% Create QuadrotorStateJacobianFcn.m with current state and control
% vectors as inputs and the Jacobians of the state time-derivative
% as outputs
matlabFunction(A,B,'File','QuadrotorStateJacobianFcn', ...
    'Vars',{state,control});

function [Rz,Ry,Rx] = rotationMatrixEulerZYX(phi,theta,psi)
% Euler ZYX angles convention
    Rx = [ 1,           0,          0;
           0,           cos(phi),  -sin(phi);
           0,           sin(phi),   cos(phi) ];
    Ry = [ cos(theta),  0,          sin(theta);
           0,           1,          0;
          -sin(theta),  0,          cos(theta) ];
    Rz = [cos(psi),    -sin(psi),   0;
          sin(psi),     cos(psi),   0;
          0,            0,          1 ];
    if nargout == 3
        % Return rotation matrix per axes
        return;
    end
    % Return rotation matrix from body frame to inertial frame
    Rz = Rz*Ry*Rx;
end

function stateExpr = subsStateVars(timeExpr,var)
    if nargin == 1 
        var = sym("t");
    end
    repDiff = @(ex) subsStateVarsDiff(ex,var);
    stateExpr = mapSymType(timeExpr,"diff",repDiff);
    repFun = @(ex) subsStateVarsFun(ex,var);
    stateExpr = mapSymType(stateExpr,"symfunOf",var,repFun);
    stateExpr = formula(stateExpr);
end

function newVar = subsStateVarsFun(funExpr,var)
    name = symFunType(funExpr);
    name = replace(name,"_Var","");
    stateVar = "_" + char(var);
    newVar = sym(name + stateVar);
end

function newVar = subsStateVarsDiff(diffExpr,var)
    if nargin == 1 
      var = sym("t");
    end
    c = children(diffExpr);
    if ~isSymType(c{1},"symfunOf",var)
      % not f(t)
      newVar = diffExpr;
      return;
    end
    if ~any([c{2:end}] == var)
      % not derivative wrt t only
      newVar = diffExpr;
      return;
    end
    name = symFunType(c{1});
    name = replace(name,"_Var","");
    extension = "_" + join(repelem("d",numel(c)-1),"") + "ot";
    stateVar = "_" + char(var);
    newVar = sym(name + extension + stateVar);
end