function animation = drone_Animation(xhistory, yreftot, env)
% This Animation code is for QuadCopter. Written by Jitendra Singh 

dronex = xhistory(:,1);
droney = xhistory(:,2);
dronez = xhistory(:,3);
roll = xhistory(:,4);
pitch = xhistory(:,5);
yaw = xhistory(:,6);
dronevarx = xhistory(:,7);
dronevary = xhistory(:,8);
dronevarz = xhistory(:,9);

x = yreftot(:,1);
y = yreftot(:,2);
z = yreftot(:,3);
roll = yreftot(:,4);
pitch = yreftot(:,5);
yaw = yreftot(:,6);
varx = gradient(yreftot(:,1));
vary = gradient(yreftot(:,2));
varz = gradient(yreftot(:,3));

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;
 view(68,53);
 grid on;
 axis equal;
 xlim([-env*20 env*20]); ylim([-env*20 env*20]); zlim([-env*10 env*10]);
 title('CyRos Drone Animation')
 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');
 
%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)

    function draw_ring(x, y, z, varx, vary, varz, size, teta)

    xx = size*sin(teta);
    yy = size*cos(teta);
    zz = 0*teta;
    pnts = [xx; yy; zz];
    n0 = [0;0;1];
    n0 = n0/norm(n0);
    var = [varx;vary;varz];
    n1 = var/norm(var);
    c = dot(n0,n1) / ( norm(n0)*norm(n1) );
    disp(c);
    s = sqrt(1-c*c); 
    u = cross(n0,n1) / ( norm(n0)*norm(n1) );
    disp(u);
    u = u/norm(u); 
    C = 1-c;

    R = [u(1)^2*C+c, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
         u(2)*u(1)*C+u(3)*s, u(2)^2*C+c, u(2)*u(3)*C-u(1)*s
         u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+c];

    pnts = R*pnts;
    pnts = pnts + [x;y;z];
    plot3(pnts(1,:), pnts(2,:), pnts(3,:), 'b')
end

teta = linspace(0, 2*pi, 100);

for i = 1:length(x)-1
    draw_ring(x(i), y(i), z(i), varx(i), vary(i), varz(i), env, teta);
end
%plot3(x,y,z, 'g-','LineWidth',1);
drawnow

 for i = 1:length(x)

     plot3(dronex(1:i),droney(1:i),dronez(1:i), 'r-','LineWidth',1);
     %disp([varx(i),vary(i),varz(i)]);
     translation = makehgtform('translate',...
                               [dronex(i) droney(i) dronez(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
      %movieVector(i) =  getframe(fig1);
        %delete(b);
     pause(0.001);
 end

drawnow

end