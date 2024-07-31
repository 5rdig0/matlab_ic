rosshutdown;
rosinit;

msg = rossubscriber("/uav1/hw_api/odometry","DataFormat","object");
msg = receive(msg,10);
disp(msg);
px = (msg.Pose.Pose.Position.X);
py = (msg.Pose.Pose.Position.Y);
pz = (msg.Pose.Pose.Position.Z);

ox = (msg.Pose.Pose.Orientation.X);
oy = (msg.Pose.Pose.Orientation.Y);
oz = (msg.Pose.Pose.Orientation.Z);
w = (msg.Pose.Pose.Orientation.W);

covariance = (msg.Pose.Covariance);

disp([px,py,pz,ox,oy,oz,w]);
disp(covariance);
rosshutdown;