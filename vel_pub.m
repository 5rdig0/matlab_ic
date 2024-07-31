rosshutdown;
rosinit;

speed = -20.0;

pub = rospublisher("/uav1/motor_speed/0","std_msgs/Float64","DataFormat","object");
msg = rosmessage(pub);
msg.Data = speed;

pause(2);
%disp(msg);
for i = 0:100000
    send(pub, msg);
end

%rostopic info /uav1/motor_speed/0;

rosshutdown;
