%%******RPY角和四元数之间的相互转换******
%%Written by Rot_Tianers
%%Code creation at:2020.03.19
%%**************************************
function [x, y, z, w] = Quaternions(roll, pitch, yaw)
clc;
%%已知：RPY角
%%求解：四元数四个变量值，输入到ROS中

%%step1:根据RPY角生成旋转矩阵（依次绕定轴X旋转roll\绕定轴Y旋转pitch\Z绕定轴旋转yaw）
alpha = roll;
theta = pitch;
gamma = yaw;
R = [cos(alpha)*cos(theta) cos(alpha)*sin(theta)*sin(gamma)-sin(alpha)*cos(gamma) cos(alpha)*sin(theta)*cos(gamma)+sin(alpha)*sin(gamma);...
    sin(alpha)*cos(theta) sin(alpha)*sin(theta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(theta)*cos(gamma)-cos(alpha)*sin(gamma);...
    -sin(theta) cos(theta)*sin(gamma) cos(theta)*cos(gamma)];
%%step2:根据转换公式求解四元数四个值
w = 0.5*sqrt(1+R(1,1)+R(2,2)+R(3,3));  %%w = cos(旋转角/2)
x = (R(3,2)-R(2,3))/(4*w);             %%x = kx*sin(旋转角/2)
y = (R(1,3)-R(3,1))/(4*w);             %%y = ky*sin(旋转角/2)
z = (R(2,1)-R(1,2))/(4*w);             %%z = kz*sin(旋转角/2)

%%step3:验证求解正确性
threshold = 10^-5;  %%阈值
test = abs(abs(y*w-x*z)-0.5);
if (test<threshold)  %%奇异姿态，俯仰角为±90°
    RPY.R = atan2(2*(y*z+w*x),1-2*(y^2+x^2));
    RPY.Y = asin(2*(y*w-x*z));
    RPY.Z = atan2(2*(x*y+w*z),1-2*(y^2+z^2));
    RPY
    disp("万向节锁情况");
else
    RPY.R = atan2(2*(y*z+w*x),1-2*(y^2+x^2));
    RPY.Y = asin(2*(y*w-x*z));
    RPY.Z = atan2(2*(x*y+w*z),1-2*(y^2+z^2));
    RPY
    disp("正常求解");
end
end








