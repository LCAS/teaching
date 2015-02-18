import math

wheel_radius = 1 / (math.pi * 2.0)
robot_radius = 0.5

wheel_circumference = 2 * math.pi * wheel_radius


def forward_kinematics(w_l, w_r):
    c_l = wheel_circumference * w_l
    c_r = wheel_circumference * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / robot_radius
    return (v, a)


def inverse_kinematics(v, a):
    c_l = v + (robot_radius * a) / 2
    c_r = v - (robot_radius * a) / 2
    w_l = c_l / wheel_circumference
    w_r = c_r / wheel_circumference
    return (w_l, w_r)


(w_l, w_r) = inverse_kinematics(1.0, 2 * math.pi)
print "w_l = %f,\tw_r = %f" % (w_l, w_r)

(v, a) = forward_kinematics(w_l, w_r)
print "v = %f,\ta = %f" % (v, a)
