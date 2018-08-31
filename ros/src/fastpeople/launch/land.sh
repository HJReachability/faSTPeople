# Tell each quadrotor to land, and put both calls in the background
# so that they execute simultaneously.

rosservice call landHY1 &
rosservice call landHY2 &
