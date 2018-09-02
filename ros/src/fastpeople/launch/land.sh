# Tell each quadrotor to land, and put both calls in the background
# so that they execute simultaneously.

rosservice call landHY4 &
rosservice call landHY5 &
