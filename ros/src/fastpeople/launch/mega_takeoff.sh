# Tell each quadrotor to takeoff, and put both calls in the background
# so that they execute simultaneously.

rosservice call takeoffHY4 &
rosservice call takeoffHY5 &
rosservice call takeoffHY6 &
rosservice call takeoffHY7 &
rosservice call takeoffHY8 &
