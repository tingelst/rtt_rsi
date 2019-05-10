require "rttlib"
require "rttros"

rttlib.color=true -- Colored console output
rtt.setLogLevel("Info")

-- Task Context
tc = rtt.getTC()
-- Deployer
depl = tc:getPeer("Deployer")

-- Interface with ROS:
depl:import("rtt_ros")

-- Load RSI component
depl:import("rtt_rsi")
depl:loadComponent("rsi", "rtt_rsi::RSIComponent")
rsi = depl:getPeer("rsi")

-- Set ip address of robot
robot_ip = rsi:getProperty("local_host")
robot_port = rsi:getProperty("local_port")
robot_ip:set("127.0.0.1") -- change this to the IP to connect
robot_port:set(49152) -- Listening port
rsi:configure()
