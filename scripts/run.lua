require "rttlib"
require "rttros"

rtt.setLogLevel("Info")

-- Task Context
tc = rtt.getTC()
-- Deployer
depl = tc:getPeer("Deployer")

-- Interface with ROS:
depl:import("rtt_ros")

-- Load my component
depl:import("rtt_rsi")
depl:loadComponent("rsi", "rtt_rsi::RSIComponent")
rsi = depl:getPeer("rsi")
rsi:configure()