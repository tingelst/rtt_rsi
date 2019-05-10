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

-- Load my component
depl:import("rtt_rsi")
depl:loadComponent("rsi", "rtt_rsi::RSIComponent")
rsi = depl:getPeer("rsi")
rsi:configure()