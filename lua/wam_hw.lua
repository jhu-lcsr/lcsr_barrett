
--[[ get ROS global service --]]
depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("lcsr_barrett")

--[ Set log-level ]--
rtt.setLogLevel("Warning")

--[[ add lua dir to the lua path --]]
package.path = ros:find("lcsr_barrett") .. "/lua/?.lua" .. ";" .. package.path

require("lcsr_barrett")
lcsr_barrett()
