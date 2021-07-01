function sleep(n)
    os.execute("sleep " .. tonumber(n))
end

local libAubo = require("mod_libaubo")
local ffi = require("ffi")

robotControl = libAubo.new()

robotControl:rs_initialize()
robotControl:rs_create_context()
robotControl:rs_login("localhost", 8899)

print("pause robot....")
robotControl:rs_move_pause()

sleep(3)

print("continue robot....")
robotControl:rs_move_continue()

robotControl:rs_logout()
