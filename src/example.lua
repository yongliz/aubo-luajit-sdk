math = require("math")
local libAubo = require("mod_libaubo")
local ffi = require("ffi")

-- 初始位置
initPos = ffi.new(
"double[6]",
{
    -0.000172 / 180 * math.pi,
    -7.291862 / 180 * math.pi,
    -75.694718 / 180 * math.pi,
    21.596727 / 180 * math.pi,
    -89.999982 / 180 * math.pi,
    -0.00458 / 180 * math.pi
}
)

-- 打印waypoint详细信息
function print_waypoint(waypoint)
    print("-------------pos-----------------")
    print("pos.x = " .. waypoint.pos.x)
    print("pos.y = " .. waypoint.pos.y)
    print("pos.z = " .. waypoint.pos.z)
    print("-------------ori-----------------")
    print("ori.w = " .. waypoint.ori.w)
    print("ori.x = " .. waypoint.ori.x)
    print("ori.y = " .. waypoint.ori.y)
    print("ori.z = " .. waypoint.ori.z)
    print("-------------joint---------------")
    print("joint.1 = " .. waypoint.joint[0] * 180 / math.pi)
    print("joint.2 = " .. waypoint.joint[1] * 180 / math.pi)
    print("joint.3 = " .. waypoint.joint[2] * 180 / math.pi)
    print("joint.4 = " .. waypoint.joint[3] * 180 / math.pi)
    print("joint.5 = " .. waypoint.joint[4] * 180 / math.pi)
    print("joint.6 = " .. waypoint.joint[5] * 180 / math.pi)
    print("---------------------------------")
end

-- 初始化
function robot_init()
    robot_control = libAubo.new()
    robot_control:rs_initialize()
    robot_control:rs_create_context()
    robot_control:rs_login("localhost", 8899)
    robot_control:rs_init_global_move_profile()
end

-- 反初始化
function robot_uninit()
    robot_control:rs_logout()
    robot_control:rs_uninitialize()
end

-- 示教器停止回调处理函数
function stop_proc()
    robot_uninit()
end

-- 轴动回零位
function movej_to_zero()
    zero_pos = ffi.new("double[6]", { 0, 0, 0, 0, 0, 0 })
    robot_control:rs_move_joint(zero_pos, true)
end

-- 轴动，直线测试
function movej_movel_test()
    -- 最大加速度
    max_acc = ffi.new("JointVelcAccParam[1]")
    max_acc[0].jointPara = { 5, 5, 5, 5, 5, 5 }
    -- 最大速度
    max_velc = ffi.new("JointVelcAccParam[1]")
    max_velc[0].jointPara = { 2, 2, 2, 2, 2, 2 }
    -- 设置最大加速度
    robot_control:rs_set_global_joint_maxacc(max_acc[0])
    -- 设置最大速度
    robot_control:rs_set_global_joint_maxvelc(max_velc[0])

    -- 循环多次测试轴动
    joint_radian1 = ffi.new("double[6]", { 1, 1, 1, 1, 1, 1 })
    joint_radian2 = ffi.new("double[6]", { 0, 0, 0, 0, 0, 0 })
    count = 3
    while (count > 0) do
        robot_control:rs_move_joint(joint_radian1, true)
        robot_control:rs_move_joint(joint_radian2, true)
        count = count - 1
    end

    -- 运动到初始位置
    robot_control:rs_move_joint(initPos, true)

    -- 直线运动到制定位置
    target = ffi.new(
    "double[6]",
    {
        -0.000172 / 180 * math.pi,
        3.869431 / 180 * math.pi,
        -80.452901 / 180 * math.pi,
        5.677671 / 180 * math.pi,
        -89.999982 / 180 * math.pi,
        -0.00458 / 180 * math.pi
    }
    )
    robot_control:rs_move_line(target, true)
end

-- 相对运动测试
function relative_move_test()
    -- 0,初始化机械臂全局参数
    robot_control:rs_init_global_move_profile()

    -- 1,运动到初始位置
    robot_control:rs_move_joint(initPos, true)

    -- 2,设置用户坐标系
    user_coord = ffi.new("CoordCalibrate[1]")
    user_coord[0].coordType = 0
    user_coord[0].methods = 0
    user_coord[0].toolDesc = { { 0, 0, 0 }, { 1, 0, 0, 0 } }
    user_coord[0].wayPointArray = {
        {-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008 },
        {-0.000006, -0.397548, -1.504121, 0.464224, -1.570796, -0.000011 },
        {-0.372856, -0.233512, -1.403906, 0.400403, -1.570793, -0.372861 }
    }

    relative = ffi.new("MoveRelative[1]")
    relative[0].ena = true
    relative[0].relativePosition = { 0, 0, 0.1 }
    relative[0].relativeOri = { 1, 0, 0, 0 }

    -- 3,设置相对运动
    robot_control:rs_set_relative_offset_on_user(relative[0], user_coord[0])

    -- 4,相对坐标系原点做相对Ｚ轴运动
    target = ffi.new(
    "double[6]",
    {
        -0.000003,
        -0.127267,
        -1.321122,
        0.376934,
        -1.570796,
        -0.000008
    }
    )
    -- 5,执行相对运动
    robot_control:rs_move_line(target, true)

    -- 6,初始化机械臂全局参数
    robot_control:rs_init_global_move_profile()
end

-- 正逆解测试
function ik_fk_test()
    -- 测试一，正解测试
    waypoint = ffi.new("wayPoint_S[1]")
    robot_control:rs_forward_kin(initPos, waypoint[0])
    -- 打印waypoint详细信息
    print_waypoint(waypoint[0])

    -- 测试二，逆解测试
    -- 1,参考关节角，这里使用初始位置作为参考
    -- 2,目标路点的位置 单位:米
    target_pos = ffi.new("Pos[1]")
    target_pos[0] = {-0.512104, -0.209791, 0.392881 }
    -- 3,目标路点的参考姿态，姿态参考初始位置姿态
    target_ori = ffi.new("Ori[1]")
    target_ori[0] = waypoint[0].ori
    -- 4,逆解
    robot_control:rs_inverse_kin(initPos, target_pos[0], target_ori[0], waypoint[0])
    -- 打印waypoint详细信息
    print_waypoint(waypoint[0])
end

function user_io_test()
    -- 设置用户ＩＯ状态
    robot_control:rs_set_user_do(robot_control.ROBOT_IO_U_DO_00, 1)

    -- 获取用户ＩＯ状态
    status = ffi.new("double[1]", { 0 })
    robot_control:rs_get_user_do(robot_control.ROBOT_IO_U_DO_00, status)
    print("get use do." .. robot_control.ROBOT_IO_U_DO_00 .. "=" .. status[0])
end

--主函数
function main()
    -- 初始化
    robot_init()
    -- 轴动直线运动测试
    movej_movel_test()
    -- 相对运动测试
    relative_move_test()
    -- 轴动回零位
    movej_to_zero()
    -- 正逆解测试
    ik_fk_test()
    -- 用户坐标系转基座坐标系
    -- 基座坐标系转用户坐标系
    --用户ＩＯ测试
    user_io_test()
    -- 反初始化
    robot_uninit()
end

--测试
main()