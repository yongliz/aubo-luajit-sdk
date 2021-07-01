local robot = {}
local AuboRobot = {}

function AuboRobot:rs_initialize()
    return self.libfd.rs_initialize()
end

function AuboRobot:rs_uninitialize()
    return self.libfd.rs_uninitialize()
end

function AuboRobot:rs_create_context()
    return self.libfd.rs_create_context(self.rshd)
end

function AuboRobot:rs_destory_context()
    return self.libfd.rs_destory_context(self.rshd[0])
end

function AuboRobot:rs_login(addr, port)
    return self.libfd.rs_login(self.rshd[0], addr, port)
end

function AuboRobot:rs_logout()
    return self.libfd.rs_logout(self.rshd[0])
end

function AuboRobot:rs_init_global_move_profile()
    return self.libfd.rs_init_global_move_profile(self.rshd[0])
end

function AuboRobot:rs_set_global_joint_maxacc(max_acc)
    return self.libfd.rs_set_global_joint_maxacc(self.rshd[0], max_acc)
end

function AuboRobot:rs_set_global_joint_maxvelc(max_velc)
    return self.libfd.rs_set_global_joint_maxvelc(self.rshd[0], max_velc)
end

function AuboRobot:rs_set_global_end_max_line_acc(max_acc)
    return self.libfd.rs_set_global_end_max_line_acc(self.rshd[0], max_acc)
end

function AuboRobot:rs_set_global_end_max_line_velc(max_velc)
    return self.libfd.rs_set_global_end_max_line_velc(self.rshd[0], max_velc)
end

function AuboRobot:rs_set_global_end_max_angle_acc(max_acc)
    return self.libfd.rs_set_global_end_max_angle_acc(self.rshd[0], max_acc)
end

function AuboRobot:rs_set_global_end_max_angle_velc(max_velc)
    return self.libfd.rs_set_global_end_max_angle_velc(self.rshd[0], max_velc)
end

function AuboRobot:rs_move_joint(joint_radian, isblock)
    return self.libfd.rs_move_joint(self.rshd[0], joint_radian, isblock)
end

function AuboRobot:rs_move_line(joint_radian, isblock)
    return self.libfd.rs_move_line(self.rshd[0], joint_radian, isblock)
end

function AuboRobot:rs_remove_all_waypoint()
    return self.libfd.rs_remove_all_waypoint(self.rshd[0])
end

function AuboRobot:rs_add_waypoint(joint_radian)
    return self.libfd.rs_add_waypoint(self.rshd[0], joint_radian)
end

function AuboRobot:rs_set_blend_radius(radius)
    return self.libfd.rs_set_blend_radius(self.rshd[0], radius)
end

function AuboRobot:rs_set_circular_loop_times(times)
    return self.libfd.rs_set_circular_loop_times(self.rshd[0], times)
end

-- int rs_set_user_coord(RSHD rshd, const CoordCalibrate *user_coord);
function AuboRobot:rs_set_user_coord(user_coord)
    return self.libfd.rs_set_user_coord(self.rshd[0], user_coord)
end

function AuboRobot:rs_set_relative_offset_on_user(relative, user_coord)
    return self.libfd.rs_set_relative_offset_on_user(self.rshd[0], relative, user_coord)
end

function AuboRobot:rs_forward_kin(joint_radian, waypoint)
    return self.libfd.rs_forward_kin(self.rshd[0], joint_radian, waypoint)
end

function AuboRobot:rs_inverse_kin(joint_radian, pos, ori, waypoint)
    return self.libfd.rs_inverse_kin(self.rshd[0], joint_radian, pos, ori, waypoint)
end

function AuboRobot:rs_move_pause()
    return self.libfd.rs_move_pause(self.rshd[0])
end

function AuboRobot:rs_move_continue()
    return self.libfd.rs_move_continue(self.rshd[0])
end

function AuboRobot:rs_move_stop()
    return self.libfd.rs_move_stop(self.rshd[0])
end

function AuboRobot:rs_move_fast_stop()
    return self.libfd.rs_move_fast_stop(self.rshd[0])
end

function AuboRobot:rs_get_user_di(addr, val)
    return self.libfd.rs_get_board_io_status_by_addr(self.rshd[0], 4, addr, val)
end

function AuboRobot:rs_set_user_do(addr, val)
    return self.libfd.rs_set_board_io_status_by_addr(self.rshd[0], 5, addr, val)
end

function AuboRobot:rs_get_user_do(addr, val)
    return self.libfd.rs_get_board_io_status_by_addr(self.rshd[0], 5, addr, val)
end

function AuboRobot:rs_get_user_ai(addr, val)
    return self.libfd.rs_get_board_io_status_by_addr(self.rshd[0], 6, addr, val)
end

function AuboRobot:rs_set_user_ao(addr, val)
    return self.libfd.rs_set_board_io_status_by_addr(self.rshd[0], 7, addr, val)
end

function AuboRobot:rs_get_user_ao(addr, val)
    return self.libfd.rs_get_board_io_status_by_addr(self.rshd[0], 7, addr, val)
end

function robot.new()
    local self = {}
    setmetatable(self, { __index = AuboRobot })
    local ffi = require("ffi")
    local C = ffi.C
    local string = require("string")
    self.libfd = ffi.load("./libpyauboi5.so")
    ffi.cdef [[
        typedef unsigned short RSHD;

        typedef struct {
            double jointPara[6];
        }JointVelcAccParam;

        typedef struct {
            double x;
            double y;
            double z;
        }Pos;

        typedef struct {
            double w;
            double x;
            double y;
            double z;
        }Ori;

        typedef struct {
            Pos toolInEndPosition;
            Ori toolInEndOrientation;
        }ToolInEndDesc;

        typedef ToolInEndDesc ToolKinematicsParam;

        typedef struct {
            int coordType;
            int methods;
            double wayPointArray[3][6];
            ToolInEndDesc toolDesc;
        }CoordCalibrate;

        typedef struct{
            bool  ena;
            float relativePosition[3];
            Ori   relativeOri;
        }MoveRelative;

        typedef struct{
            Pos pos;
            Ori ori;
            double joint[6];
        }wayPoint_S;

        /**
        * @brief 初始化机械臂控制库
        * @return RS_SUCC 成功 其他失败
        */
        int rs_initialize(void);

        /**
        * @brief 反初始化机械臂控制库
        * @return RS_SUCC 成功 其他失败
        */
        int rs_uninitialize(void);
        
        //robot service context
        /**
        * @brief 创建机械臂控制上下文句柄
        * @param rshd
        * @return RS_SUCC 成功 其他失败
        */
        int rs_create_context(RSHD *rshd/*returned context handle*/);
        
        /**
        * @brief 注销机械臂控制上下文句柄
        * @param rshd
        * @return RS_SUCC 成功 其他失败
        */
        int rs_destory_context(RSHD rshd);
        
        //login logout
        /**
        * @brief 链接机械臂服务器
        * @param rshd 械臂控制上下文句柄
        * @param addr 机械臂服务器的IP地址
        * @param port 机械臂服务器的端口号
        * @return RS_SUCC 成功 其他失败
        */
        int rs_login(RSHD rshd, const char * addr, int port);
        
        /**
        * @brief 断开机械臂服务器链接
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_logout(RSHD rshd);

        /**
        * @brief 初始化全局的运动属性
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_init_global_move_profile(RSHD rshd);

        /**
        * @brief 设置六个关节的最大加速度
        * @param rshd 械臂控制上下文句柄
        * @param max_acc 六个关节的最大加速度，单位(rad/ss)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_joint_maxacc(RSHD rshd, const JointVelcAccParam  *max_acc);

        /**
        * @brief 设置六个关节的最大速度
        * @param rshd 械臂控制上下文句柄
        * @param max_velc 六个关节的最大速度，单位(rad/s)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_joint_maxvelc(RSHD rshd, const JointVelcAccParam *max_velc);

        /**
        * @brief 设置机械臂末端最大线加速度
        * @param rshd 械臂控制上下文句柄
        * @param max_acc 末端最大加线速度，单位(m/s^2)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_end_max_line_acc(RSHD rshd, double max_acc);

        /**
        * @brief 设置机械臂末端最大线速度
        * @param rshd 械臂控制上下文句柄
        * @param max_velc 末端最大线速度，单位(m/s)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_end_max_line_velc(RSHD rshd, double max_velc);

        /**
        * @brief 设置机械臂末端最大角加速度
        * @param rshd 械臂控制上下文句柄
        * @param max_acc 末端最大角加速度，单位(rad/s^2)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_end_max_angle_acc(RSHD rshd, double max_acc);

        /**
        * @brief 设置机械臂末端最大角速度
        * @param rshd 械臂控制上下文句柄
        * @param max_velc 末端最大速度，单位(rad/s)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_global_end_max_angle_velc(RSHD rshd, double max_velc);
        
        /**
        * @brief 机械臂轴动
        * @param rshd 械臂控制上下文句柄
        * @param joint_radian 六个关节的关节角，单位(rad)
        * @param isblock    isblock==true  代表阻塞，机械臂运动直到到达目标位置或者出现故障后返回。
        *                   isblock==false 代表非阻塞，立即返回，运动指令发送成功就返回，函数返回后机械臂开始运动。
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_joint(RSHD rshd, double joint_radian[6], bool isblock);

        /**
        * @brief 机械臂保持当前姿态直线运动
        * @param rshd 械臂控制上下文句柄
        * @param joint_radian 六个关节的关节角，单位(rad)
        * @param isblock    isblock==true  代表阻塞，机械臂运动直到到达目标位置或者出现故障后返回。
        *                   isblock==false 代表非阻塞，立即返回，运动指令发送成功就返回，函数返回后机械臂开始运动。
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_line(RSHD rshd, double joint_radian[6], bool isblock);

        /**
        * @brief 保持当前位置变换姿态做旋转运动
        * @param rshd 械臂控制上下文句柄
        * @param user_coord 用户坐标系
        * @param rotate_axis :转轴(x,y,z) 例如：(1,0,0)表示沿Y轴转动
        * @param rotate_angle 旋转角度 单位（rad）
        * @param isblock    isblock==true  代表阻塞，机械臂运动直到到达目标位置或者出现故障后返回。
        *                   isblock==false 代表非阻塞，立即返回，运动指令发送成功就返回，函数返回后机械臂开始运动。
        * @return RS_SUCC 成功 其他失败
        */
        //TODO
        //int rs_move_rotate(RSHD rshd, const CoordCalibrate *user_coord, const Move_Rotate_Axis *rotate_axis, double rotate_angle,  bool isblock = true);

        /**
        * @brief 清除所有已经设置的全局路点
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_remove_all_waypoint(RSHD rshd);

        /**
        * @brief 添加全局路点用于轨迹运动
        * @param rshd 械臂控制上下文句柄
        * @param joint_radian 六个关节的关节角，单位(rad)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_add_waypoint(RSHD rshd, double joint_radian[6]);

        /**
        * @brief 设置交融半径
        * @param rshd 械臂控制上下文句柄
        * @param radius 交融半径，单位(m)
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_blend_radius(RSHD rshd, double radius);

        /**
        * @brief 设置圆运动圈数
        * @param rshd 械臂控制上下文句柄
        * @param times 当times大于0时，机械臂进行圆运动times次
        *              当times等于0时，机械臂进行圆弧轨迹运动
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_circular_loop_times(RSHD rshd, int times);

        /**
        * @brief 设置用户坐标系
        * @param rshd 械臂控制上下文句柄
        * @param user_coord 用户坐标系
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_user_coord(RSHD rshd, const CoordCalibrate *user_coord);

        /**
        * @brief 设置基于用户标系运动偏移量
        * @param rshd 械臂控制上下文句柄
        * @param relative 相对位移(x, y, z) 单位(m)
        * @param user_coord 用户坐标系
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_relative_offset_on_user(RSHD rshd, const MoveRelative *relative, const CoordCalibrate *user_coord);

        /**
        * @brief 正解　　　　　此函数为正解函数，已知关节角求对应位置的位置和姿态。
        * @param rshd 械臂控制上下文句柄
        * @param joint_radian 六个关节的关节角，单位(rad)
        * @param waypoint 六个关节角,位置,姿态
        * @return RS_SUCC 成功 其他失败
        */
        int rs_forward_kin(RSHD rshd, const double joint_radian[6], wayPoint_S *waypoint);

        /**
        * @brief 逆解 此函数为机械臂逆解函数，根据位置信息(x,y,z)和对应位置的参考姿态(w,x,y,z)得到对应位置的关节角信息。
        * @param rshd 械臂控制上下文句柄
        * @param joint_radian 参考关节角（通常为当前机械臂位置）单位(rad)
        * @param pos 目标路点的位置 单位:米
        * @param ori 目标路点的参考姿态
        * @param waypoint 目标路点信息
        * @return RS_SUCC 成功 其他失败
        */
        int rs_inverse_kin(RSHD rshd, double joint_radian[6], const Pos *pos, const Ori *ori, wayPoint_S *waypoint);

        /**
        * @brief 停止机械臂运动
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_stop(RSHD rshd);

        /**
        * @brief 停止机械臂运动
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_fast_stop(RSHD rshd);

        /**
        * @brief 暂停机械臂运动
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_pause(RSHD rshd);

        /**
        * @brief 暂停后回复机械臂运动
        * @param rshd 械臂控制上下文句柄
        * @return RS_SUCC 成功 其他失败
        */
        int rs_move_continue(RSHD rshd);

        /**
        * @brief 根据接口板IO类型和地址设置IO状态
        * @param rshd 械臂控制上下文句柄
        * @param type IO类型
        * @param addr IO状态
        * @param val  IO状态
        * @return RS_SUCC 成功 其他失败
        */
        int rs_set_board_io_status_by_addr(RSHD rshd, int type, int addr, double val);

        /**
        * @brief 根据接口板IO类型和地址获取IO状态
        * @param rshd 械臂控制上下文句柄
        * @param type IO类型
        * @param addr IO地址
        * @param val
        * @return RS_SUCC 成功 其他失败
        */
        int rs_get_board_io_status_by_addr(RSHD rshd, int type, int addr, double *val);


    ]]

    self.rshd = ffi.new("RSHD[1]", { 0 })
    -- 接口板用户DI
    self.ROBOT_IO_F1    = 30
    self.ROBOT_IO_F2    = 31
    self.ROBOT_IO_F3    = 32
    self.ROBOT_IO_F4    = 33
    self.ROBOT_IO_F5    = 34
    self.ROBOT_IO_F6    = 35
    self.ROBOT_IO_U_DI_00 = 36
    self.ROBOT_IO_U_DI_01 = 37
    self.ROBOT_IO_U_DI_02 = 38
    self.ROBOT_IO_U_DI_03 = 39
    self.ROBOT_IO_U_DI_04 = 40
    self.ROBOT_IO_U_DI_05 = 41
    self.ROBOT_IO_U_DI_06 = 42
    self.ROBOT_IO_U_DI_07 = 43
    self.ROBOT_IO_U_DI_10 = 44
    self.ROBOT_IO_U_DI_11 = 45
    self.ROBOT_IO_U_DI_12 = 46
    self.ROBOT_IO_U_DI_13 = 47
    self.ROBOT_IO_U_DI_14 = 48
    self.ROBOT_IO_U_DI_15 = 49
    self.ROBOT_IO_U_DI_16 = 50
    self.ROBOT_IO_U_DI_17 = 51

    -- 接口板用户DO
    self.ROBOT_IO_U_DO_00 = 32
    self.ROBOT_IO_U_DO_01 = 33
    self.ROBOT_IO_U_DO_02 = 34
    self.ROBOT_IO_U_DO_03 = 35
    self.ROBOT_IO_U_DO_04 = 36
    self.ROBOT_IO_U_DO_05 = 37
    self.ROBOT_IO_U_DO_06 = 38
    self.ROBOT_IO_U_DO_07 = 39
    self.ROBOT_IO_U_DO_10 = 40
    self.ROBOT_IO_U_DO_11 = 41
    self.ROBOT_IO_U_DO_12 = 42
    self.ROBOT_IO_U_DO_13 = 43
    self.ROBOT_IO_U_DO_14 = 44
    self.ROBOT_IO_U_DO_15 = 45
    self.ROBOT_IO_U_DO_16 = 46
    self.ROBOT_IO_U_DO_17 = 47

    -- 接口板用户AI
    self.ROBOT_IO_VI0 = 0
    self.ROBOT_IO_VI1 = 1
    self.ROBOT_IO_VI2 = 2
    self.ROBOT_IO_VI3 = 3

    -- 接口板用户AO
    self.ROBOT_IO_VO0 = 0
    self.ROBOT_IO_VO1 = 1
    self.ROBOT_IO_CO0 = 2
    self.ROBOT_IO_CO1 = 3

    return self
end

return robot