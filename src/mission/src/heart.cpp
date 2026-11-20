#include <template.h>
#include <heart.h>

// 全局变量定义（与 hover.cpp 一致）
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;

// 爱心任务参数（yaml 可覆盖）
float heart_altitude = 1.2f;  // 画心起点高度 [m]，相对初始位置（心尖会下探约 0.76 height）
float heart_width = 0.8f;     // 爱心水平宽度 [m]（映射到飞行 x）
float heart_height = 1.0f;    // 爱心竖直高度 [m]（映射到飞行 z，> width 才是竖向）
float heart_offset_x = 0.0f;  // 爱心中心相对起飞点的前后偏移 [m]
float heart_offset_y = 0.0f;  // 爱心中心相对起飞点的左右偏移 [m]
float heart_speed = 0.30f;    // 画心速度 [m/s]
float heart_hold = 3.0f;      // 画完悬停时间 [s]

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "heart_altitude: " << heart_altitude << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  std::cout << "heart: width=" << heart_width << " height=" << heart_height
            << " speed=" << heart_speed << " hold=" << heart_hold
            << " offset=(" << heart_offset_x << ", " << heart_offset_y << ")" << std::endl;
  if (if_debug == 1)
    cout << "自动offboard" << endl;
  else
    cout << "遥控器offboard" << endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "heart");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取
  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("altitude", heart_altitude, heart_altitude);
  nh.param<float>("width", heart_width, heart_width);
  nh.param<float>("height", heart_height, heart_height);
  nh.param<float>("offset_x", heart_offset_x, heart_offset_x);
  nh.param<float>("offset_y", heart_offset_y, heart_offset_y);
  nh.param<float>("speed", heart_speed, heart_speed);
  nh.param<float>("hold_time", heart_hold, heart_hold);
  print_param();

  // ★ 起飞前置条件：必须敲 1，否则直接退出（与 hover.cpp 一致）
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待连接到飞控（与 hover.cpp 一致）
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  // 第一段：切 OFFBOARD / 解锁 / 飞到目标高度（照搬 hover.cpp）
  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if (if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        // if_debug=0：必须由遥控器切到 OFFBOARD 才起飞（hover.cpp 原逻辑）
        ROS_INFO("Waiting for OFFBOARD mode (switch by RC)");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    // ★ 实机关键：mission_pos_cruise 发的是 (heart_altitude + init_position_z_take_off)，
    //   所以这里必须减掉 init 偏移。仿真里 init_z=0 掩盖了这个问题；实机 odom 原点
    //   若不在地面（例如气压计/绝对高度），不减就永远到不了，程序卡死在这里。
    if (fabs(local_pos.pose.pose.position.z - init_position_z_take_off - heart_altitude) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, heart_altitude, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  /* ---- 爱心航点（唯一新增的数据准备）---- */
  const HeartPath heart = make_heart(heart_width, heart_height, HEART_DS);

  // precision_land() 的 type_mask 是 960，没有屏蔽 velocity / yaw_rate 字段，
  // 先清零，免得上一个任务留下的速度把飞机往横向带（保证垂直降落）
  setpoint_raw.velocity.x = 0.0;
  setpoint_raw.velocity.y = 0.0;
  setpoint_raw.velocity.z = 0.0;
  setpoint_raw.yaw_rate = 0.0;

  // 第二段：任务状态机（hover.cpp 的 case 1/2/3，中间插入画爱心）
  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);

    switch (mission_num)
    {
      // mission1: 起飞（到目标高度并稳定 1 秒，照搬 hover.cpp）
      case 1:
        if (mission_pos_cruise(0, 0, heart_altitude, 0, err_max))
        {
          if (lib_time_record_func(1.0, ros::Time::now()))
          {
            mission_num = 2;
            last_request = ros::Time::now();
          }
        }
        else if (ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      // mission2: 飞到爱心起点（心口凹处）
      case 2:
        if (mission_pos_cruise(heart.fx[0] + heart_offset_x,
                               heart.fy[0] + heart_offset_y,
                               heart_altitude, 0, err_max))
        {
          ROS_INFO("Reached heart start point, drawing heart now");
          mission_num = 3;
        }
        break;

      // mission3: 画爱心（替代 hover.cpp 的 hover(30.0)）
      case 3:
        if (heart_fly(heart, heart_offset_x, heart_offset_y, heart_altitude, heart_speed))
        {
          mission_num = 4;
        }
        break;

      // mission4: 悬停
      case 4:
        if (hover(heart_hold))
        {
          mission_num = 5;
        }
        break;

      // mission5: 等速度收敛（收敛完才能锁准降落点，保证垂直降落）
      case 5:
        if (heart_settle())
        {
          mission_num = 6;
        }
        break;

      // mission6: 垂直降落
      case 6:
        if (precision_land())
        {
          mission_num = 7;
        }
        break;

      // mission7: 落地后停桨（hover.cpp 没有这一步，是为了满足"桨叶停转"新增）
      case 7:
        heart_stop_motors(mavros_setpoint_pos_pub, set_mode_client,
                          arming_client, ctrl_pwm_client, rate);
        mission_num = -1;  // 任务结束
        break;

      default:
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();

    if (mission_num == -1)
    {
      exit(0);
    }
  }
  return 0;
}
