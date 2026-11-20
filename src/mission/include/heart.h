#pragma once

/************************************************************************
 * heart.h —— 爱心任务专用函数
 *
 * 用法（与工程里 avoidance.cpp 用 avoidance.h 的方式一致）：
 *     #include <template.h>
 *     #include <heart.h>
 *
 * 本文件直接使用 template.h 里的全局量（setpoint_raw / local_pos /
 * current_state / init_position_* / yaw / mission_pos_cruise / precision_land），
 * 因此不重复 include <template.h> —— template.h 没有 include guard，
 * 重复包含会导致全局变量重复定义。
 ************************************************************************/

#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HEART_DS 0.02f  // 爱心航点等弧长间距 [m]

/************************************************************************
 * 函数 H1: 心形线原始参数方程
 *     sx(t) = 16 sin^3(t)                                      左右（宽度）
 *     sy(t) = 13cos(t) - 5cos(2t) - 2cos(3t) - cos(4t)         上下（对称轴）
 *
 * ★ 竖向爱心：画在 xz 竖直平面 ★
 *   sy（尖-瓣对称轴）映射到飞行 z（竖直），sx 映射到飞行 x（水平前后），
 *   y 固定在起飞点。从侧面（沿 y 看）是一枚真正立着的爱心：心尖朝下、
 *   两个圆瓣朝上；不是躺在水平面里的那种。
 ************************************************************************/
static inline double heart_sample_sx(double t)
{
  const double s = sin(t);
  return 16.0 * s * s * s;
}
static inline double heart_sample_sy(double t)
{
  return 13.0 * cos(t) - 5.0 * cos(2.0 * t) - 2.0 * cos(3.0 * t) - cos(4.0 * t);
}

struct HeartPath
{
  std::vector<float> fx;  // 竖直方向（对称轴/高度），画心时映射到 z [m]
  std::vector<float> fy;  // 水平方向（宽度），画心时映射到 x [m]
  float ds = HEART_DS;    // 实际航点间距 [m]
  float length = 0.0f;    // 周长 [m]
};

/************************************************************************
 * 函数 H2: 生成爱心航点
 *   密采样 -> 分别按 width / height 缩放 -> 按等弧长重采样
 *   等弧长是为了匀速：t 均匀采样会在心尖附近挤点、圆瓣附近稀点，飞出来不匀。
 *
 *   ★ 以 t=0 的心口点为原点（不是几何中心）：
 *     心形线 t=0 与 t=2π 是同一个点，把这点对齐到 (0,0) 后，
 *     画完一圈自然回到起飞点，不需要额外飞回段。
 ************************************************************************/
static HeartPath make_heart(float width, float height, float ds)
{
  const int N = 4000;
  std::vector<double> sx(N + 1), sy(N + 1);
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;

  for (int i = 0; i <= N; ++i)
  {
    const double t = 2.0 * M_PI * i / N;
    sx[i] = heart_sample_sx(t);
    sy[i] = heart_sample_sy(t);
    xmin = std::min(xmin, sx[i]);
    xmax = std::max(xmax, sx[i]);
    ymin = std::min(ymin, sy[i]);
    ymax = std::max(ymax, sy[i]);
  }

  const double kx = width / (xmax - xmin);    // 宽度比例（映射到飞行 x）
  const double ky = height / (ymax - ymin);   // 高度比例（映射到飞行 z）
  const double x0 = sx[0];                    // t=0 的左右值 (=0)
  const double y0 = sy[0];                    // t=0 的前后值 (=5)

  std::vector<float> acc(N + 1, 0.0f);
  for (int i = 1; i <= N; ++i)
  {
    const double d_fx = (sy[i] - sy[i - 1]) * ky;
    const double d_fy = (sx[i] - sx[i - 1]) * kx;
    acc[i] = acc[i - 1] + (float)std::hypot(d_fx, d_fy);
  }

  HeartPath p;
  p.length = acc[N];
  const int M = std::max(8, (int)std::lround(p.length / ds));
  p.ds = p.length / M;
  p.fx.reserve(M + 1);
  p.fy.reserve(M + 1);

  int j = 0;
  for (int m = 0; m <= M; ++m)
  {
    const float target = p.length * m / M;
    while (j + 1 < N && acc[j + 1] < target) ++j;
    const float seg = acc[j + 1] - acc[j];
    const float w = (seg > 1e-9f) ? (target - acc[j]) / seg : 0.0f;
    const double fy = sx[j] + w * (sx[j + 1] - sx[j]);
    const double fx = sy[j] + w * (sy[j + 1] - sy[j]);
    p.fx.push_back((float)((fx - y0) * ky));  // 竖直 = 高度，t=0 时为 0
    p.fy.push_back((float)((fy - x0) * kx));  // 水平 = 宽度，t=0 时为 0
  }

  ROS_INFO("Heart path: %d waypoints, height(z)=%.2f m, width(x)=%.2f m, perimeter=%.2f m, step=%.3f m",
           (int)p.fx.size(), height, width, p.length, p.ds);
  return p;
}

/************************************************************************
 * 函数 H3: 沿爱心轨迹发设定点（画完一圈、回到起点并停稳后返回 true）
 *   type_mask / coordinate_frame / init 偏移 全部与 mission_pos_cruise 一致。
 *
 *   收尾：发完最后一个点后持续把目标钉在起点，等水平速度收敛再返回，
 *   这样降落时 precision_land 锁的落点就是出发点，下降垂直。
 ************************************************************************/
static bool heart_fly(const HeartPath &p, float ox, float oy, float z, float speed)
{
  static bool started = false;
  static ros::Time t0;

  if (!started)
  {
    t0 = ros::Time::now();
    started = true;
    ROS_INFO("Start drawing heart ...");
  }

  const float moved = speed * (ros::Time::now() - t0).toSec();
  const int last = (int)p.fx.size() - 1;
  int idx = (int)(moved / p.ds);
  if (idx > last) idx = last;   // 画完后目标钉在最后一个点（=起点）

  setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
  setpoint_raw.coordinate_frame = 1;
  // xz 平面：宽度(fy)→x 水平，对称轴(fx)→z 竖直，y 固定在起飞点
  setpoint_raw.position.x = p.fy[idx] + ox + init_position_x_take_off;
  setpoint_raw.position.y = oy + init_position_y_take_off;
  setpoint_raw.position.z = z + p.fx[idx] + init_position_z_take_off;
  setpoint_raw.yaw = 0.0;

  ROS_INFO_THROTTLE(1.0, "Heart %.0f%%  target(%.2f, %.2f, %.2f)",
                    100.0 * moved / p.length,
                    setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);

  if (idx >= last)
  {
    const geometry_msgs::Twist &v = local_pos.twist.twist;
    const float v3d = (float)std::sqrt(v.linear.x * v.linear.x +
                                       v.linear.y * v.linear.y +
                                       v.linear.z * v.linear.z);
    if (v3d < 0.08f || (ros::Time::now() - t0).toSec() > p.length / speed + 8.0)
    {
      started = false;
      ROS_INFO("Heart finished in %.1f s, back at start and settled (v3d=%.3f)",
               (ros::Time::now() - t0).toSec(), v3d);
      return true;
    }
  }
  return false;
}

/************************************************************************
 * 函数 H4: 等飞机停稳（降落前用）
 *   precision_land() 第一次调用会把「当前 xy」锁成落点；若此时还在横向漂移，
 *   落点就锁偏了，下降看起来不垂直。hover.cpp 是干等 30 秒，这里按速度判断。
 ************************************************************************/
static bool heart_settle(float v_eps = 0.08f, double timeout_s = 8.0)
{
  static bool started = false;
  static ros::Time t0;

  if (!started)
  {
    t0 = ros::Time::now();
    started = true;
    ROS_INFO("Waiting for velocity to converge before landing ...");
  }

  const geometry_msgs::Twist &v = local_pos.twist.twist;
  const float vxy = (float)std::hypot(v.linear.x, v.linear.y);
  const bool done = (vxy < v_eps) || ((ros::Time::now() - t0).toSec() > timeout_s);

  if (done)
  {
    started = false;
    ROS_INFO("Settled (vxy=%.3f m/s)", vxy);
  }
  return done;
}

/************************************************************************
 * 函数 H5: 落地后停桨（轮询到 armed == false 才算任务结束）
 *
 *   hover.cpp 降落完直接 exit(0)：节点一退出没人发设定点，PX4 在
 *   COM_OF_LOSS_T 后触发 OFFBOARD 失控保护，桨叶会继续转。
 *
 *   安全约束：
 *     - 只有相对高度 < 0.25 m 才允许上锁（空中上锁 = 摔机）
 *     - 空中超过 5 秒还没落地 -> 切 AUTO.LAND 交给飞控
 *     - 全程持续发设定点，避免掉进失控保护
 ************************************************************************/
static bool heart_stop_motors(ros::Publisher &pub, ros::ServiceClient &setmode,
                              ros::ServiceClient &arming, ros::ServiceClient &cmd,
                              ros::Rate &rate)
{
  mavros_msgs::CommandBool arm_false;
  arm_false.request.value = false;

  mavros_msgs::SetMode land_mode;
  land_mode.request.custom_mode = "AUTO.LAND";

  const ros::Time t0 = ros::Time::now();
  ros::Time t_last = t0;
  bool auto_land_sent = false;
  bool told_waiting = false;
  bool told_timeout = false;

  // 强制上锁：只有确认贴地(高度 < 0.5 m)才调用，空中上锁等于摔机。
  // 放 while 外，供 1 秒循环和超时兜底共用。
  auto try_disarm = [&](double hh) {
    if (arming.call(arm_false) && arm_false.response.success)
      ROS_INFO("Disarm success (h=%.2f m)", hh);
    else
    {
      mavros_msgs::CommandLong force;
      force.request.command = 400;      // MAV_CMD_COMPONENT_ARM_DISARM
      force.request.param1 = 0.0f;      // 0 = disarm
      force.request.param2 = 21196.0f;  // 21196 = 强制
      if (cmd.call(force) && force.response.success)
        ROS_WARN("Force disarm success (h=%.2f m)", hh);
    }
  };

  while (ros::ok() && current_state.armed)
  {
    const double dt = (ros::Time::now() - t0).toSec();
    const double h = local_pos.pose.pose.position.z - init_position_z_take_off;

    if ((ros::Time::now() - t_last).toSec() > 1.0)
    {
      t_last = ros::Time::now();

      if (h < 0.25)
      {
        // 已贴地：先普通上锁，失败再强制
        try_disarm(h);
      }
      else
      {
        if (!told_waiting)
        {
          ROS_INFO("Still %.2f m above ground, waiting to land ...", h);
          told_waiting = true;
        }
        if (!auto_land_sent && dt > 5.0)
        {
          if (setmode.call(land_mode) && land_mode.response.mode_sent)
          {
            auto_land_sent = true;
            ROS_WARN("Switch to AUTO.LAND, let the FCU land and disarm");
          }
        }
      }
    }

    if (dt > 20.0 && h < 0.5)
    {
      // 超时兜底：已贴地就直接强制上锁，不再等。
      // 关键：绝不 return false 让主程序 exit(0)——节点一退出没人发设定点，
      // PX4 会触发 OFFBOARD 失控保护，桨叶继续转。这里循环到 armed=false 才退出。
      if (!told_timeout)
      {
        ROS_WARN("Stop motors timeout, force disarm since already on ground (h=%.2f m)", h);
        told_timeout = true;
      }
      try_disarm(h);
    }
    else if (dt > 20.0 && !told_timeout)
    {
      ROS_ERROR("Still %.2f m high after 20 s, waiting for AUTO.LAND. Do NOT kill this node.", h);
      told_timeout = true;
    }

    pub.publish(setpoint_raw);  // 持续发，别掉进失控保护
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Motors stopped (armed = false)");
  return true;
}
