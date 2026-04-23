/*
Test to benchmark the performance of findPathFurthestReachedPoint. 
Currently, the way how I'm using it is:

1. Build and test
*  colcon build --packages-select nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
*  /opt/overlay_ws/build/nav2_mppi_controller/benchmark/utils_benchmark

2. Manually change the findPathFurthestReachedPoint to another implementation 
   (e.g. with the current method currently in main), then run the tests again to compare the results.


This is not an automated benchmark; you have to manually change the implementation and run the tests 
to compare the results. 
Note: the test tries to replicate the actual MPPI noise model and trajectory generation, instead of
using pre-generated/hardcoded candidate trajectories.

*/


#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include <random>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

using namespace mppi::utils;  // NOLINT
using namespace mppi;  // NOLINT

struct ScenarioData
{
  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  Eigen::ArrayXf costs;
  float model_dt;
};

static ScenarioData buildScenario(
  int batch, int time_steps, float prune_distance, float vx_nominal)
{
  ScenarioData s;
  const float path_step = 0.025f;
  const float model_dt = 0.05f;
  s.model_dt = model_dt;
  const int path_pts = static_cast<int>(prune_distance / path_step) + 1;

  // Default MPPI noise & constraint parameters (from nav2_params.yaml)
  const float vx_std = 0.2f;
  const float wz_std = 0.4f;
  const float ax_max_dt = 3.0f * model_dt;   // 0.15 m/s per step
  const float ax_min_dt = -3.0f * model_dt;  // -0.15 m/s per step
  const float az_max_dt = 3.5f * model_dt;   // 0.175 rad/s per step

  // Straight-line path along x-axis
  nav_msgs::msg::Path plan;
  plan.poses.resize(path_pts);
  for (int i = 0; i < path_pts; ++i) {
    plan.poses[i].pose.position.x = path_step * i;
    plan.poses[i].pose.position.y = 0.0;
  }
  s.path = toTensor(plan);

  // Replicate the actual MPPI noise model (DiffDrive, per-step Gaussian + accel clamping)
  // - Control sequence = [vx_nominal, ...] (robot cruising at max speed)
  // - Per-step noise: N(0, vx_std) for vx, N(0, wz_std) for wz (nominal wz = 0)
  // - Acceleration clamping: |vx[t] - vx[t-1]| <= ax_max * dt (matches MotionModel::predict)
  // - DiffDrive integration: x += vx*cos(θ)*dt, y += vx*sin(θ)*dt, θ += wz*dt
  s.trajectories.x = Eigen::ArrayXXf::Zero(batch, time_steps);
  s.trajectories.y = Eigen::ArrayXXf::Zero(batch, time_steps);
  s.trajectories.yaws = Eigen::ArrayXXf::Zero(batch, time_steps);

  std::mt19937 rng(44);  // Fixed seed for reproducibility across runs
  std::normal_distribution<float> vx_noise_dist(0.0f, vx_std);
  std::normal_distribution<float> wz_noise_dist(0.0f, wz_std);

  for (int i = 0; i < batch; ++i) {
    float vx = vx_nominal;   // Initial robot speed (col 0 in real MPPI)
    float wz = 0.0f;
    float x = 0.0f, y = 0.0f, theta = 0.0f;

    for (int t = 0; t < time_steps; ++t) {
      // Noised control = nominal + per-step Gaussian noise
      float cvx = vx_nominal + vx_noise_dist(rng);
      float cwz = wz_noise_dist(rng);

      // Acceleration clamping (matches MotionModel::predict)
      float lower_vx = (vx > 0.0f) ? (vx + ax_min_dt) : (vx - ax_max_dt);
      float upper_vx = (vx > 0.0f) ? (vx + ax_max_dt) : (vx - ax_min_dt);
      cvx = std::clamp(cvx, lower_vx, upper_vx);
      vx = cvx;

      cwz = std::clamp(cwz, wz - az_max_dt, wz + az_max_dt);
      wz = cwz;

      // DiffDrive integration (position uses θ at start of step, matching MPPI)
      x += vx * cosf(theta) * model_dt;
      y += vx * sinf(theta) * model_dt;
      theta += wz * model_dt;

      s.trajectories.x(i, t) = x;
      s.trajectories.y(i, t) = y;
      s.trajectories.yaws(i, t) = theta;
    }
  }

  return s;
}

static void BM_FindFurthest(benchmark::State & state)
{
  const int batch = static_cast<int>(state.range(0));
  const int time_steps = static_cast<int>(state.range(1));
  const float prune_distance = static_cast<float>(state.range(2)) / 10.0f;
  const float vx_nominal = static_cast<float>(state.range(3)) / 100.0f;

  auto scenario = buildScenario(batch, time_steps, prune_distance, vx_nominal);

  CriticData data =
  {scenario.state, scenario.trajectories, scenario.path, scenario.goal,
    scenario.costs, scenario.model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt, {}};

  for (auto _ : state) {
    auto result = findPathFurthestReachedPoint(data);
    benchmark::DoNotOptimize(result);
  }
}

// Args: {batch, time_steps, prune_distance_dm (decimeters), vx_nominal_cm_s}
// Scenario                       batch ts prune vx   
// Default config                 2000  56 2.0m  0.5
BENCHMARK(BM_FindFurthest)->Args({2000, 56, 20, 50})->Iterations(5000)->Unit(benchmark::kMicrosecond);

// Default, smaller batch         batch ts prune vx  
BENCHMARK(BM_FindFurthest)->Args({1000, 56, 20, 50})->Iterations(5000)->Unit(benchmark::kMicrosecond);

// Longer horizon, med batch      batch ts prune vx  
BENCHMARK(BM_FindFurthest)->Args({1000, 80, 30, 50})->Iterations(5000)->Unit(benchmark::kMicrosecond);

// Longer horizon, large batch   batch  ts  prune  vx  
BENCHMARK(BM_FindFurthest)->Args({2000, 80, 30, 50})->Iterations(5000)->Unit(benchmark::kMicrosecond);

// Fast robot, long prune      1000   80  5.0m   1.0
BENCHMARK(BM_FindFurthest)->Args({1000, 80, 50, 100})->Iterations(5000)->Unit(benchmark::kMicrosecond);

// Fast robot, large batch     2000   80  5.0m   1.0
BENCHMARK(BM_FindFurthest)->Args({2000, 80, 50, 100})->Iterations(5000)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
