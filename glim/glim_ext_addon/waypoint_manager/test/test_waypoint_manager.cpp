#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <glim_ext/waypoint_manager_module.hpp>
#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <thread>
#include <vector>

class WaypointManagerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  static glim::EstimationFrame::Ptr create_frame(const Eigen::Vector3d& translation, const double stamp) {
    auto frame = std::make_shared<glim::EstimationFrame>();
    frame->frame_id = glim::FrameID::LIDAR;
    frame->stamp = stamp;
    frame->T_lidar_imu = Eigen::Isometry3d::Identity();
    frame->T_world_lidar = Eigen::Translation3d(translation) * Eigen::Quaterniond::Identity();
    frame->T_world_imu = frame->T_world_lidar;
    return frame;
  }

  static glim::SubMap::Ptr create_submap(
    const int id,
    const Eigen::Isometry3d& T_map_origin,
    const Eigen::Vector3d& odom_origin,
    const double first_stamp = 0.0,
    const double last_stamp = 100.0) {
    auto submap = std::make_shared<glim::SubMap>();
    submap->id = id;
    submap->T_world_origin = T_map_origin;
    // origin_frame() is the middle frame.  The two fixture frames deliberately
    // use the same pose so the expected local transform is unambiguous.
    submap->frames = {create_frame(odom_origin, first_stamp), create_frame(odom_origin, last_stamp)};
    return submap;
  }

  static void register_submap(glim::WaypointManager& manager, const glim::SubMap::Ptr& submap) {
    manager.on_new_submap(submap);
    manager.on_insert_submap(submap);
  }
};

TEST_F(WaypointManagerTest, RejectsWhenNoOdometryFrame) {
  glim::WaypointManager manager;
  auto req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  req->name = "early_waypoint";
  auto res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();

  manager.add_waypoint_cb(req, res);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("no odometry yet"), std::string::npos);
}

TEST_F(WaypointManagerTest, AddsAndResolvesWaypointInMapFrame) {
  glim::WaypointManager manager;
  auto submap = create_submap(
    0, Eigen::Translation3d(0.5, 1.0, 0.0) * Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.5, 1.0, 0.0));
  register_submap(manager, submap);

  manager.on_update_new_frame(create_frame(Eigen::Vector3d(1.0, 2.0, 0.5), 10.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "wp_alpha";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);

  ASSERT_TRUE(add_res->success);
  ASSERT_EQ(manager.count(), 1u);
  const auto& wp = manager.get_waypoints().front();
  EXPECT_EQ(wp.submap_id, 0);
  EXPECT_NEAR(wp.T_submap_sensor.translation().x(), 0.5, 1e-5);
  EXPECT_NEAR(wp.T_submap_sensor.translation().y(), 1.0, 1e-5);
  EXPECT_NEAR(wp.T_submap_sensor.translation().z(), 0.5, 1e-5);

  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "wp_alpha";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  EXPECT_EQ(get_res->pose.header.frame_id, "map");
  EXPECT_NEAR(get_res->pose.pose.position.x, 1.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 2.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.z, 0.5, 1e-5);
}

TEST_F(WaypointManagerTest, LoopClosureSubmapUpdateShiftsWorldPose) {
  glim::WaypointManager manager;
  auto submap = create_submap(
    0, Eigen::Translation3d(1.0, 1.0, 0.0) * Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 1.0, 0.0));
  register_submap(manager, submap);
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(3.0, 2.0, 0.0), 10.0));

  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "loop_test_wp";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);

  submap->T_world_origin = Eigen::Translation3d(6.0, -1.0, 0.0) * Eigen::Quaterniond::Identity();
  manager.on_update_submaps({submap});
  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "loop_test_wp";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  EXPECT_NEAR(get_res->pose.pose.position.x, 8.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 0.0, 1e-5);
}

TEST_F(WaypointManagerTest, TagAfterExistingCorrectionUsesCorrectedMapCoordinates) {
  glim::WaypointManager manager;
  auto submap = create_submap(
    0, Eigen::Translation3d(1.0, 1.0, 0.0) * Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 1.0, 0.0));
  register_submap(manager, submap);

  // The old code would return the raw odometry pose (3, 2), not (8, 0).
  submap->T_world_origin = Eigen::Translation3d(6.0, -1.0, 0.0) * Eigen::Quaterniond::Identity();
  manager.on_update_submaps({submap});
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(3.0, 2.0, 0.0), 10.0));

  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "corrected_before_tag";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);

  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "corrected_before_tag";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  EXPECT_NEAR(get_res->pose.pose.position.x, 8.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 0.0, 1e-5);
}

TEST_F(WaypointManagerTest, QueuedWaypointBindsWhenItsSubmapFinalizes) {
  glim::WaypointManager manager;
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(3.0, 0.0, 0.0), 10.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "pending_wp";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);
  EXPECT_NE(add_res->message.find("queued"), std::string::npos);
  EXPECT_EQ(manager.count(), 0u);

  auto submap = create_submap(0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0), 5.0, 15.0);
  register_submap(manager, submap);
  ASSERT_EQ(manager.count(), 1u);
  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "pending_wp";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  EXPECT_NEAR(get_res->pose.pose.position.x, 2.0, 1e-5);
}

TEST_F(WaypointManagerTest, DuplicateWaypointUpdatesExisting) {
  glim::WaypointManager manager;
  auto submap = create_submap(0, Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
  register_submap(manager, submap);
  auto req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  req->name = "duplicate_wp";
  auto res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(1.0, 1.0, 0.0), 10.0));
  manager.add_waypoint_cb(req, res);
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(5.0, 5.0, 0.0), 20.0));
  manager.add_waypoint_cb(req, res);

  ASSERT_TRUE(res->success);
  EXPECT_EQ(manager.count(), 1u);
  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "duplicate_wp";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  EXPECT_NEAR(get_res->pose.pose.position.x, 5.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 5.0, 1e-5);
}

TEST_F(WaypointManagerTest, ListAndSaveWaypoints) {
  glim::WaypointManager manager;
  auto submap = create_submap(0, Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
  register_submap(manager, submap);
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(2.5, 3.5, 1.0), 10.0));
  auto req1 = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  req1->name = "wp_one";
  auto res1 = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(req1, res1);
  auto req2 = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  req2->name = "wp_two";
  auto res2 = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(req2, res2);

  auto list_req = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Request>();
  auto list_res = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Response>();
  manager.list_waypoints_cb(list_req, list_res);
  ASSERT_EQ(list_res->names.size(), 2u);
  EXPECT_EQ(list_res->names[0], "wp_one");
  EXPECT_EQ(list_res->names[1], "wp_two");

  const std::string test_yaml = "/tmp/test_waypoint_output.yaml";
  std::remove(test_yaml.c_str());
  auto save_req = std::make_shared<waypoint_interfaces::srv::SaveWaypoints::Request>();
  save_req->path = test_yaml;
  auto save_res = std::make_shared<waypoint_interfaces::srv::SaveWaypoints::Response>();
  manager.save_waypoints_cb(save_req, save_res);
  ASSERT_TRUE(save_res->success);
  YAML::Node yaml = YAML::LoadFile(test_yaml);
  ASSERT_TRUE(yaml["waypoints"].IsSequence());
  EXPECT_EQ(yaml["waypoints"].size(), 2u);
  EXPECT_NEAR(yaml["waypoints"][0]["local_xyz"][0].as<double>(), 2.5, 1e-4);
  EXPECT_NEAR(yaml["waypoints"][0]["map_xyz"][1].as<double>(), 3.5, 1e-4);
  std::remove(test_yaml.c_str());
}

TEST_F(WaypointManagerTest, NotFoundForUnknownWaypointAndRejectsEmptyName) {
  glim::WaypointManager manager;
  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "non_existent";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  EXPECT_FALSE(get_res->found);

  manager.on_update_new_frame(create_frame(Eigen::Vector3d::Zero(), 1.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  EXPECT_FALSE(add_res->success);
}

// Regression test: tagging a waypoint inside the still-forming submap (no
// submap finalized yet to bind it against) used to vanish with no trace if
// the session ended before a new submap arrived - write_yaml only looked at
// `waypoints`, never `pending_waypoints`. This is the realistic "tag the
// last waypoint right before stopping" case from the actual mission workflow.
TEST_F(WaypointManagerTest, PendingWaypointSurvivesSaveAndListsAsPending) {
  glim::WaypointManager manager;
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(4.0, -2.0, 0.0), 10.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "last_second_wp";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);
  ASSERT_EQ(manager.count(), 0u);  // still pending, no submap to bind to

  auto list_req = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Request>();
  auto list_res = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Response>();
  manager.list_waypoints_cb(list_req, list_res);
  ASSERT_EQ(list_res->names.size(), 1u);
  EXPECT_EQ(list_res->names[0], "last_second_wp (pending)");

  const std::string test_yaml = "/tmp/test_pending_waypoint_output.yaml";
  std::remove(test_yaml.c_str());
  auto save_req = std::make_shared<waypoint_interfaces::srv::SaveWaypoints::Request>();
  save_req->path = test_yaml;
  auto save_res = std::make_shared<waypoint_interfaces::srv::SaveWaypoints::Response>();
  manager.save_waypoints_cb(save_req, save_res);
  ASSERT_TRUE(save_res->success);

  YAML::Node yaml = YAML::LoadFile(test_yaml);
  ASSERT_TRUE(yaml["waypoints"].IsSequence());
  EXPECT_EQ(yaml["waypoints"].size(), 0u);
  ASSERT_TRUE(yaml["pending_waypoints"].IsSequence());
  ASSERT_EQ(yaml["pending_waypoints"].size(), 1u);
  EXPECT_EQ(yaml["pending_waypoints"][0]["name"].as<std::string>(), "last_second_wp");
  EXPECT_NEAR(yaml["pending_waypoints"][0]["odom_xyz"][0].as<double>(), 4.0, 1e-5);
  EXPECT_NEAR(yaml["pending_waypoints"][0]["odom_xyz"][1].as<double>(), -2.0, 1e-5);
  std::remove(test_yaml.c_str());
}

// get_waypoint_cb only scans the bound `waypoints` vector, never
// `pending_waypoints`.  A tag made near the tail of the still-open submap
// (the exact "tag right before the flag, submap not finalized yet" mission
// case) is therefore genuinely invisible to /get_waypoint until the next
// on_new_submap fires and binds it.  waypoint_mission.py polls /get_waypoint
// every 2s, so this is the observable behavior an operator would see.
TEST_F(WaypointManagerTest, GetWaypointForStillPendingNameReportsNotFound) {
  glim::WaypointManager manager;
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(1.0, 1.0, 0.0), 10.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "flag_wp";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);
  ASSERT_EQ(manager.count(), 0u);  // pending: no submap covers stamp=10 yet

  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "flag_wp";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  EXPECT_FALSE(get_res->found);
}

// find_submap_for_stamp_locked (used for both live tagging and pending-queue
// binding) walks submap_references newest-first and returns the first whose
// [first_stamp, last_stamp] covers the query stamp.  With more than one
// finalized submap in flight, a pending waypoint must land in the submap
// whose *stamp range* actually covers it - not merely the most recently
// finalized one - and its world pose must come from that submap specifically.
TEST_F(WaypointManagerTest, PendingWaypointBindsToCorrectSubmapAmongMultipleCandidates) {
  glim::WaypointManager manager;

  // Tag while stamp=7 is covered by no submap yet -> queued.
  manager.on_update_new_frame(create_frame(Eigen::Vector3d(12.0, 1.0, 0.0), 7.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "mid_flag";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);
  ASSERT_EQ(manager.count(), 0u);

  // Submap A covers [0,5] - does not include stamp=7, must NOT bind to it.
  auto submap_a = create_submap(
    0, Eigen::Translation3d(1000.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 5.0);
  register_submap(manager, submap_a);
  EXPECT_EQ(manager.count(), 0u) << "stamp=7 is outside submap A's [0,5] range";

  // Submap B covers [6,10] - includes stamp=7, must bind here.
  auto submap_b = create_submap(
    1, Eigen::Translation3d(100.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(), Eigen::Vector3d(10.0, 0.0, 0.0), 6.0, 10.0);
  register_submap(manager, submap_b);
  ASSERT_EQ(manager.count(), 1u);
  EXPECT_EQ(manager.get_waypoints().front().submap_id, 1);

  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "mid_flag";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  ASSERT_TRUE(get_res->found);
  // local = inv(odom_origin_B=(10,0,0)) * (12,1,0) = (2,1,0)
  // world  = T_world_origin_B=(100,0,0) * (2,1,0) = (102,1,0)
  EXPECT_NEAR(get_res->pose.pose.position.x, 102.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 1.0, 1e-5);
}

// on_new_submap (SubMappingCallbacks) and on_insert_submap (GlobalMappingCallbacks)
// are two independent callback slots fired by two different pipeline stages
// (glim/mapping/callbacks.hpp:79 vs :110). A submap can therefore be finalized
// (and a waypoint bound to it, moving it out of `pending_waypoints`) before the
// global mapper has registered its world pose in `submap_world_pose`. During
// that window get_waypoint_cb's `break` on a missing submap_world_pose entry
// (waypoint_manager_module.hpp:204-205) falls through to `res->found = false`,
// so an operator who tags a waypoint and immediately polls /get_waypoint (as
// waypoint_mission.py does every 2s) can see a spurious "not found" for a
// waypoint that IS bound. This documents the current behavior; see report for
// whether it's considered acceptable.
TEST_F(WaypointManagerTest, GetWaypointNotFoundWhileSubmapWorldPoseNotYetInserted) {
  glim::WaypointManager manager;
  auto submap = create_submap(0, Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
  manager.on_new_submap(submap);  // finalized, but NOT yet on_insert_submap

  manager.on_update_new_frame(create_frame(Eigen::Vector3d(3.0, 4.0, 0.0), 10.0));
  auto add_req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
  add_req->name = "race_wp";
  auto add_res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
  manager.add_waypoint_cb(add_req, add_res);
  ASSERT_TRUE(add_res->success);
  ASSERT_EQ(manager.count(), 1u) << "waypoint is bound to the submap already";

  auto get_req = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Request>();
  get_req->name = "race_wp";
  auto get_res = std::make_shared<waypoint_interfaces::srv::GetWaypoint::Response>();
  manager.get_waypoint_cb(get_req, get_res);
  EXPECT_FALSE(get_res->found) << "world pose not registered yet - current code reports not-found here";

  // Once the global mapper catches up, it resolves normally.
  manager.on_insert_submap(submap);
  manager.get_waypoint_cb(get_req, get_res);
  EXPECT_TRUE(get_res->found);
  EXPECT_NEAR(get_res->pose.pose.position.x, 3.0, 1e-5);
  EXPECT_NEAR(get_res->pose.pose.position.y, 4.0, 1e-5);
}

// Cheap concurrency smoke test: add_waypoint_cb, on_update_new_frame and
// list_waypoints_cb are all called through the same mtx.  Hammer them from
// several threads and check the invariant that every successfully-added
// name is exactly once in the union of bound + pending waypoints, with no
// crash / UB under TSAN-less CI. Deterministic in outcome (not timing).
TEST_F(WaypointManagerTest, ConcurrentAddAndListDoNotRaceOrCrash) {
  glim::WaypointManager manager;
  constexpr int kThreads = 4;
  constexpr int kPerThread = 50;

  std::vector<std::thread> workers;
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&manager, t]() {
      for (int i = 0; i < kPerThread; ++i) {
        manager.on_update_new_frame(create_frame(Eigen::Vector3d(t, i, 0.0), t * 1000.0 + i));
        auto req = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Request>();
        req->name = "wp_" + std::to_string(t) + "_" + std::to_string(i);
        auto res = std::make_shared<waypoint_interfaces::srv::AddWaypoint::Response>();
        manager.add_waypoint_cb(req, res);

        auto list_req = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Request>();
        auto list_res = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Response>();
        manager.list_waypoints_cb(list_req, list_res);
      }
    });
  }
  for (auto& w : workers) w.join();

  auto list_req = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Request>();
  auto list_res = std::make_shared<waypoint_interfaces::srv::ListWaypoints::Response>();
  manager.list_waypoints_cb(list_req, list_res);
  EXPECT_EQ(list_res->names.size(), static_cast<size_t>(kThreads * kPerThread));
}
