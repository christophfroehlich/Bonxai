#include "bonxai_server.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace {

class TestBonxaiServer : public bonxai_server::BonxaiServer {
 public:
  using BonxaiServer::BonxaiServer;

  void setMapOptions(const BonxaiT::Options& options) {
    bonxai_->setOptions(options);
  }

  const BonxaiT::Options& mapOptions() const {
    return bonxai_->options();
  }
};

TEST(BonxaiServer, ResetPreservesMapOptions) {
  rclcpp::init(0, nullptr);

  {
    TestBonxaiServer server(rclcpp::NodeOptions{});
    auto options = server.mapOptions();
    options.prob_miss_log = TestBonxaiServer::BonxaiT::logods(0.3F);
    options.prob_hit_log = TestBonxaiServer::BonxaiT::logods(0.85F);
    options.clamp_min_log = TestBonxaiServer::BonxaiT::logods(0.2F);
    options.clamp_max_log = TestBonxaiServer::BonxaiT::logods(0.9F);
    options.occupancy_threshold_log = TestBonxaiServer::BonxaiT::logods(0.6F);
    options.ray_mode = TestBonxaiServer::BonxaiT::Options::RayMode::Approximate;
    server.setMapOptions(options);

    EXPECT_TRUE(server.resetSrv(
        std::make_shared<TestBonxaiServer::ResetSrv::Request>(),
        std::make_shared<TestBonxaiServer::ResetSrv::Response>()));

    const auto& reset_options = server.mapOptions();
    EXPECT_EQ(reset_options.prob_miss_log, options.prob_miss_log);
    EXPECT_EQ(reset_options.prob_hit_log, options.prob_hit_log);
    EXPECT_EQ(reset_options.clamp_min_log, options.clamp_min_log);
    EXPECT_EQ(reset_options.clamp_max_log, options.clamp_max_log);
    EXPECT_EQ(reset_options.occupancy_threshold_log, options.occupancy_threshold_log);
    EXPECT_EQ(reset_options.ray_mode, options.ray_mode);
  }

  rclcpp::shutdown();
}

}  // namespace
