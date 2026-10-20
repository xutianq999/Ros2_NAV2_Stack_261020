#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <functional>
#include <string>
#include <vector>

// Execute the shipped tree with Nav2's real control nodes and fake action
// endpoints. This test never creates ROS nodes or publishes motion commands.
class FakeAction : public BT::StatefulActionNode
{
public:
  using Tick = std::function<BT::NodeStatus(const BT::NodeConfiguration &)>;
  FakeAction(const std::string & name, const BT::NodeConfiguration & config,
    Tick tick, std::function<void()> halt)
  : BT::StatefulActionNode(name, config), tick_(tick), halt_(halt) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal"), BT::OutputPort<std::string>("path"),
      BT::InputPort<std::string>("planner_id"), BT::InputPort<std::string>("controller_id"),
      BT::InputPort<std::string>("service_name"),
      BT::InputPort<std::string>("server_name", "backup"),
      BT::InputPort<std::string>("backup_dist"), BT::InputPort<std::string>("backup_speed"),
      BT::InputPort<std::string>("time_allowance"), BT::InputPort<std::string>("wait_duration")};
  }

  BT::NodeStatus onStart() override {return tick_(config());}
  BT::NodeStatus onRunning() override {return tick_(config());}
  void onHalted() override {halt_();}

private:
  Tick tick_;
  std::function<void()> halt_;
};

class RecoveryTreeTest : public testing::Test
{
public:
  BT::BehaviorTreeFactory factory;
  std::vector<std::string> motion;
  BT::NodeStatus smart{BT::NodeStatus::SUCCESS}, backup{BT::NodeStatus::SUCCESS};
  int waits{0}, halts{0};

  void SetUp() override
  {
    for (const auto & plugin : {"nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node", "nav2_rate_controller_bt_node"})
    {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }
    factory.registerSimpleCondition("GoalUpdated", [](BT::TreeNode &) {
      return BT::NodeStatus::FAILURE;
    });
    for (const std::string tag : {"ComputePathToPose", "FollowPath",
      "ClearEntireCostmap", "BackUp", "Wait"})
    {
      factory.registerBuilder<FakeAction>(tag,
        [this, tag](const std::string & name, const BT::NodeConfiguration & config) {
          return std::make_unique<FakeAction>(name, config,
            [this, tag](const BT::NodeConfiguration & conf) {
              if (tag == "FollowPath") {return BT::NodeStatus::FAILURE;}
              if (tag == "Wait") {++waits;}
              if (tag == "BackUp") {
                const auto it = conf.input_ports.find("server_name");
                const std::string action = it == conf.input_ports.end() ? "backup" : it->second;
                motion.push_back(action);
                return action == "smart_recovery" ? smart : backup;
              }
              return BT::NodeStatus::SUCCESS;
            }, [this] {++halts;});
        });
    }
  }

  BT::Tree tree() {return factory.createTreeFromFile(RECOVERY_TREE_PATH);}
  BT::NodeStatus finish(BT::Tree & t)
  {
    for (int i = 0; i < 100; ++i) {
      const auto result = t.tickRoot();
      if (result != BT::NodeStatus::RUNNING) {return result;}
    }
    return BT::NodeStatus::RUNNING;
  }
};

TEST_F(RecoveryTreeTest, EveryRetryAttemptsSmartMotion)
{
  auto t = tree();
  EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motion, std::vector<std::string>(4, "smart_recovery"));
  EXPECT_EQ(waits, 0);
}

TEST_F(RecoveryTreeTest, SmartFailureFallsBackToCheckedBackupEveryTime)
{
  smart = BT::NodeStatus::FAILURE;
  auto t = tree();
  EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
  ASSERT_EQ(motion.size(), 8u);
  for (size_t i = 0; i < motion.size(); i += 2) {
    EXPECT_EQ(motion[i], "smart_recovery");
    EXPECT_EQ(motion[i + 1], "backup");
  }
  EXPECT_EQ(waits, 0);
}

TEST_F(RecoveryTreeTest, BlockedMotionWaitsAndStopsAfterBoundedRetries)
{
  smart = backup = BT::NodeStatus::FAILURE;
  auto t = tree();
  EXPECT_EQ(finish(t), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motion.size(), 8u);
  EXPECT_EQ(waits, 4);
}

TEST_F(RecoveryTreeTest, CancelHaltsActiveMotionWithoutStartingBackup)
{
  smart = BT::NodeStatus::RUNNING;
  auto t = tree();
  EXPECT_EQ(t.tickRoot(), BT::NodeStatus::RUNNING);
  t.haltTree();
  EXPECT_EQ(halts, 1);
  EXPECT_EQ(motion, std::vector<std::string>{"smart_recovery"});
}
