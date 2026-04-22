#pragma once

#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "game_controller_hl_interfaces/msg/game_state.hpp"

namespace stride::op3
{

class GameStateListener
{
public:
  using GameStateMsg = game_controller_hl_interfaces::msg::GameState;

  struct Config
  {
    std::string topic{"/gamestate"};
    int init_pose_page{80};
    int walk_ready_page{9};
    bool use_default_transitions{true};
  };

  struct Callbacks
  {
    std::function<void(bool)> head_tracking_cb;
    std::function<void(bool)> ball_follow_cb;
    std::function<void(int)> action_page_cb;
    std::function<void(const GameStateMsg &)> state_update_cb;
  };

  GameStateListener(rclcpp::Node *node,
                    Config config,
                    Callbacks callbacks);

  bool hasState() const { return have_state_; }
  const GameStateMsg &latestState() const { return latest_state_; }

private:
  void onGameState(const GameStateMsg::SharedPtr msg);
  void handleStateTransition(uint8_t new_state);
  void requestActionPage(int page) const;
  void setHeadTracking(bool enable) const;
  void setBallFollowing(bool enable) const;

  rclcpp::Node *node_{nullptr};
  Config config_;
  Callbacks callbacks_;
  rclcpp::Subscription<GameStateMsg>::SharedPtr sub_;

  GameStateMsg latest_state_{};
  bool have_state_{false};
  uint8_t last_state_{GameStateMsg::GAMESTATE_INITIAL};
};

}  // namespace stride::op3
