#include "op3_control_bridge/gamestate_listener.hpp"

#include <stdexcept>
#include <utility>

namespace stride::op3
{

namespace
{
const char *stateToString(uint8_t state)
{
  using GS = GameStateListener::GameStateMsg;
  switch (state)
  {
    case GS::GAMESTATE_INITIAL:
      return "INITIAL";
    case GS::GAMESTATE_READY:
      return "READY";
    case GS::GAMESTATE_SET:
      return "SET";
    case GS::GAMESTATE_PLAYING:
      return "PLAYING";
    case GS::GAMESTATE_FINISHED:
      return "FINISHED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

GameStateListener::GameStateListener(rclcpp::Node *node,
                                     Config config,
                                     Callbacks callbacks)
: node_(node),
  config_(std::move(config)),
  callbacks_(std::move(callbacks))
{
  if (!node_)
  {
    throw std::invalid_argument("GameStateListener requires a valid node");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
             .reliability(rclcpp::ReliabilityPolicy::Reliable)
             .durability(rclcpp::DurabilityPolicy::Volatile)
             .history(rclcpp::HistoryPolicy::KeepLast);
  sub_ = node_->create_subscription<GameStateMsg>(
    config_.topic, qos,
    std::bind(&GameStateListener::onGameState, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "GameStateListener subscribed to %s",
              config_.topic.c_str());
}

void GameStateListener::onGameState(const GameStateMsg::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  latest_state_ = *msg;
  const bool state_changed = !have_state_ || (msg->game_state != last_state_);
  have_state_ = true;
  last_state_ = msg->game_state;

  if (callbacks_.state_update_cb)
  {
    callbacks_.state_update_cb(latest_state_);
  }

  if (!state_changed)
  {
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Game state changed to %s (%u)",
              stateToString(msg->game_state),
              msg->game_state);
  if (config_.use_default_transitions)
  {
    handleStateTransition(msg->game_state);
  }
}

void GameStateListener::handleStateTransition(uint8_t state)
{
  using GS = GameStateMsg;
  switch (state)
  {
    case GS::GAMESTATE_INITIAL:
      setBallFollowing(false);
      setHeadTracking(false);
      requestActionPage(config_.init_pose_page);
      break;
    case GS::GAMESTATE_READY:
      setBallFollowing(false);
      setHeadTracking(false);
      requestActionPage(config_.walk_ready_page);
      break;
    case GS::GAMESTATE_SET:
      setBallFollowing(false);
      setHeadTracking(true);
      break;
    case GS::GAMESTATE_PLAYING:
      setHeadTracking(true);
      setBallFollowing(true);
      break;
    case GS::GAMESTATE_FINISHED:
    default:
      setBallFollowing(false);
      setHeadTracking(false);
      requestActionPage(config_.init_pose_page);
      break;
  }
}

void GameStateListener::requestActionPage(int page) const
{
  if (!callbacks_.action_page_cb)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "No action page callback set; ignoring request for page %d",
                 page);
    return;
  }
  callbacks_.action_page_cb(page);
}

void GameStateListener::setHeadTracking(bool enable) const
{
  if (!callbacks_.head_tracking_cb)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "No head tracking callback set; ignoring request (%d)",
                 enable);
    return;
  }
  callbacks_.head_tracking_cb(enable);
}

void GameStateListener::setBallFollowing(bool enable) const
{
  if (!callbacks_.ball_follow_cb)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "No ball follow callback set; ignoring request (%d)",
                 enable);
    return;
  }
  callbacks_.ball_follow_cb(enable);
}

}  // namespace stride::op3
