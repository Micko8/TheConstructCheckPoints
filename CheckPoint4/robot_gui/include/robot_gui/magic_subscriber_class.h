#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

#include <mutex>
#include <ros/ros.h>
#include <string>


template <typename ROSMessageType> class MagicSubscriber {
public:
    MagicSubscriber(ros::NodeHandle &ros_node,
                const std::string &subscriber_topic) {
    m_subscriber_topic = subscriber_topic;
    m_ros_node_object = &ros_node;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }

    subscriber_ = m_ros_node_object->subscribe(
        m_subscriber_topic, 1,
        &MagicSubscriber<ROSMessageType>::CallbackToTopic, this);
  };

  void CallbackToTopic(const typename ROSMessageType::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_msg_ = *msg;
    has_message_ = true;
  }

  bool getLatest(ROSMessageType &output) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_message_) {
    return false;
  }

  output = latest_msg_;
  return true;
}

private:
  ros::NodeHandle *m_ros_node_object;
  std::string m_subscriber_topic;
  ros::Subscriber subscriber_;
  ROSMessageType latest_msg_;
  bool has_message_ = false;
  mutable std::mutex mutex_; // mutable because getLatest is const method...
};

#endif