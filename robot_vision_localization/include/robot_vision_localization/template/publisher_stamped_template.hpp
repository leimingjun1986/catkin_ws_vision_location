/*
 * @Description: publisher template 
 * @Author: PIFAN
 * @Date: 2021-1-15
 */
#ifndef AGV_NAVIGATION_PUBLISHER_STAMPED_TEMPLATE_HPP_
#define AGV_NAVIGATION_PUBLISHER_STAMPED_TEMPLATE_HPP_

#include <string>
#include <ros/ros.h>

namespace robot_vision_localization 
{
template <typename T>
class PublisherStamped
{
public:
    PublisherStamped(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);              
    PublisherStamped() = default;
    virtual ~PublisherStamped() = default;
    void publish(const T &msg) const;
    // void publish(const T &msg, ros::Time time);
    bool hasSubscribers();
protected:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    // T data_;
};

template <typename T>
PublisherStamped<T>::PublisherStamped(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size):nh_(nh)
{
    publisher_ = nh_.advertise<T>(topic_name, buff_size, true);
}

template <typename T>
bool PublisherStamped<T>::hasSubscribers()
{
    return publisher_.getNumSubscribers() != 0;
}

template <typename T>
void PublisherStamped<T>::publish(const T &msg) const
{
    publisher_.publish(msg);
}


}
#endif