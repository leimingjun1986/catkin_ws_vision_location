/*
 * @Description: ros SubscriberDeque template
 * @Author: Pi Fan
 * @Date: 2021-1-15
 */
#ifndef AGV_NAVIGATION_TEMPLATE_SUBSCRIBER_DEQUE_HPP_
#define AGV_NAVIGATION_TEMPLATE_SUBSCRIBER_DEQUE_HPP_

#include <deque>
#include <ros/ros.h>
#include <mutex>

namespace robot_vision_localization 
{
template <typename T>   //
class SubscriberDeque 
{
public:
    SubscriberDeque(ros::NodeHandle& nh, std::string topicName, size_t buffSize);
    SubscriberDeque() = default;
    void parseData(std::deque<T> &data);
private:
    void msgCallback(const T &msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<T> newArrivedDeque_;
private:
    std::mutex mutex_;
};

template <typename T>
SubscriberDeque<T>::SubscriberDeque(ros::NodeHandle& nh, std::string topicName, size_t buffSize):nh_(nh) 
{
    subscriber_ = nh_.subscribe(topicName, buffSize, &SubscriberDeque<T>::msgCallback, this);
}

template <typename T>
void SubscriberDeque<T>::msgCallback(const T &msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    newArrivedDeque_.push_back(msg);
}

template <typename T>
void SubscriberDeque<T>::parseData(std::deque<T> &data) 
{   
    std::unique_lock<std::mutex> lock(mutex_);
    if (newArrivedDeque_.size())
    {
        data.insert(data.end(), newArrivedDeque_.begin(), newArrivedDeque_.end());
        newArrivedDeque_.clear();
    }
}
}
#endif