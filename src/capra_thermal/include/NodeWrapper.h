#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H

#include "ros/ros.h"

namespace capra {
    
    template <typename MsgType, typename MsgHandle>
    class NodeWrapper {
        public:
            void repost(const MsgHandle& msg);

            void subscribe_republish(ros::NodeHandle& n, const std::string&, const std::string&);

        private:
            ros::Publisher _pub;
            ros::Subscriber _sub;
    };

    template <typename MsgType, typename MsgHandle>
    void NodeWrapper<MsgType, MsgHandle>::repost(const MsgHandle& msg)
    {
        this->_pub.publish(*msg);
    }

    template <typename MsgType, typename MsgHandle>
    void NodeWrapper<MsgType, MsgHandle>::subscribe_republish(ros::NodeHandle& n, 
            const std::string& driver_topic, const std::string& new_topic)
    {
        _sub = n.subscribe(driver_topic, 100, &NodeWrapper<MsgType, MsgHandle>::repost, this);
        _pub = n.advertise<MsgType>(new_topic, 50);
    }

}

#endif // NODE_WRAPPER_H