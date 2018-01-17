#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H

#include "ros/ros.h"

namespace capra {
    
    //The NodeWrapper needs info about the type of message
    //we subscribe to, and the type of handle to that 
    //message to implement callback functionality
    template <typename MsgType>
    class NodeWrapper {
        public:
            // No implicit conversion should be necessary for 
            // this class' purpose 
            explicit NodeWrapper();
            explicit NodeWrapper(const uint32_t);

            void repost(const typename MsgType::ConstPtr& msg);

            void subscribe_republish(ros::NodeHandle& n, const std::string&, 
                    const std::string&);
            void subscribe_republish(ros::NodeHandle& n, const std::string&,
                    const std::string&, uint32_t);
            void subscribe_republish(ros::NodeHandle&n, const std::string&, 
                    const std::string&, uint32_t, uint32_t);

        private:
            ros::Publisher _pub;
            ros::Subscriber _sub;
            uint32_t _default_queuesize = 1000;
    };

    /*
     *Constructors
     */
    template <typename MsgType>
    NodeWrapper<MsgType>::NodeWrapper()
    {}

    template <typename MsgType>
    NodeWrapper<MsgType>::NodeWrapper(const uint32_t queuesize) : 
            _default_queuesize(queuesize)
    {}

    /*
     *Callback republishes messages
     */
    template <typename MsgType>
    void NodeWrapper<MsgType>::repost(const typename MsgType::ConstPtr& msg)
    {
        this->_pub.publish(*msg);
    }

    /*
     * Subscribes to a topic and hooks up the callback to its messages.
     * Overloads are used to specify different queue sizes before
     * either the publisher or the subscriber flushes messages.
     * An overload taking a single queue size as argument 
     * only specifies the publisher's queue size. If trying to
     * specify the subscriber's queue size, use the overload
     * taking two queue sizes as parameters, or a combination
     * of the default queue size constructor and the single
     * queue size parameter overload.
     */
    template <typename MsgType>
    void NodeWrapper<MsgType>::subscribe_republish(ros::NodeHandle& n, 
            const std::string& lowlevel_topic, const std::string& new_topic)
    {
        subscribe_republish(n, lowlevel_topic, new_topic, 
                _default_queuesize, _default_queuesize);
    }

    template <typename MsgType>
    void NodeWrapper<MsgType>::subscribe_republish(ros::NodeHandle& n, 
            const std::string& lowlevel_topic, const std::string& new_topic,
            const uint32_t pub_queuesize)
    {
        subscribe_republish(n, lowlevel_topic, new_topic, 
                pub_queuesize, _default_queuesize);
    }

    template <typename MsgType>
    void NodeWrapper<MsgType>::subscribe_republish(ros::NodeHandle& n, 
            const std::string& lowlevel_topic, const std::string& new_topic,
            const uint32_t pub_queuesize, const uint32_t sub_queuesize)
    {
        this->_pub = n.advertise<MsgType>(new_topic, pub_queuesize);
        this->_sub = n.subscribe(lowlevel_topic, sub_queuesize, 
                &NodeWrapper<MsgType>::repost, this);
    }

}

#endif // NODE_WRAPPER_H