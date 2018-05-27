#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H

#include "ros/ros.h"
#include <forward_list>

namespace capra {
    
    // The NodeWrapper needs info about the type of message
    // we subscribe to
    template <typename TMessage>
    class NodeWrapper {
        public:
            // No implicit conversion should be necessary for 
            // this class' purpose 
            explicit NodeWrapper();
            explicit NodeWrapper(const uint32_t);

            // Methods to affect state
            virtual void add_observer(
                    std::function<void (const typename TMessage::ConstPtr& msg)>);
            void clear() { _callbacks.clear(); }
            void wake_observers() {this->mode == Mode::Custom; }
            void silence() { this->_mode == Mode::Silent; }
            void wake_default() { this->_mode == Mode::Default; }

            // These overloads subscribe with default callback
            void subscribe_republish(ros::NodeHandle& n, const std::string&, 
                    const std::string&);
            void subscribe_republish(ros::NodeHandle& n, const std::string&,
                    const std::string&, uint32_t);
            void subscribe_republish(ros::NodeHandle&n, const std::string&, 
                    const std::string&, uint32_t, uint32_t);

            // These overloads replace default callback with custom callback
            void subscribe_republish(ros::NodeHandle&, 
                    const std::string&, const std::string&,
                    std::function<void (const typename TMessage::ConstPtr& msg)>);
            void subscribe_republish(ros::NodeHandle&, 
                    const std::string&, const std::string&,
                    const uint32_t,
                    std::function<void (const typename TMessage::ConstPtr& msg)>);
            void subscribe_republish(ros::NodeHandle&, 
                    const std::string&, const std::string&,
                    const uint32_t, const uint32_t,
                    std::function<void (const typename TMessage::ConstPtr& msg)>);

        protected:
            // Default callback
            virtual void forward(const typename TMessage::ConstPtr& msg);

        private:
            // Observers
            typedef std::function<void (const typename TMessage::ConstPtr& msg)> 
                    TCallback;
            std::forward_list<TCallback> _callbacks;

            ros::Publisher _pub;
            ros::Subscriber _sub;
            uint32_t _default_queuesize = 1000;

            enum Mode 
            {
                Default,
                Custom,
                Silent
            } _mode;
    };

    /*
     *Constructors
     */
    template <typename TMessage>
    NodeWrapper<TMessage>::NodeWrapper()
    {}

    template <typename TMessage>
    NodeWrapper<TMessage>::NodeWrapper(const uint32_t queuesize) : 
            _default_queuesize(queuesize)
    {}

    /*
     *Callback republishes messages
     */
    template <typename TMessage>
    void NodeWrapper<TMessage>::forward(const typename TMessage::ConstPtr& msg)
    {
        // Uncomment line below to test connection
        // ROS_INFO("Republished message");
        switch (this->_mode){

            case Mode::Default:
                this->_pub.publish(*msg);
                break;

            case Mode::Custom:
                    for (auto& c : _callbacks)
                        c(msg);
                break;

            case Mode::Silent:
                break;
        }
    }

    /*
     * Replaces function currently held by this->_callback
     */
    template <typename TMessage>
    void NodeWrapper<TMessage>::add_observer(
            std::function<void (const typename TMessage::ConstPtr& msg)> callback)
    {
        this->_callbacks.emplace_front(callback);
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
    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic, 
            const std::string& new_topic)
    {
        this->subscribe_republish(n, lowlevel_topic, new_topic, 
                _default_queuesize, _default_queuesize);
    }

    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic, 
            const std::string& new_topic,
            const uint32_t pub_queuesize)
    {
        this->subscribe_republish(n, lowlevel_topic, new_topic, 
                pub_queuesize, _default_queuesize);
    }

    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic,
            const std::string& new_topic,
            const uint32_t pub_queuesize,
            const uint32_t sub_queuesize)
    {
        this->_pub = n.advertise<TMessage>(new_topic, pub_queuesize);
        this->_sub = n.subscribe(lowlevel_topic, sub_queuesize, 
                &NodeWrapper<TMessage>::forward, this);
    }

    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic, 
            const std::string& new_topic,
            std::function<void (const typename TMessage::ConstPtr& msg)> callback)
    {
        this->subscribe_republish(n, lowlevel_topic, new_topic, 
                _default_queuesize, _default_queuesize, callback);
    }

    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic, 
            const std::string& new_topic,
            const uint32_t pub_queuesize, 
            std::function<void (const typename TMessage::ConstPtr& msg)> callback)
    {
        this->subscribe_republish(n, lowlevel_topic, new_topic, 
                pub_queuesize, _default_queuesize, callback);
    }

    template <typename TMessage>
    void NodeWrapper<TMessage>::subscribe_republish(
            ros::NodeHandle& n, 
            const std::string& lowlevel_topic, 
            const std::string& new_topic,
            const uint32_t pub_queuesize, 
            const uint32_t sub_queuesize,
            std::function<void (const typename TMessage::ConstPtr& msg)> callback)
    {
        this->_callbacks.emplace_front(callback);
        this->_mode = Mode::Custom;
        this->subscribe_republish(n, lowlevel_topic, new_topic, 
                pub_queuesize, sub_queuesize);
    }

}

#endif // NODE_WRAPPER_H