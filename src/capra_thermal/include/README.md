# Generic wrapper around ros nodes
I had to bypass standard implementation of callback functionality since the        
ros::NodeHandle::subscribe method has limited flexibility in that area 
(if I am not mistaken). In addition, multiple observers can now observe 
the same topic on an as-per-needed basis with the NodeWrapper<T>::add_observer() method.
A std::forward_list was used for resource efficiency and add/removal speed.    
Random access is not of interest for the purpose of a list of callbacks, especially
when publisher/subscriber pattern should not depend on subscribers' order.
Observers can also be silenced with the NodeWrapper<T>::silence() method 
and woke up with NodeWrapper<T>::wake() on an as-per-needed basis.