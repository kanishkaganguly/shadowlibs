//
// Created by behzad on 6/5/19.
//

#include <shadowlibs/shadow_utils.hpp>
#include <shadowlibs/shadow_planning_options.hpp>
#include <shadowlibs/shadow_finger.hpp>

namespace shadow_hand {
    class Hand {
        // NodeHandle
        ros::NodeHandle _node_handle;
        // Vector of the pointers to the fingers that we want to control
        std::vector <shadow_finger::Finger> _fingerVec;

        // Minimum requirement of a hand is two fingers
    public:
        Hand(shadow_finger::Finger &finger1, shadow_finger::Finger &finger2, ros::NodeHandle &node_handle) :
                _node_handle(node_handle) {
            _fingerVec.push_back(finger1);
            _fingerVec.push_back(finger2);
            ROS_INFO_STREAM("Initialized Hand with: " << finger1._finger_name << " and " << finger2._finger_name);
        }

        void addFinger(shadow_finger::Finger &finger) { _fingerVec.push_back(finger); }

        int numFingers() { return _fingerVec.size();}

        std::vector <shadow_finger::Finger> getFingers() { return _fingerVec;}
    };
}
