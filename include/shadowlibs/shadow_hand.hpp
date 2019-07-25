//
// Created by behzad on 6/5/19.
//

#include <shadowlibs/shadow_finger.hpp>

namespace shadow_hand {
class Hand {
public:
  /** @brief
   * Vector of the shared pointers to the fingers that we want to control
   * Shared pointers because you cannot make a Hand without having created Fingers
   */
  std::vector <std::shared_ptr<shadow_finger::Finger>> _fingerVec;

  Hand(std::initializer_list <std::shared_ptr<shadow_finger::Finger>> &fingers_list) {
    /** Minimum 2 fingers are required in Hand */
    assert(fingers_list.size() >= 2);
    /** Populate Hand with Fingers */
    for (auto finger:fingers_list) {
      _fingerVec.push_back(finger);
    }
    ROS_INFO_STREAM("Initialized Hand with: " << _fingerVec.size() << " fingers.");
  }

  void addFinger(const std::shared_ptr <shadow_finger::Finger> &finger) { _fingerVec.push_back(finger); }

  int numFingers() { return _fingerVec.size(); }

  std::vector <std::shared_ptr<shadow_finger::Finger>> getFingers() { return _fingerVec; }
};
}
