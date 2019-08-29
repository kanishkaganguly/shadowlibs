//
// Created by behzad on 6/5/19.
//

#include <shadowlibs/shadow_finger.hpp>

/** @namespace shadow_hand shadow_hand.hpp "include/shadowlibs/shadow_hand.hpp"
 *  @brief Utility functions for managing groups of fingers of Shadow Hand.
 * 	Constructs "Hand" class of shared pointers to multiple fingers, once multiple finger
 * 	instances have been created.
 */
namespace shadow_hand {
/** @class Hand shadow_hand.hpp "include/shadowlibs/shadow_hand.hpp"
 *  @brief Utility functions for managing groups of fingers of Shadow Hand.
 * 	Constructs "Hand" class of shared pointers to multiple fingers, once multiple finger
 * 	instances have been created.
 */
class Hand {
public:
  /**
   * @brief Vector of the shared pointers to the fingers that we want to control
   * Shared pointers because you cannot make a Hand without having created Fingers
   */
  std::vector <std::shared_ptr<shadow_finger::Finger>> _fingerVec;

  /**
   * @brief Constructor for Hand class
   * @param fingers_list List of shared pointers to Fingers
   */
  Hand(std::initializer_list <std::shared_ptr<shadow_finger::Finger>> &fingers_list) {
    // Minimum 2 fingers are required in Hand
    assert(fingers_list.size() >= 2);
    // Populate Hand with Fingers
    for (auto finger:fingers_list) {
      _fingerVec.push_back(finger);
    }
    ROS_INFO_STREAM("Initialized Hand with: " << _fingerVec.size() << " fingers.");
  }

  /**
   * @brief Add Finger to Hand
   * @param finger Shared pointer to shadow_finger::Finger
   */
  void addFinger(const std::shared_ptr <shadow_finger::Finger> &finger) { _fingerVec.push_back(finger); }

  /**
   * @brief Get number of Fingers in hand
   * @return Integer size of _fingerVec
   */
  int numFingers() { return _fingerVec.size(); }

  /**
   * @brief Return all instances of Fingers in Hand as vector
   * @return Vector of shared pointers to fingers
   */
  std::vector <std::shared_ptr<shadow_finger::Finger>> getFingers() { return _fingerVec; }
};
}
