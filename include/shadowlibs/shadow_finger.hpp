//
// Created by kganguly on 4/30/19.
//

#pragma once

#include <shadowlibs/shadow_utils.hpp>

namespace shadow_finger {
    // Planning group names
    static const std::string first_finger_group = "rh_first_finger";
    static const std::string middle_finger_group = "rh_middle_finger";
    static const std::string ring_finger_group = "rh_ring_finger";
    static const std::string little_finger_group = "rh_little_finger";
    static const std::string thumb_group = "rh_thumb";
    static const std::string hand_group = "right_hand";

    // End-effector names
    static const std::string eef_first_finger = "rh_fftip";
    static const std::string eef_middle_finger = "rh_mftip";
    static const std::string eef_ring_finger = "rh_rftip";
    static const std::string eef_little_finger = "rh_lftip";
    static const std::string eef_thumb = "rh_thtip";

    // List of end-effector names
    std::vector <std::string> eef_names;
}
