# ShadowHand Library [![Build Status](https://travis-ci.com/kanishkaganguly/shadowlibs.svg?token=Q6CpHKtysiKmxPk3Lgqs&branch=master)](https://travis-ci.com/kanishkaganguly/shadowlibs)
### Utility library for Shadow Dexterous Hand

- Abstraction library for using the ShadowHand and BioTac sensors
- Divided into 3 main parts:
    - `shadow_finger`: Contains abstractions for each finger, and corresponding BioTac sensor
    - `shadow_planning`: Contains abstractions for planning and execution of each `Finger`
    - `shadow_utils`: Contains utilities for ROS, RViz, TF, MoveIt and some general conversion utilities
    - `shadow_imports`: Bunch of imports for everything
 - Most of the code is designed to directly use a `Finger` object, although they are overloaded to use lower level MoveIt inputs as well.
 - Examples for using this library is given in the [ShadowHand](https://github.com/kanishkaganguly/shadowhand) repository.
