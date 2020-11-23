# ShadowHand Library [![Build Status](https://travis-ci.com/kanishkaganguly/shadowlibs.svg?token=Q6CpHKtysiKmxPk3Lgqs&branch=master)](https://travis-ci.com/kanishkaganguly/shadowlibs)
### Utility library for Shadow Dexterous Hand

- [Documentation](https://kanishkaganguly.github.io/shadowlibs/doxygen_generated/html/files.html) is available here.
- Abstraction library for using the ShadowHand and BioTac sensors
- Divided into 3 main parts:
    - `shadow_finger`: Contains abstractions for each finger, and corresponding BioTac sensor
    - `shadow_planning`: Contains abstractions for planning and execution of each `Finger`
    - `shadow_utils`: Contains utilities for ROS, RViz, TF, MoveIt and some general conversion utilities
    - `shadow_imports`: Bunch of imports for everything
 - Most of the code is designed to directly use a `Finger` object, although they are overloaded to use lower level MoveIt inputs as well.
 - Examples for using this library is given in the [ShadowHand](https://github.com/kanishkaganguly/shadowhand) repository.
- Make sure to compile this first, before ShadowHand package, since this will generate a `libshadowlibs.so` file that can be used by other ROS packages. Check your `devel` folder to see if this file has been created after compilation.

### License
This project is released under the [BSD-3 license](https://github.com/kanishkaganguly/shadowlibs/blob/master/LICENSE). If you use `shadowlibs` in your research, please cite our pre-print publication:
```bibtex
@misc{ganguly2020grasping,
      title={Grasping in the Dark: Compliant Grasping using Shadow Dexterous Hand and BioTac Tactile Sensor}, 
      author={Kanishka Ganguly and Behzad Sadrfaridpour and Krishna Bhavithavya Kidambi and Cornelia Fermüller and Yiannis Aloimonos},
      year={2020},
      eprint={2011.00712},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
