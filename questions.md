## Q-Matrix
- Can't I just add the acceleration to the state vector? I know I can't measure it with a sensor, but I can calculate it in each prediction step ((v2 - v1) / dt). Where does the filter break if I'd do it this way?

## Project Setup
* why is Eigen included in the project? one can install it easily on any system
* why did you choose this websocket library that is so cumbersome to install? there should be easier solutions for inter-program communication
* `sudo make install` on my machine? that's not necessary, I can just link it locally
  * you could just utilize a git submodule for 3rd party libs
  * the library will sit there for ever and compromise my system
  * you're suggesting to disable a crucial security feature of osx just for the sake of using this library! please find another solution, this is just bad for so many reasons - especially for unexperienced users who don't know what they are doing with those lines

## Simulator
* a step-button would be useful
* uses two cores while idling and doing nothing .. draining the battery in half an hour
* a line-plot would be better as visualization (at least for the filtered position)
* an ellipsoid plot for the covariance would be nice
* the HUD is not working correctly for some resolutions
* what do the arrows mean in the blue sensor input plot? bearing? is this correctly visualized? the direction seems off