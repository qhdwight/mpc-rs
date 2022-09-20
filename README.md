# Linear Model Predictive Control

### Demo

https://user-images.githubusercontent.com/20666629/191329146-cf487d34-716d-4843-b2ec-523a79885047.mp4

### Idea

We can model our system and predict its state into the future.
We can also model how our inputs affect this future state.
Using a constraint solver such as QP, we can find a series of control inputs that minimizes error over a "horizon" or short time in the future.

See: https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1056/Lecture14_MPC.pdf

### Libraries

[nalgebra](https://nalgebra.org/) for matrix operations

[splines](https://docs.rs/splines/latest/splines/) for linear interpolation

[osqp](https://docs.rs/osqp/latest/osqp/) for a sparse QP solver

[Bevy](https://bevyengine.org/) for visualization + [lyon](https://github.com/Nilirad/bevy_prototype_lyon) for path rendering

### Code

[controller.rs](./src/controller.rs) Linear MPC implementation, QP solver over horizon

[robot.rs](./src/robot.rs) Kinematic modeling of unicycle robot

[math.rs](./src/math.rs) Misc. features for [nalgebra](https://nalgebra.org/) since it is relatively new
