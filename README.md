# Covariance Matrix Adaptation Evolution Strategy and Locomotion

Using the covariance matrix adaptation evolution strategy (CMA-ES) to generate animations for creatures in a land-based physical simulation. 

I built a variety of creatures using the Dynamimc Animation and Robotics Toolkit (DART), including a 3 segment worm, a spider, a quadruped, and a quadruped with wide feet. I also created controllers to move these creatures. I modeled the movement of each joint along the necessary axes using periodic functions. These functions were sinusoids with an amplitude, period, and phase shift. For the quadruped, I created a number of different controllers to experiment with different types of symmetrical movement. For example, one controller had the left legs and right legs move opposite to each other. Another one had the front legs move in sync while the back legs did the same. An advantage to this was that it reduced the number of parameters that needed to be optimized. For the quadruped with feet, I also experimented with the shape of the feet. I tried a rounded capsule shape and a wide flat shape. Ultimately, the quadruped with wide, flat feet ended up with the smoothest animation after optimizing using CMA-ES. It can be seen below.

https://user-images.githubusercontent.com/18231852/145095666-46bff181-b8af-4b1a-b56d-29b394bc6c86.mp4

As I progressed from the worm to the quadruped with feet, the degrees of freedom increased and so too did the amount of time CMA-ES took. If I wanted to run it with a large population size and many generations, it would take an inordinate amount of time. So I made one optimization and paralleliized the CMA-ES procedure using Python's multiprocessing library.

It does seem that an evolutionary strategy can be used to produce somewhat realistic animations, such as the one above.


