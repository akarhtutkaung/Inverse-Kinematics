# Inverse Kinematics
@Author:
- Akar (Ace) Kaung
- Muhammad Arfan Maulana

Language: Processing

## Summary
This program implements the inverse kinematics to move around the joints of the character. We use 2 different type of inverse kinematics algorithm, CCD, and FABRIK.

- Single-arm IK

https://user-images.githubusercontent.com/76828992/201558767-ecd4b0a4-9f6d-4f7d-9c0f-bc3a3c80531c.mp4
 
- Multi-arm IK

https://user-images.githubusercontent.com/76828992/201558936-01c0bacd-e71b-4ee8-ac84-79ce14822242.mp4

- Joint limits
  - From 0-9sec of the video, we can see that the wrist can only more up to -90 degree and +90 degree
  - From 9-18sec of the video, we can see that the wrist can move fully up to any angles. In real life, the wrist can only bend forward and backward up to 90 degree and anything more than that, it will break the wrist.
  - This joint limits has been applied to the other joints such as the legs, as to keep the video time down, we only show the arm's joint.

https://user-images.githubusercontent.com/76828992/201559479-a170f50b-3911-4ecb-a01b-8c51dca98f0f.mp4

- User can select which arms/legs they would like to move (User Interaction)
- User can move the character itself using arrow keys (Moving IK)


https://user-images.githubusercontent.com/76828992/201560417-2229b9a8-77d9-473f-9afb-72f2751e9f10.mp4

- Re-rooting IK
  - User can toggle craw action by pressing *space* bar and move the character around to make it look like crawling


https://user-images.githubusercontent.com/76828992/201560583-a4931df2-b862-4538-ae85-cf1fd828a3c7.mp4


### Alternative IK Solver
- For this project, we implemented two different implementations of IK: **CCD** and **FABRIK**.
- In the videos above, the left top, left bottom, and right bottom limbs used CCD, while the top right limb used FABRIK.
- Summary of the solvers:
  - **CCD (Cyclic Coordinate Descent):** The simplest IK method that primarily computes the angles of each joint to reach a goal.
    - The algorithm starts by calculating the angle of the lastmost joint such that the end effector (the tip of the limb) becomes closest to the goal.
    - Then, the algorithm moves up the limb to the prior joint, and calculates the angle such that the end effector moves even closer to the goal.
    - This process is done all the way up the arm until the root joint, then repeated over and over until the limb reaches its optimal position.
    - **PROS**:
      - It is the simplest method out there and is applied in many fields.
      - It is easy to implement joint limits (i.e. the maximum angle of a given joint at any direction)
    - **CONS**:
      - It is simple to implement only in 2D. In 3D, where rotations are represented with Quaternions and/or matrices than Euler angles, the math involved becomes more complex.
      - In our implementation, the arm does not 'instantly' move to its optimized position.
  - **FABRIK (Forward And Backward Reaching Inverse Kinematics):** An alternative IK method that primarily computes the positions of each joint to reach a goal.
    - The algorithm begins by moving the end effector directly to the goal. After this, it will pull the next joint up to the arm closer to the goal such that the distance between this joint and the end effector is preserved.
    - Then, the algorithm moves up the limb to the next joint, and moves it closer to the joint that was moved on the last step to preserve the length between them,
    - This process is done all the way up the arm until even the root itself is repositioned. Then, the same process is done in _reverse_, so that the root is returned to its original position.
    - This process of "pulling" the limb to the goal and back to the root is repeated over and over until the limb reaches its optimal position
    - **PROS**:
      - Easier to implement in 3D compared to CCD, as the logic behind it works primarily with vector mathematics, ignoring how angles are represented.
      - In our implementation, the arm reachest its optimized position much quicker than with CCD.
    - **CONS**:
      - As the algorithm ignores the angles of each joint, implementing joint angle limits is significantly more complex. We were not able to implement a working joint limit unfortunately.


#### Which is the better IK solver?
- It mostly depends, but CCD seems to be more practical overall.
  - The one con that FABRIK has is unfortunately very significant: difficult joint constraints. It is rare to see a use of IK that does not require the use
  - Although using CCD can be more complex in higher dimensions, the complexity comes from simply using a different rotation paradigm. Solving joint limits for this is significantly easier than trying to implement joint limits in an algorithm that 'ignores' angles in its calculations (FABRIK).
