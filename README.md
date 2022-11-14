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


- Alternative IK Solver
  - 
