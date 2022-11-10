import java.util.ArrayList;

public class Chain {
  private ArrayList<Float> jointLimits;
  private float totallengths = 0;
  private float tolerance = 0.1;
  private Vector2 root;
  
  // startPos[0] == root,..., startPos[startPos.size()-1] = endPos,
  public ArrayList<Vector2> startPos;
  public ArrayList<Float> lengths;
  public ArrayList<Float> rotates;
  public int numLinks;
  
  Chain(ArrayList<Float> lengths, ArrayList<Float> rotates, Vector2 root) {
    this.jointLimits = new ArrayList<Float>();
    this.startPos = new ArrayList<Vector2>();
    this.startPos.add(root);
    this.root = root;
    for(float length : lengths){
      this.startPos.add(new Vector2(0,0));
      this.jointLimits.add(Float.POSITIVE_INFINITY);
      totallengths += length;
    }
    this.lengths = lengths;
    this.rotates = rotates;
    numLinks = lengths.size();
    fk();
  }
  
  private void fk() {
    if (numLinks > 0) {
      float theta = 0;
      for (int i = 0; i < numLinks; i++) {  
        theta += rotates.get(i);
        startPos.set(i + 1, new Vector2(cos(theta) * lengths.get(i), sin(theta) * lengths.get(i)).plus(startPos.get(i)));
      }
    }
  }
  
  public void ccd(Vector2 goal) {
    for (int i = numLinks - 1; i >= 0; i--) {      
      Vector2 startToGoal, startToEndEffector;
      float dotProd, angleDiff;
      
      startToGoal = goal.minus(startPos.get(i));
      startToEndEffector = startPos.get(startPos.size() - 1).minus(startPos.get(i));
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd, -1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        rotates.set(i, rotates.get(i) + angleDiff);
      else
        rotates.set(i, rotates.get(i) - angleDiff);
      // Joint limit set here
      if(rotates.get(i)>jointLimits.get(i)){
        rotates.set(i, jointLimits.get(i));
      } else if (rotates.get(i)<-jointLimits.get(i)){
        rotates.set(i, -jointLimits.get(i));
      }
      fk();
    }
  }

  private void calculateRotate(){
    // Vector2 startToGoal, startToEndEffector;
    // float dotProd, angleDiff;
    
    // // startToGoal = startPos.get(endPosIndex).minus(startPos.get(startPosIndex));
    // // Vector2 unit = new Vector2(1,0);
    // // // startToEndEffector = startPos.get(startPos.size() - 1).minus(startPos.get(startPosIndex));
    // // dotProd = dot(startToGoal.normalized(),unit);
    // // // dotProd = clamp(dotProd, -1,1);
    // //   angleDiff = Main.Vector2.directionTo(startToGoal);
    // // if (cross(startToGoal,startToEndEffector) < 0)
    // //   rotates.set(startPosIndex, angleDiff);
    // // else


    // Vector2 targetDirection = startPos.get(startPosIndex).directionTo(startPos.get(endPosIndex));
    // Vector2 unit = new Vector2(1,0);
    // float targetAngle = dot(unit, targetDirection);
    // targetAngle = acos(targetAngle);
    // if (cross(targetDirection,unit) < 0)
    //   rotates.set(startPosIndex, 45f);
    // else
    //   rotates.set(startPosIndex, 45f);


    //   // rotates.set(startPosIndex, 0+targetAngle);
    
    //Each rotation in rotates is the sum of all rotations before.
    //We will try to fix this by combating this issue.
    //To do this, this function is modified so that calling this will calculate the rotation from start to end effector from scratch.
    //Knowing how rotates works, we will have two variables: accumRotates, which is the accumulated rotation from the previous rotates, and worldRotation, which is the "world space" rotation.
    //So we will first find the world rotation - which is just the angle of the vector pointing from one link to the next...
    //Then we will calculate the accumRotates, which will get all the rotations prior to this.
    //Then, we get the difference between the accumRotates and worldRotation, and set this as the new angle.

    //As a note:
    // StartPos holds the position of each joint (from root to end effector)
    // Rotates holds the rotation of just each rectangle.
    // StartPos.size == (rotates.size + 1)
    for (int i = 0; i < startPos.size() - 1; i++){
      float accumRotates = 0; //holds sum of rotations accumulated from previous rotates
      for (int j = 0; j < i; j++){
        accumRotates += rotates.get(j);
      }
      Vector2 accumDirection = new Vector2(cos(accumRotates), sin(accumRotates)); //this is teh direction vector of accumRotates
      Vector2 worldDirection = (startPos.get(i+1).minus(startPos.get(i))).normalized(); //get the world target direction.
      Vector2 unitDirection = new Vector2(1,0); //the direction of a 0 angle in the world.
      float worldRotation = acos(dot(worldDirection, unitDirection)); //this is the rotation relative to the world.

      float angleDiff = worldRotation - accumRotates; //Now we get the angle relative to the previous rotates...
      // rotates.set(i, angleDiff);
      if (cross(accumDirection, worldDirection) < 0)
        rotates.set(i, angleDiff);
      else
        rotates.set(i, -angleDiff);
    }
  }

  private void fabrikBackward(Vector2 goal){
    startPos.set(startPos.size() - 1, goal);
    for(int i = numLinks - 1; i >= 0; i--){
      Vector2 newPos = startPos.get(i + 1).directionTo(startPos.get(i));
      newPos.setToLength(lengths.get(i));
      newPos.add(startPos.get(i + 1));
      startPos.set(i, newPos);
      // calculateRotate(i, i + 1, goal);
    }
  }

  private void fabrikForward(Vector2 goal){
    startPos.set(0, root);
    for(int i = 1; i <= numLinks; i++){
      Vector2 newPos = startPos.get(i - 1).directionTo(startPos.get(i));
      newPos.setToLength(lengths.get(i - 1));
      newPos.add(startPos.get(i - 1));
      startPos.set(i, newPos);
      // calculateRotate(i-1, i, goal);
    }    
  }

  public void fabrik(Vector2 goal) {
    // distance = target - root
    float distance = startPos.get(0).distanceTo(goal);
    if(distance > totallengths){
      // target out of reach
      for (int i = 0; i < numLinks; i++) {     
        Vector2 startToGoal = goal.minus(startPos.get(i));
        Vector2 startToEndEffector = startPos.get(startPos.size() - 1).minus(startPos.get(i));
        float dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
        dotProd = clamp(dotProd, -1,1);
        float angleDiff = acos(dotProd);
        if (cross(startToGoal,startToEndEffector) < 0)
          rotates.set(i, rotates.get(i) + angleDiff);
        else
          rotates.set(i, rotates.get(i) - angleDiff);
        fk();
      }
    } else {
      float dif = startPos.get(startPos.size() - 1).distanceTo(goal);
      int count = 0;
      while(dif > tolerance){
        fabrikBackward(goal);
        fabrikForward(goal);
        dif = startPos.get(startPos.size() - 1).distanceTo(goal);
        count++;
        if(count > 10){
          break;
        }
      }
      calculateRotate();
    }
    
    
    // for (int i = numLinks - 1; i >= 0; i--) {      
      // Vector2 startToGoal, startToEndEffector;
      // float dotProd, angleDiff;
      
      // startToGoal = goal.minus(startPos.get(i));
      // startToEndEffector = startPos.get(startPos.size() - 1).minus(startPos.get(i));
      // dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      // dotProd = clamp(dotProd, -1,1);
      // angleDiff = acos(dotProd);
      // if (cross(startToGoal,startToEndEffector) < 0)
      //   rotates.set(i, rotates.get(i) + angleDiff);
      // else
      //   rotates.set(i, rotates.get(i) - angleDiff);
    //   // Joint limit set here
    //   if(rotates.get(i)>jointLimits.get(i)){
    //     rotates.set(i, jointLimits.get(i));
    //   } else if (rotates.get(i)<-jointLimits.get(i)){
    //     rotates.set(i, -jointLimits.get(i));
    //   }
    //   ccdFk();
    // }
  }

  // Link start from 1
  public float getRotateTo(int link){
    float r = 0;
    for(int i = 0; i < link; i++){
      r += rotates.get(i);
    }
    return r;
  }
  
  // Link start from 1
  public void setJointLimit(int link, float limit){
    jointLimits.set(link - 1, limit);
  } 
}
