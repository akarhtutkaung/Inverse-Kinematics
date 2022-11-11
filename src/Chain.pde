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
  public boolean useWorld = false; //if using CCD, this is false. If using FABRIK, is true.
  
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
    useWorld = false; //set to false, which will make rectangles rendered via getRotateTo and rotates.
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

  private void calculateWorldRotate(){
    //Calling this function will calculate the angle of all the rectangles in world space.
    //Used for FABRIK and not CCD!
    for (int i = 0; i < startPos.size() - 1; i++){
      Vector2 worldDirection = (startPos.get(i+1).minus(startPos.get(i))).normalized(); //get the world target direction.
      Vector2 unitDirection = new Vector2(1,0); //the direction of a 0 angle in the world.
      float worldRotation = acos(dot(worldDirection, unitDirection)); //this is the rotation relative to the world.
       if (cross(worldDirection, unitDirection) < 0)
         rotates.set(i, worldRotation);
       else
         rotates.set(i, -worldRotation);
    }
  }

  private void fabrikBackward(Vector2 goal){
    startPos.set(startPos.size() - 1, goal);
    for(int i = numLinks - 1; i >= 0; i--){
      Vector2 newPos = startPos.get(i + 1).directionTo(startPos.get(i));
      newPos.setToLength(lengths.get(i));
      newPos.add(startPos.get(i + 1));
      startPos.set(i, newPos);
    }
  }

  private void fabrikForward(Vector2 goal){
    startPos.set(0, root);
    for(int i = 1; i <= numLinks; i++){
      Vector2 newPos = startPos.get(i - 1).directionTo(startPos.get(i));
      newPos.setToLength(lengths.get(i - 1));
      newPos.add(startPos.get(i - 1));
      startPos.set(i, newPos);
    }    
  }

  public void fabrik(Vector2 goal) {
    //useWorld = true; //set to trye, which will make rectangles rotated world relatively.
    // distance = target - root
    float distance = startPos.get(0).distanceTo(goal);
    useWorld = true; //FABRIK! The angles stored in rotates correspond to angles in world space.
    float dif = startPos.get(startPos.size() - 1).distanceTo(goal);
    int count = 0;
    while(true){
      fabrikBackward(goal);
      fabrikForward(goal);
      dif = startPos.get(startPos.size() - 1).distanceTo(goal);
      count++;
      if(count > 10){
        break;
      }
      calculateWorldRotate();
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
