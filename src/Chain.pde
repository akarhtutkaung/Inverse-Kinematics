import java.util.ArrayList;

public class Chain {
  private ArrayList<Float> jointLimits;
  private float totallengths = 0;
  private float tolerance = 0.1;
  private Vector2 root;
  private ArrayList<Vector2> prevStartPos;
  
  // startPos[0] == root,..., startPos[startPos.size()-1] = endPos,
  public ArrayList<Vector2> startPos;
  public ArrayList<Float> lengths;
  public ArrayList<Float> rotates;
  public int numLinks;
  
  Chain(ArrayList<Float> lengths, ArrayList<Float> rotates, Vector2 root) {
    this.jointLimits = new ArrayList<Float>();
    this.startPos = new ArrayList<Vector2>();
    this.prevStartPos = new ArrayList<Vector2>();
    this.startPos.add(root);
    this.prevStartPos.add(root);
    this.root = root;
    for(float length : lengths){
      this.startPos.add(new Vector2(0,0));
      this.prevStartPos.add(new Vector2(0,0));
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

  private void calculateRotate(int startPosIndex, int endPosIndex, Vector2 goal){
    Vector2 startToInitial, startToNew;
    float dotProd, angleDiff;
    startToInitial = (prevStartPos.get(endPosIndex).plus(startPos.get(startPosIndex).minus(prevStartPos.get(startPosIndex)))).minus(startPos.get(startPosIndex));
    startToNew = startPos.get(endPosIndex).minus(startPos.get(startPosIndex));
    dotProd = dot(startToNew.normalized(), startToInitial.normalized());
    dotProd = clamp(dotProd, -1,1);
    angleDiff = acos(dotProd);
    if (cross(startToInitial, startToNew) > 0)
      rotates.set(startPosIndex, rotates.get(startPosIndex) + angleDiff);
    else
      rotates.set(startPosIndex, rotates.get(startPosIndex) - angleDiff);
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
        for(int i=0; i<startPos.size(); i++){
          prevStartPos.set(i, new Vector2(startPos.get(i).x, startPos.get(i).y));
        }
        fabrikBackward(goal);
        fabrikForward(goal);
        for(int i=0; i < startPos.size() - 1; i++){
          calculateRotate(i, i + 1, goal);
        }
        // fk();
        dif = startPos.get(startPos.size() - 1).distanceTo(goal);
        count++;
        if(count > 10){
          // break;
        }
      }
    }
  }

  // Link start from 1
  public float getRotateTo(int link){
    float r = 0;
    for(int i = 0; i < link; i++){
      r += rotates.get(i);
    }
    return r;
    // return rotates.get(link-1);
  }
  
  // Link start from 1
  public void setJointLimit(int link, float limit){
    jointLimits.set(link - 1, limit);
  } 
}