import java.util.ArrayList;

public class Links {
  private ArrayList<Float> jointLimits;
  
  // startPos[0] == root,..., startPos[startPos.size()-1] = endPos,
  public ArrayList<Vector2> startPos;
  public ArrayList<Float> length;
  public ArrayList<Float> rotate;
  public int numLinks;
  
  Links(ArrayList<Float> length, ArrayList<Float> rotate, Vector2 root) {
    this.jointLimits = new ArrayList<Float>();
    this.startPos = new ArrayList<Vector2>();
    this.startPos.add(root);
    for(float f : length){
      this.startPos.add(new Vector2(0,0));
      this.jointLimits.add(Float.POSITIVE_INFINITY);
    }
    this.length = length;
    this.rotate = rotate;
    numLinks = length.size();
    fk();
  }
  
  private void fk() {
    if (numLinks > 0) {
      float theta = 0;
      for (int i = 0; i < numLinks; i++) {  
        theta += rotate.get(i);
        startPos.set(i + 1, new Vector2(cos(theta) * length.get(i), sin(theta) * length.get(i)).plus(startPos.get(i)));
      }
    }
  }
  
  public void solve(Vector2 goal) {
    for (int i = numLinks - 1; i >= 0; i--) {      
      Vector2 startToGoal, startToEndEffector;
      float dotProd, angleDiff;
      
      startToGoal = goal.minus(startPos.get(i));
      startToEndEffector = startPos.get(startPos.size() - 1).minus(startPos.get(i));
      dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
      dotProd = clamp(dotProd, -1,1);
      angleDiff = acos(dotProd);
      if (cross(startToGoal,startToEndEffector) < 0)
        rotate.set(i, rotate.get(i) + angleDiff);
      else
        rotate.set(i, rotate.get(i) - angleDiff);
      // Joint limit set here
      if(rotate.get(i)>jointLimits.get(i)){
        rotate.set(i, jointLimits.get(i));
      } else if (rotate.get(i)<-jointLimits.get(i)){
        rotate.set(i, -jointLimits.get(i));
      }
      fk();
    }
  }

  // Link start from 1
  public float getRotateTo(int link){
    float r = 0;
    for(int i = 0; i < link; i++){
      r += rotate.get(i);
    }
    return r;
  }
  
  // Link start from 1
  public void setJointLimit(int link, float limit){
    jointLimits.set(link - 1, limit);
  } 
}