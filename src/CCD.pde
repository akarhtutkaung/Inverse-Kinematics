public class CCD extends Chain {
  CCD(ArrayList<Float> lengths, ArrayList<Float> rotates, Vector2 root) {
    jointLimits = new ArrayList<Float>();
    startPos = new ArrayList<Vector2>();
    startPos.add(root);
    this.root = root;
    for(float length : lengths){
      startPos.add(new Vector2(0,0));
      jointLimits.add(Float.POSITIVE_INFINITY);
    }
    this.lengths = lengths;
    this.rotates = rotates;
    numLinks = lengths.size();
    fk();
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

  // Link start from 1
  public float getRotate(int link){
    float r = 0;
    for(int i = 0; i < link; i++){
      r += rotates.get(i);
    }
    return r;
  }
}