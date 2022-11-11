public class FABRIK extends Chain {
  private ArrayList<Vector2> prevStartPos;
  private float totallengths = 0;
  private float tolerance = 0.1;

  FABRIK(ArrayList<Float> lengths, ArrayList<Float> rotates, Vector2 root) {
    jointLimits = new ArrayList<Float>();
    startPos = new ArrayList<Vector2>();
    prevStartPos = new ArrayList<Vector2>();
    startPos.add(root);
    prevStartPos.add(root);
    this.root = root;
    for(float length : lengths){
      startPos.add(new Vector2(0,0));
      prevStartPos.add(new Vector2(0,0));
      jointLimits.add(Float.POSITIVE_INFINITY);
      totallengths += length;
    }
    this.lengths = lengths;
    this.rotates = rotates;
    numLinks = lengths.size();
    fk();
  }

    private void calculateRotate(int startPosIndex, int endPosIndex){
      Vector2 startToInitial, startToNew;
      float dotProd, angleDiff;
      startToInitial = (prevStartPos.get(endPosIndex).plus(startPos.get(startPosIndex).minus(prevStartPos.get(startPosIndex)))).minus(startPos.get(startPosIndex));
      startToNew = startPos.get(endPosIndex).minus(startPos.get(startPosIndex));
      dotProd = dot(startToNew.normalized(), startToInitial.normalized());
      dotProd = clamp(dotProd, -1,1);
      angleDiff = acos(dotProd);
      if (cross(startToNew, startToInitial) < 0)
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

  public void solve(Vector2 goal) {
    float distance = startPos.get(0).distanceTo(goal);
    if(distance > totallengths){
      Vector2 direction = root.directionTo(goal);
      Vector2 newPos = root;
      for(int i=0; i<startPos.size() - 1; i++){
        newPos = newPos.plus(direction.times(lengths.get(i)));
        startPos.set(i + 1, newPos);
        
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
          calculateRotate(i, i + 1);
        }
        dif = startPos.get(startPos.size() - 1).distanceTo(goal);
        count++;
        if(count > 10){
          break;
        }
      }
    }
  }

  // Link start from 1
  public float getRotate(int link){
    return rotates.get(link-1);
  }
}