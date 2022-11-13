public class FABRIK extends Chain {
  private ArrayList<Vector2> prevStartPos;
  private float totallengths = 0;

  FABRIK(ArrayList<Float> lengths, ArrayList<Float> rotates, Vector2 root, String name) {
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
    this.name = name;
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
    int count = 0;
    while(true){
      for(int i=0; i<startPos.size(); i++){
        prevStartPos.set(i, new Vector2(startPos.get(i).x, startPos.get(i).y));
      }
      fabrikBackward(goal);
      fabrikForward(goal);
      count++;
      if(count > 10){
        break;
      }
      calculateWorldRotate();
    }
  }

  private void calculateWorldRotate(){
    //Calling this function will calculate the angle of all the rectangles in world space.
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

  // Link start from 1
  public float getRotate(int link){
    return rotates.get(link-1);
  }
}
