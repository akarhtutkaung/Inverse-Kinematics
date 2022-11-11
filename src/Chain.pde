import java.util.ArrayList;

public abstract class Chain {
  protected ArrayList<Float> jointLimits;
  protected Vector2 root;
  protected ArrayList<Vector2> startPos;
  protected ArrayList<Float> lengths;
  protected ArrayList<Float> rotates;
  protected int numLinks;

  public abstract void solve(Vector2 goal);
  public abstract float getRotate(int link);
  public void setJointLimit(int link, float limit){  
    // Link start from 1
    jointLimits.set(link - 1, limit);
  } 
  protected void fk() {
    if (numLinks > 0) {
      float theta = 0;
      for (int i = 0; i < numLinks; i++) {  
        theta += rotates.get(i);
        startPos.set(i + 1, new Vector2(cos(theta) * lengths.get(i), sin(theta) * lengths.get(i)).plus(startPos.get(i)));
      }
    }
  }
  public Float getLength(int i){
    return lengths.get(i);
  }
  public Vector2 getPos(int i){
    return startPos.get(i);
  }
  public int getNumLinks(){
    return numLinks;
  }
}