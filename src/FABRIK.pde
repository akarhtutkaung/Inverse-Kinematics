public class FABRIK extends Chain {
  private ArrayList<Vector2> prevStartPos;
  private float totallengths = 0;

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

  //connecting to goal
  private void fabrikBackward(Vector2 goal){
    startPos.set(startPos.size() - 1, goal);
    for(int i = numLinks - 1; i >= 0; i--){
      Vector2 newPos = startPos.get(i + 1).directionTo(startPos.get(i));
        //Vector2 refDir; //the reference direction to measure angle from.
        ////Vector2 cross; //To determine 
        ////Joint limit; at this point, newPos is still just the direction vector. It is to be constrained.
        //if (i == numLinks - 1) { //joint limit for root
        //  refDir = new Vector2(1,0);
        //} else { //joint limit for some joint
        //  refDir = startPos.get(i + 2).directionTo(startPos.get(i + 1));
        //}
        //float crossProd = cross(newPos, refDir);
        //float refAngle = acos(dot(refDir, new Vector2(1,0))); //the angle of the reference direction.
        //println(refAngle);
        //float angleDiff = acos(dot(newPos, refDir)); //angle between newPos and reference direction.
        ////float newPosNewX;
        ////float newPosNewY;
        //if(crossProd < 0 /*& angleDiff > jointLimits.get(i)*/){ //counterclockwise (positive) angle. If greater than joint limit...
        //  if (angleDiff > jointLimits.get(i)){
        //    println("broke!");
        //    newPos.x = cos(refAngle + jointLimits.get(i))*refDir.x - sin(refAngle + jointLimits.get(i))*refDir.y; //x2 = cosβx1−sinβy1
        //    newPos.y = sin(refAngle + jointLimits.get(i))*refDir.x + cos(refAngle + jointLimits.get(i))*refDir.y; //y2 = sinβx1+cosβy1
        //  }
        //} else  { //clockwise (negative) angle
        //  if (angleDiff < -jointLimits.get(i)){
        //    println("broke!");
        //    newPos.x = cos(refAngle -jointLimits.get(i))*refDir.x - sin(refAngle -jointLimits.get(i))*refDir.y; //x2 = cosβx1−sinβy1
        //    newPos.y = sin(refAngle -jointLimits.get(i))*refDir.x + cos(refAngle -jointLimits.get(i))*refDir.y; //y2 = sinβx1+cosβy1
        //  }
        //}
      newPos.setToLength(lengths.get(i));
      newPos.add(startPos.get(i + 1));
      startPos.set(i, newPos);
    }
  }

  //connecting to start
  private void fabrikForward(Vector2 goal){
    startPos.set(0, root);
    for(int i = 1; i <= numLinks; i++){
      Vector2 newPos = startPos.get(i - 1).directionTo(startPos.get(i));
        Vector2 refDir; //the reference direction to measure angle from.\
        //Vector2 cross; //To determine 
        //Joint limit; at this point, newPos is still just the direction vector. It is to be constrained.
        if (i == 1) { //joint limit for root
          refDir = new Vector2(1,0);
        } else { //joint limit for some joint
          refDir = startPos.get(i - 2).directionTo(startPos.get(i - 1));
        }
        float crossProd = cross(newPos, refDir);
        float refAngle = acos(dot(refDir, new Vector2(1,0))); //the angle of the reference direction.
        //println(refAngle);
        float angleDiff = acos(dot(newPos, refDir)); //angle between newPos and reference direction.
        //float newPosNewX;
        //float newPosNewY;
        if(crossProd > 0/* & angleDiff > jointLimits.get(i - 1)*/){ //counterclockwise (positive) angle. If greater than joint limit...
          if (angleDiff > jointLimits.get(i - 1)){
            println(refAngle);
            println("broke!3 New angle = " + (refAngle + jointLimits.get(i-1)/2));
            newPos.x = cos(refAngle + jointLimits.get(i - 1)/2)*refDir.x - sin(refAngle + jointLimits.get(i - 1)/2)*refDir.y; //x2 = cosβx1−sinβy1
            newPos.y = sin(refAngle + jointLimits.get(i - 1)/2)*refDir.x + cos(refAngle + jointLimits.get(i - 1)/2)*refDir.y; //y2 = sinβx1+cosβy1
          }
        } else  { //clockwise (negative) angle
          if (-angleDiff < -jointLimits.get(i - 1)){
            println(refAngle);
            println("broke!4");
            newPos.x = cos(refAngle -jointLimits.get(i - 1)/2)*refDir.x - sin(refAngle -jointLimits.get(i - 1)/2)*refDir.y; //x2 = cosβx1−sinβy1
            newPos.y = sin(refAngle -jointLimits.get(i - 1)/2)*refDir.x + cos(refAngle -jointLimits.get(i - 1)/2)*refDir.y; //y2 = sinβx1+cosβy1
          }
        }
      
      //end of Joint Limit calc
      newPos.setToLength(lengths.get(i - 1));
      newPos.add(startPos.get(i - 1));
      startPos.set(i, newPos);
      
      //joint limit
      //if (i < numLinks - 1){
      //  Vector2 previousRectDir = startPos.get(i-2).minus(startPos.get(i-1)).normalized();
      //  Vector2 curRectDir = startPos.get(i-1).minus(startPos.get(i)).normalized();
      //  float rotationFromBefore = acos(dot(worldDirection, unitDirection)); //this is the rotation relative to the world.
      //}
      
      
           ////joint limits (THIS REFERENCE TO THIS)
     // float rotateDiff = i != 0 ? rotates.get(i) - rotates.get(i-1) : -1f; //get the difference between this angle and last.
     // if(rotateDiff != -1 && rotateDiff > jointLimits.get(i)){
     //   rotates.set(i, rotates.get(i-1) + jointLimits.get(i));
     // } else if (rotateDiff != -1 && rotateDiff < -jointLimits.get(i)){
     //   rotates.set(i, rotates.get(i-1) - jointLimits.get(i));
     // }
    }    
  }

  public void solve(Vector2 goal) {
    //int count = 0;
    //while(true){ //removed to make the animation "smoother".
      for(int i=0; i<startPos.size(); i++){
        prevStartPos.set(i, new Vector2(startPos.get(i).x, startPos.get(i).y));
      }
      fabrikBackward(goal);
      fabrikForward(goal);
      //count++;
      //if(count > 10){
      //  break;
      //}
      calculateWorldRotate();
    //}
  }

  private void calculateWorldRotate(){
    //Calling this function will calculate the angle of all the rectangles in world space.
    for (int i = 0; i < startPos.size() - 1; i++){
      Vector2 worldDirection = (startPos.get(i+1).minus(startPos.get(i))).normalized(); //get the world target direction.
      Vector2 unitDirection = new Vector2(1,0); //the direction of a 0 angle in the world.
      float worldRotation = acos(dot(worldDirection, unitDirection)); //this is the rotation relative to the world.
      //boolean crossLessZero;
      if (cross(worldDirection, unitDirection) < 0){
        rotates.set(i, worldRotation);
        //crossLessZero = true;
      } else {
        rotates.set(i, -worldRotation);
        //crossLessZero = false;
      }
     
     ////joint limits
     // float rotateDiff = i != 0 ? rotates.get(i) - rotates.get(i-1) : -1f; //get the difference between this angle and last.
     // if(rotateDiff != -1 && rotateDiff > jointLimits.get(i)){
     //   rotates.set(i, rotates.get(i-1) + jointLimits.get(i));
     // } else if (rotateDiff != -1 && rotateDiff < -jointLimits.get(i)){
     //   rotates.set(i, rotates.get(i-1) - jointLimits.get(i));
     // }
    }
  }

  // Link start from 1
  public float getRotate(int link){
    return rotates.get(link-1);
  }
}
