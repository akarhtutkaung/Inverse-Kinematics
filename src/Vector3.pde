
///////////////////
// Vector3 Library
///////////////////

public class Vector3 {
  public float x, y, z;
  
  public Vector3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +", "+z+")";
  }
  
  public float length(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public float lengthSqr(){
    return x*x+y*y+z*z;
  }
  
  public Vector3 plus(Vector3 rhs){
    return new Vector3(x+rhs.x, y+rhs.y, z+rhs.z);
  }
  
  public void add(Vector3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vector3 minus(Vector3 rhs){
    return new Vector3(x-rhs.x, y-rhs.y, z-rhs.z);
  }
  
  public void subtract(Vector3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vector3 times(float rhs){
    return new Vector3(x*rhs, y*rhs, z*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y + z*z);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  }
  
  public Vector3 normalized(){
    float magnitude = sqrt(x*x + y*y + z*z);
    return new Vector3(x/magnitude, y/magnitude, z/magnitude);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
      z *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y + z*z);
    x *= newL/magnitude;
    y *= newL/magnitude;
    z *= newL/magnitude;
  }
  
  public float distanceTo(Vector3 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float dz = rhs.z - z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }

  public Vector3 directionTo(Vector3 p){
    float dx = p.x - x;
    float dy = p.y - y;
    float dz = p.y - z;
    Vector3 dir = new Vector3(dx, dy, dz);
    dir = dir.normalized();
    return dir;
  }
}

Vector3 interpolate(Vector3 a, Vector3 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vector3 a, Vector3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector3 projAB(Vector3 a, Vector3 b){
  return b.times(a.x*b.x + a.y*b.y + a.z*b.z);
}
