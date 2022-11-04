
///////////////////
// Vector2D Library
///////////////////

public class Vector2 {
  public float x, y;
  
  public Vector2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  public Vector2 plus(Vector2 rhs){
    return new Vector2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vector2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vector2 minus(Vector2 rhs){
    return new Vector2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vector2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vector2 times(float rhs){
    return new Vector2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vector2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vector2(x/magnitude, y/magnitude);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public float distanceTo(Vector2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }

  public Vector2 directionTo(Vector2 p){
    float x = p.x - this.x;
    float y = p.y - this.y;
    Vector2 dir = new Vector2(x, y);
    dir = dir.normalized();
    return dir;
  }
}

Vector2 interpolate(Vector2 a, Vector2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float dot(Vector2 a, Vector2 b){
  return a.x*b.x + a.y*b.y;
}

Vector2 projAB(Vector2 a, Vector2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float cross(Vector2 a, Vector2 b) {
  return a.x * b.y - a.y * b.x;
}

float clamp(float f, float min, float max) {
  if (f < min) return min;
  if (f > max) return max;
  return f;
}