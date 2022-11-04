void setup() {
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//Root
Vector2 root = new Vector2(0,0);

//Upper Arm
float l0 = 60; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 90;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 30;
float a2 = 0.3; //Wrist joint

Vector2 start_l1,start_l2,endPoint;

void solve() {
  Vector2 goal = new Vector2(mouseX, mouseY);
  
  Vector2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += angleDiff;
  else
    a2 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if(a2>1.57079633){
  a2 = 1.57079633;
} else if (a2<-1.57079633){
  a2 = -1.57079633;
}
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
  a1 += angleDiff;
  else
  a1 -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
  a0 += angleDiff;
  else
  a0 -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2);
}

void fk() {
  start_l1 = new Vector2(cos(a0) * l0,sin(a0) * l0).plus(root);
  start_l2 = new Vector2(cos(a0 + a1) * l1,sin(a0 + a1) * l1).plus(start_l1);
  endPoint = new Vector2(cos(a0 + a1 + a2) * l2,sin(a0 + a1 + a2) * l2).plus(start_l2);
}

float armW = 20;
void draw() {
  fk();
  solve();
  
  background(250,250,250);
  
  // shoulder
  fill(255, 219, 172);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  
  // elbow
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0 + a1);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  
  // wrist
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0 + a1 + a2);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  
}