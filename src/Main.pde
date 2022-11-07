void setup() {
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

final float topArmYOffset = 20;
final float botArmYOffset = 150;
final float rightArmXOffset = 60;

final float upperArmLength = 60;
final float lowerArmLength = 90;
final float handLength = 30;

//Root
Vector2 rootM = new Vector2(200,30);
Vector2 rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
Vector2 rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
Vector2 rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
Vector2 rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);


float cosTheta = cos(radians(45)); // x
float sinTheta = sin(radians(45)); // y

void setLeftRightRoot(){
  rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
  rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
  rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
  rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);
}

//Upper Arm
float l0 = upperArmLength; 
float a0LT = 0.3; //Left Top - Shoulder joint
float a0RT = 0.3; //Right Bot - Shoulder joint
float a0LB = 0.3; //Left Top - Shoulder joint
float a0RB = 0.3; //Right Bot - Shoulder joint

//Lower Arm
float l1 = lowerArmLength;
float a1LT = 0.3; //Left Top - Elbow joint
float a1RT = 0.3; //Right Bot - Elbow joint
float a1LB = 0.3; //Left Top - Elbow joint
float a1RB = 0.3; //Right Bot - Elbow joint

//Hand
float l2 = handLength;
float a2LT = 0.3; //Left Top - Wrist joint
float a2RT = 0.3; //Right Bot - Wrist joint
float a2LB = 0.3; //Left Top - Wrist joint
float a2RB = 0.3; //Right Bot - Wrist joint

Vector2 start_l1_LT,start_l2_LT,endPoint_LT;
Vector2 start_l1_LB,start_l2_LB,endPoint_LB;
Vector2 start_l1_RT,start_l2_RT,endPoint_RT;
Vector2 start_l1_RB,start_l2_RB,endPoint_RB;

//goals
Vector2 leftGoal = new Vector2(mouseX, mouseY);
Vector2 rightGoal = new Vector2(mouseX, mouseY);
boolean trueMoveLeftElseMoveRight = true;
boolean moveBoth = false;

void solve() {
  //Vector2 goal = new Vector2(mouseX, mouseY);
  if (moveBoth){
    leftGoal = rightGoal = new Vector2(mouseX, mouseY);
  } else if (trueMoveLeftElseMoveRight){
    leftGoal = new Vector2(mouseX, mouseY);
  } else {
    rightGoal = new Vector2(mouseX, mouseY);
  }
  
  Vector2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  // =================================================================================
  //Update wrist joint
  // Right Top - wrist
  startToGoal = rightGoal.minus(start_l2_RT);
  startToEndEffector = endPoint_RT.minus(start_l2_RT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2RT += angleDiff;
  else
    a2RT -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if(a2RT>1.57079633){
  a2RT = 1.57079633;
} else if (a2RT<-1.57079633){
  a2RT = -1.57079633;
}
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  // Left Top - wrist
  startToGoal = leftGoal.minus(start_l2_LT);
  startToEndEffector = endPoint_LT.minus(start_l2_LT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
  a2LT += angleDiff;
  else
  a2LT -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if (a2LT > 1.57079633) {
    a2LT = 1.57079633;
  } else if (a2LT <-  1.57079633) {
    a2LT = -1.57079633;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Right Bot - wrist
  startToGoal = rightGoal.minus(start_l2_RB);
  startToEndEffector = endPoint_RB.minus(start_l2_RB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2RB += angleDiff;
  else
    a2RB -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if(a2RB>1.57079633){
  a2RB = 1.57079633;
} else if (a2RT<-1.57079633){
  a2RB = -1.57079633;
}
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  // Left Bot - wrist
  startToGoal = leftGoal.minus(start_l2_LB);
  startToEndEffector = endPoint_LB.minus(start_l2_LB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
  a2LB += angleDiff;
  else
  a2LB -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if (a2LB > 1.57079633) {
    a2LB = 1.57079633;
  } else if (a2LB <-  1.57079633) {
    a2LB = -1.57079633;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================
  

  // =================================================================================
  //Update elbow joint
  // Right Top- elbow
  startToGoal = rightGoal.minus(start_l1_RT);
  startToEndEffector = endPoint_RT.minus(start_l1_RT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1RT += angleDiff;
  else
    a1RT -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)

  //Left Top - elbow
  startToGoal = leftGoal.minus(start_l1_LT);
  startToEndEffector = endPoint_LT.minus(start_l1_LT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1LT += angleDiff;
  else
    a1LT -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Right Bot - elbow
  startToGoal = rightGoal.minus(start_l1_RB);
  startToEndEffector = endPoint_RB.minus(start_l1_RB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1RB += angleDiff;
  else
    a1RB -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)

  //Left Bot - elbow
  startToGoal = leftGoal.minus(start_l1_LB);
  startToEndEffector = endPoint_LB.minus(start_l1_LB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1LB += angleDiff;
  else
    a1LB -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================

  
  // =================================================================================
  //Update shoulder joint
  // Right - shoulder
  startToGoal = rightGoal.minus(rootRT);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_RT.minus(rootRT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0RT += angleDiff;
  else
    a0RT -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Left - shoulder
  startToGoal = leftGoal.minus(rootLT);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_LT.minus(rootLT);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0LT += angleDiff;
  else
    a0LT -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Right - shoulder
  startToGoal = rightGoal.minus(rootRB);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_RB.minus(rootRB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0RB += angleDiff;
  else
    a0RB -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Left - shoulder
  startToGoal = leftGoal.minus(rootLB);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_LB.minus(rootLB);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0LB += angleDiff;
  else
    a0LB -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================
  
  println("Angle 0:",a0RT,"Angle 1:",a1RT,"Angle 2:",a2RT);
  }
  
  void fk() {
  // Right Top hand
  start_l1_RT = new Vector2(cos(a0RT) * l0,sin(a0RT) * l0).plus(rootRT);
  start_l2_RT = new Vector2(cos(a0RT + a1RT) * l1,sin(a0RT + a1RT) * l1).plus(start_l1_RT);
  endPoint_RT = new Vector2(cos(a0RT + a1RT + a2RT) * l2,sin(a0RT + a1RT + a2RT) * l2).plus(start_l2_RT);
  // Left Top hand
  start_l1_LT = new Vector2(cos(a0LT) * l0,sin(a0LT) * l0).plus(rootLT);
  start_l2_LT = new Vector2(cos(a0LT + a1LT) * l1,sin(a0LT + a1LT) * l1).plus(start_l1_LT);
  endPoint_LT = new Vector2(cos(a0LT + a1LT + a2LT) * l2,sin(a0LT + a1LT + a2LT) * l2).plus(start_l2_LT);
  // Right Bot hand
  start_l1_RB = new Vector2(cos(a0RB) * l0,sin(a0RB) * l0).plus(rootRB);
  start_l2_RB = new Vector2(cos(a0RB + a1RB) * l1,sin(a0RB + a1RB) * l1).plus(start_l1_RB);
  endPoint_RB = new Vector2(cos(a0RB + a1RB + a2RB) * l2,sin(a0RB + a1RB + a2RB) * l2).plus(start_l2_RB);
  // Left Bot hand
  start_l1_LB = new Vector2(cos(a0LB) * l0,sin(a0LB) * l0).plus(rootLB);
  start_l2_LB = new Vector2(cos(a0LB + a1LB) * l1,sin(a0LB + a1LB) * l1).plus(start_l1_LB);
  endPoint_LB = new Vector2(cos(a0LB + a1LB + a2LB) * l2,sin(a0LB + a1LB + a2LB) * l2).plus(start_l2_LB);
  }
  
  float armW = 20;
  void draw() {
  fk();
  solve();
  handleArrowKeys(1.0/frameRate);
  
  background(250,250,250);
  
  fill(255, 219, 172);
  
  // body
  pushMatrix();
  translate(rootM.x,rootM.y);
  rect(0, 0, 60, 200, 28);
  popMatrix();
  
  // right top shoulder
  pushMatrix();
  translate(rootRT.x, rootRT.y);
  rotate(a0RT);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  // right bot shoulder
  pushMatrix();
  translate(rootRB.x, rootRB.y);
  rotate(a0RB);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  // left top shoulder
  pushMatrix();
  translate(rootLT.x, rootLT.y);
  rotate(a0LT);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  // left bot shoulder
  pushMatrix();
  translate(rootLB.x, rootLB.y);
  rotate(a0LB);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  
  // right top elbow
  pushMatrix();
  translate(start_l1_RT.x,start_l1_RT.y);
  rotate(a0RT + a1RT);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  // right bot elbow
  pushMatrix();
  translate(start_l1_RB.x,start_l1_RB.y);
  rotate(a0RB + a1RB);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  // left top elbow
  pushMatrix();
  translate(start_l1_LT.x,start_l1_LT.y);
  rotate(a0LT + a1LT);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  // left bot elbow
  pushMatrix();
  translate(start_l1_LB.x,start_l1_LB.y);
  rotate(a0LB + a1LB);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  
  // right top wrist
  pushMatrix();
  translate(start_l2_RT.x,start_l2_RT.y);
  rotate(a0RT + a1RT + a2RT);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  // right bot wrist
  pushMatrix();
  translate(start_l2_RB.x,start_l2_RB.y);
  rotate(a0RB + a1RB + a2RB);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  // left top wrist
  pushMatrix();
  translate(start_l2_LT.x,start_l2_LT.y);
  rotate(a0LT + a1LT + a2LT);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  // left bot wrist
  pushMatrix();
  translate(start_l2_LB.x,start_l2_LB.y);
  rotate(a0LB + a1LB + a2LB);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  
  }
float obstacleSpeed = 300;

void handleArrowKeys(float dt){
  Vector2 obstacleVel = new Vector2(0, 0);
  if (leftPressed) obstacleVel = new Vector2(-obstacleSpeed, 0);
  else if (rightPressed) obstacleVel = new Vector2(obstacleSpeed, 0);
  else if (upPressed) obstacleVel = new Vector2(0, -obstacleSpeed);
  else if (downPressed) obstacleVel = new Vector2(0, obstacleSpeed);

  if (downPressed && leftPressed) obstacleVel = new Vector2(-obstacleSpeed * cosTheta, obstacleSpeed * sinTheta);
  else if (downPressed && rightPressed) obstacleVel = new Vector2(obstacleSpeed * cosTheta, obstacleSpeed * sinTheta);
  else if (upPressed && leftPressed) obstacleVel = new Vector2(-obstacleSpeed * cosTheta, -obstacleSpeed * sinTheta);
  else if (upPressed && rightPressed) obstacleVel = new Vector2(obstacleSpeed * cosTheta, -obstacleSpeed * sinTheta);

  if (leftPressed && rightPressed) {
    if (upPressed) obstacleVel = new Vector2(0, -obstacleSpeed);
    else if (downPressed) obstacleVel = new Vector2(0, obstacleSpeed);
    else obstacleVel = new Vector2(0, 0);
  }
  if (upPressed && downPressed) {
    if (leftPressed) obstacleVel = new Vector2(-obstacleSpeed, 0);
    else if (rightPressed) obstacleVel = new Vector2(obstacleSpeed, 0);
    else obstacleVel = new Vector2(0, 0);
  }

  if (upPressed && downPressed && leftPressed && rightPressed) obstacleVel = new Vector2(0, 0);
  
  rootM.x += obstacleVel.x * dt;
  rootM.y += obstacleVel.y * dt;
  setLeftRightRoot();
}

boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed;
void keyPressed() {
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true;
  if (keyCode == DOWN) downPressed = true;
  if (keyCode == TAB) trueMoveLeftElseMoveRight = !trueMoveLeftElseMoveRight;
  if (keyCode == 32) moveBoth = !moveBoth; //space
}

void keyReleased() {
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false;
  if (keyCode == DOWN) downPressed = false;
}
