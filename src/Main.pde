void setup() {
  size(640,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//Root
Vector2 rootM = new Vector2(200,30);
Vector2 rootL = new Vector2(rootM.x, rootM.y + 20);
Vector2 rootR = new Vector2(rootM.x + 60, rootM.y + 20);


float cosTheta = cos(radians(45)); // x
float sinTheta = sin(radians(45)); // y

void setLeftRightRoot(){
  rootL = new Vector2(rootM.x, rootM.y + 20);
  rootR = new Vector2(rootM.x + 60, rootM.y + 20);
}

//Upper Arm
float l0 = 60; 
float a0L = 0.3; //Left - Shoulder joint
float a0R = 0.3; //Right - Shoulder joint

//Lower Arm
float l1 = 90;
float a1L = 0.3; //Left - Elbow joint
float a1R = 0.3; //Right - Elbow joint

//Hand
float l2 = 30;
float a2L = 0.3; //Left - Wrist joint
float a2R = 0.3; //Right - Wrist joint

Vector2 start_l1_L,start_l2_L,endPoint_L;
Vector2 start_l1_R,start_l2_R,endPoint_R;

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
  // Right - wrist
  startToGoal = rightGoal.minus(start_l2_R);
  startToEndEffector = endPoint_R.minus(start_l2_R);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2R += angleDiff;
  else
    a2R -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if(a2R>1.57079633){
  a2R = 1.57079633;
} else if (a2R<-1.57079633){
  a2R = -1.57079633;
}
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  // Left - wrist
  startToGoal = leftGoal.minus(start_l2_L);
  startToEndEffector = endPoint_L.minus(start_l2_L);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
  a2L += angleDiff;
  else
  a2L -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if (a2L > 1.57079633) {
    a2L = 1.57079633;
  } else if (a2L <-  1.57079633) {
    a2L = -1.57079633;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================
  

  // =================================================================================
  //Update elbow joint
  // Right - elbow
  startToGoal = rightGoal.minus(start_l1_R);
  startToEndEffector = endPoint_R.minus(start_l1_R);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1R += angleDiff;
  else
    a1R -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)

  //Left - elbow
  startToGoal = leftGoal.minus(start_l1_L);
  startToEndEffector = endPoint_L.minus(start_l1_L);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1L += angleDiff;
  else
    a1L -= angleDiff;
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================

  
  // =================================================================================
  //Update shoulder joint
  // Right - shoulder
  startToGoal = rightGoal.minus(rootR);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_R.minus(rootR);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0R += angleDiff;
  else
    a0R -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)

  // Left - shoulder
  startToGoal = leftGoal.minus(rootL);
  if (startToGoal.length() <.0001) return;
  startToEndEffector = endPoint_L.minus(rootL);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0L += angleDiff;
  else
    a0L -= angleDiff;
  /*TODO: Shoulder joint limits here*/
  fk(); //Update link positions with fk (e.g. end effector changed)
  // =================================================================================
  
  println("Angle 0:",a0R,"Angle 1:",a1R,"Angle 2:",a2R);
  }
  
  void fk() {
  // Right hand
  start_l1_R = new Vector2(cos(a0R) * l0,sin(a0R) * l0).plus(rootR);
  start_l2_R = new Vector2(cos(a0R + a1R) * l1,sin(a0R + a1R) * l1).plus(start_l1_R);
  endPoint_R = new Vector2(cos(a0R + a1R + a2R) * l2,sin(a0R + a1R + a2R) * l2).plus(start_l2_R);
  // Left hand
  start_l1_L = new Vector2(cos(a0L) * l0,sin(a0L) * l0).plus(rootL);
  start_l2_L = new Vector2(cos(a0L + a1L) * l1,sin(a0L + a1L) * l1).plus(start_l1_L);
  endPoint_L = new Vector2(cos(a0L + a1L + a2L) * l2,sin(a0L + a1L + a2L) * l2).plus(start_l2_L);
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
  
  // right shoulder
  pushMatrix();
  translate(rootR.x, rootR.y);
  rotate(a0R);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  // left shoulder
  pushMatrix();
  translate(rootL.x, rootL.y);
  rotate(a0L);
  rect(0, -armW / 3, l0, armW);
  popMatrix();
  
  // right elbow
  pushMatrix();
  translate(start_l1_R.x,start_l1_R.y);
  rotate(a0R + a1R);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  // left elbow
  pushMatrix();
  translate(start_l1_L.x,start_l1_L.y);
  rotate(a0L + a1L);
  rect(0, -armW / 2, l1, armW);
  popMatrix();
  
  // right wrist
  pushMatrix();
  translate(start_l2_R.x,start_l2_R.y);
  rotate(a0R + a1R + a2R);
  rect(0, -armW / 2, l2, armW);
  popMatrix();
  // left wrist
  pushMatrix();
  translate(start_l2_L.x,start_l2_L.y);
  rotate(a0L + a1L + a2L);
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
