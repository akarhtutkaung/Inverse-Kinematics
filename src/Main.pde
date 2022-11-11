import java.util.ArrayList;
import java.util.*;

final int width = 640;
final int height = 480;
final float topArmYOffset = 20;
final float botArmYOffset = 150;
final float rightArmXOffset = 60;

final float upperArmLength = 60;
final float lowerArmLength = 90;
final float handLength = 30;

final float armW = 20;

final float cosTheta = cos(radians(45)); // x
final float sinTheta = sin(radians(45)); // y

//Roots
Vector2 rootM = new Vector2(200,30);
Vector2 rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
Vector2 rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
Vector2 rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
Vector2 rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);

ArrayList<Chain> chains = new ArrayList<Chain>();;

final float rotateI = 0.3;

//goals
Vector2 leftGoalTop = new Vector2(0, rootM.y + topArmYOffset);
Vector2 rightGoalTop = new Vector2(width, rootM.y + topArmYOffset);
Vector2 leftGoalBot = new Vector2(rootM.x, height);
Vector2 rightGoalBot = new Vector2(rootM.x + rightArmXOffset, height);
int chainChoice = 0; // 0: LT, 1: RT, 2: LB, 3: RB

//TEST
Chain globalRightArmChain;

void setup() {
  size(649,480);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  
  ArrayList<Float> lengthRT = new ArrayList<Float>();
  ArrayList<Float> rotateRT = new ArrayList<Float>();
  lengthRT.add(upperArmLength);
  lengthRT.add(lowerArmLength);
  lengthRT.add(handLength);
  rotateRT.add(rotateI);
  rotateRT.add(rotateI);
  rotateRT.add(rotateI);
  Chain chainRT = new Chain(lengthRT, rotateRT, rootRT);
  chainRT.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainRT);
  
  ArrayList<Float> lengthRB = new ArrayList<Float>();
  ArrayList<Float> rotateRB = new ArrayList<Float>();
  lengthRB.add(upperArmLength);
  lengthRB.add(lowerArmLength);
  lengthRB.add(handLength);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  Chain chainRB = new Chain(lengthRB, rotateRB, rootRB);
  chainRB.setJointLimit(2, (float)Math.PI/2);
  chainRB.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainRB);
  globalRightArmChain = chainRB;//DEBUG

  ArrayList<Float> lengthLT = new ArrayList<Float>();
  ArrayList<Float> rotateLT = new ArrayList<Float>();
  lengthLT.add(upperArmLength);
  lengthLT.add(lowerArmLength);
  lengthLT.add(handLength);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  Chain chainLT = new Chain(lengthLT, rotateLT, rootLT);
  chainLT.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainLT);
  
  ArrayList<Float> lengthLB = new ArrayList<Float>();
  ArrayList<Float> rotateLB = new ArrayList<Float>();
  lengthLB.add(upperArmLength);
  lengthLB.add(lowerArmLength);
  lengthLB.add(handLength);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  Chain chainLB = new Chain(lengthLB, rotateLB, rootLB);
  chainLB.setJointLimit(2, (float)Math.PI/2);
  chainLB.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainLB);
}

void setLeftRightRoot() {
  println("called");
  rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
  rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
  rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
  rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);
}

void switchLinks() {
  switch(chainChoice) {
    case(0) : 
      leftGoalTop = new Vector2(mouseX, mouseY);
    break;
    case(1) : 
      rightGoalTop = new Vector2(mouseX, mouseY);
    break;
    case(2) : 
      leftGoalBot = new Vector2(mouseX, mouseY);
    break;
    case(3) : 
      rightGoalBot = new Vector2(mouseX, mouseY);
    break;
  }
}

void draw() {
  chains.get(0).fabrik(rightGoalTop);
  chains.get(1).ccd(rightGoalBot);
  chains.get(2).ccd(leftGoalTop);
  chains.get(3).ccd(leftGoalBot);
  switchLinks();
  handleArrowKeys(1.0 / frameRate);
  
  background(250,250,250);
  
  fill(255, 219, 172);
  
  // body
  pushMatrix();
  translate(rootM.x,rootM.y);
  rect(0, 0, 60, 200, 28);
  popMatrix();

  // Draw chains
  for(Chain l : chains){
    for(int i=0; i<l.numLinks; i++){
      pushMatrix();
      translate(l.startPos.get(i).x, l.startPos.get(i).y);
      if (l.useWorld){
        rotate(l.rotates.get(i)); //rotation is independent of other rotates, i.e. world space.
      } else {
        rotate(l.getRotateTo(i+1)); //rotation is added from the previous rotates.
      }
      
      circle(0,0,10);
      rect(0, -armW / 3, l.lengths.get(i), armW);
      if (i + 1 >= l.numLinks){
        popMatrix();
        pushMatrix();
        translate(l.startPos.get(i+1).x, l.startPos.get(i+1).y);
        circle(0,0,10);
      }
      popMatrix();
    }
  }

  //render joints
  // for()
}
float obstacleSpeed = 300;

void handleArrowKeys(float dt) {
  Vector2 obstacleVel = new Vector2(0, 0);
  if (leftPressed) obstacleVel = new Vector2( - obstacleSpeed, 0);
  else if (rightPressed) obstacleVel = new Vector2(obstacleSpeed, 0);
  else if (upPressed) obstacleVel = new Vector2(0, -obstacleSpeed);
  else if (downPressed) obstacleVel = new Vector2(0, obstacleSpeed);
  
  if (downPressed && leftPressed) obstacleVel = new Vector2( - obstacleSpeed * cosTheta, obstacleSpeed * sinTheta);
  else if (downPressed && rightPressed) obstacleVel = new Vector2(obstacleSpeed * cosTheta, obstacleSpeed * sinTheta);
  else if (upPressed && leftPressed) obstacleVel = new Vector2( - obstacleSpeed * cosTheta, -obstacleSpeed * sinTheta);
  else if (upPressed && rightPressed) obstacleVel = new Vector2(obstacleSpeed * cosTheta, -obstacleSpeed * sinTheta);
  
  if (leftPressed && rightPressed) {
    if (upPressed) obstacleVel = new Vector2(0, -obstacleSpeed);
    else if (downPressed) obstacleVel = new Vector2(0, obstacleSpeed);
    else obstacleVel = new Vector2(0, 0);
  }
  if (upPressed && downPressed) {
    if (leftPressed) obstacleVel = new Vector2( - obstacleSpeed, 0);
    else if (rightPressed) obstacleVel = new Vector2(obstacleSpeed, 0);
    else obstacleVel = new Vector2(0, 0);
  }
  
  if (upPressed && downPressed && leftPressed && rightPressed) obstacleVel = new Vector2(0, 0);
  
  rootM.x += obstacleVel.x * dt;
  rootM.y += obstacleVel.y * dt;
  setLeftRightRoot();
  for (Chain chain : chains){
    chain.startPos.get(0).x += obstacleVel.x * dt;
    chain.startPos.get(0).y += obstacleVel.y * dt;
    
  }
}

boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed;
void keyPressed() {
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true;
  if (keyCode == DOWN) downPressed = true;
  if (keyCode == TAB) {
    if (chainChoice == 3) {
      chainChoice = 0;
    } else {
      chainChoice++;
    }
  }
}

void keyReleased() {
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false;
  if (keyCode == DOWN) downPressed = false;
}
