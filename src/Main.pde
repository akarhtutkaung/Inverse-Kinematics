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

ArrayList<Links> links = new ArrayList<Links>();;

final float rotateI = 0.3;

//goals
Vector2 leftGoalTop = new Vector2(0, rootM.y + topArmYOffset);
Vector2 rightGoalTop = new Vector2(width, rootM.y + topArmYOffset);
Vector2 leftGoalBot = new Vector2(rootM.x, height);
Vector2 rightGoalBot = new Vector2(rootM.x + rightArmXOffset, height);
int linksChoice = 0; // 0: LT, 1: RT, 2: LB, 3: RB

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
  Links linksRT = new Links(lengthRT, rotateRT, rootRT);
  linksRT.setJointLimit(3, (float)Math.PI/2);
  links.add(linksRT);
  
  ArrayList<Float> lengthRB = new ArrayList<Float>();
  ArrayList<Float> rotateRB = new ArrayList<Float>();
  lengthRB.add(upperArmLength);
  lengthRB.add(lowerArmLength);
  lengthRB.add(handLength);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  Links linksRB = new Links(lengthRB, rotateRB, rootRB);
  linksRB.setJointLimit(2, (float)Math.PI/2);
  linksRB.setJointLimit(3, (float)Math.PI/2);
  links.add(linksRB);

  ArrayList<Float> lengthLT = new ArrayList<Float>();
  ArrayList<Float> rotateLT = new ArrayList<Float>();
  lengthLT.add(upperArmLength);
  lengthLT.add(lowerArmLength);
  lengthLT.add(handLength);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  Links linksLT = new Links(lengthLT, rotateLT, rootLT);
  linksLT.setJointLimit(3, (float)Math.PI/2);
  links.add(linksLT);
  
  ArrayList<Float> lengthLB = new ArrayList<Float>();
  ArrayList<Float> rotateLB = new ArrayList<Float>();
  lengthLB.add(upperArmLength);
  lengthLB.add(lowerArmLength);
  lengthLB.add(handLength);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  Links linksLB = new Links(lengthLB, rotateLB, rootLB);
  linksLB.setJointLimit(2, (float)Math.PI/2);
  linksLB.setJointLimit(3, (float)Math.PI/2);
  links.add(linksLB);
}

void setLeftRightRoot() {
  rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
  rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
  rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
  rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);
}

void switchLinks() {
  switch(linksChoice) {
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
  links.get(0).solve(rightGoalTop);
  links.get(1).solve(rightGoalBot);
  links.get(2).solve(leftGoalTop);
  links.get(3).solve(leftGoalBot);
  switchLinks();
  handleArrowKeys(1.0 / frameRate);
  
  background(250,250,250);
  
  fill(255, 219, 172);
  
  // body
  pushMatrix();
  translate(rootM.x,rootM.y);
  rect(0, 0, 60, 200, 28);
  popMatrix();

  // Draw links
  for(Links l : links){
    for(int i=0; i<l.numLinks; i++){
      pushMatrix();
      translate(l.startPos.get(i).x, l.startPos.get(i).y);
      rotate(l.getRotateTo(i+1));
      rect(0, -armW / 3, l.length.get(i), armW);
      popMatrix();
    }
  }
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
}

boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed;
void keyPressed() {
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true;
  if (keyCode == DOWN) downPressed = true;
  if (keyCode == TAB) {
    if (linksChoice == 3) {
      linksChoice = 0;
    } else {
      linksChoice++;
    }
  }
}

void keyReleased() {
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false;
  if (keyCode == DOWN) downPressed = false;
}
