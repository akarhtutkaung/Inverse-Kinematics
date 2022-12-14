/**
*  Guide: 
*   Press TAB to iterate through limbs to set goals.
*   Press 1-9 to choose specifically which limb to move (order determined by order of chain/goals added to chains/goals in setup)
*   Press SPACE to toggle between crawling and manually setting limb goals.
*   
*/

import java.util.ArrayList;
import java.util.*;

final int width = 640;
final int height = 480;
final float topArmYOffset = 20;
final float botArmYOffset = 150;
final float rightArmXOffset = 60;

final float upperArmLength = 60; //60
final float lowerArmLength = 90; //90
final float handLength = 30; //30

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
ArrayList<Vector2> goals = new ArrayList<Vector2>(); //each element corresponds to a chain.
int chainChoice = 0; // 0: LT, 1: RT, 2: LB, 3: RB

//crawling variables
boolean crawling = false; //if set to true, the roots will automatically shift based on reach.
float crawlTolerance = 60f;//distance before goal is set to new root.
Vector2 lastDirection; //Keeps track of movement to determine where to reposition goals (NOT User defined)
float goalRandomization = (float) Math.PI/2;//when relocating goals for crawling, goal is deviated slightly.

void setup() {
  size(1289, 720); 
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  
  //left top
  ArrayList<Float> lengthLT = new ArrayList<Float>();
  ArrayList<Float> rotateLT = new ArrayList<Float>();
  lengthLT.add(upperArmLength);
  lengthLT.add(lowerArmLength);
  lengthLT.add(handLength);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  rotateLT.add(rotateI);
  Chain chainLT = new CCD(lengthLT, rotateLT, rootLT, "LT");
  chainLT.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainLT);
  Vector2 leftGoalTop = new Vector2(0, rootM.y + topArmYOffset);
  goals.add(leftGoalTop);
  
  //right top
  ArrayList<Float> lengthRT = new ArrayList<Float>();
  ArrayList<Float> rotateRT = new ArrayList<Float>();
  lengthRT.add(upperArmLength);
  lengthRT.add(lowerArmLength);
  lengthRT.add(handLength);
  rotateRT.add(rotateI);
  rotateRT.add(rotateI);
  rotateRT.add(rotateI);
  Chain chainRT = new FABRIK(lengthRT, rotateRT, rootRT, "RT");
  chainRT.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainRT);
  Vector2 rightGoalTop = new Vector2(width, rootM.y + topArmYOffset);
  goals.add(rightGoalTop);
  
  //left bottom
  ArrayList<Float> lengthLB = new ArrayList<Float>();
  ArrayList<Float> rotateLB = new ArrayList<Float>();
  lengthLB.add(upperArmLength);
  lengthLB.add(lowerArmLength);
  lengthLB.add(handLength);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  rotateLB.add(rotateI);
  Chain chainLB = new CCD(lengthLB, rotateLB, rootLB, "LB");
  chainLB.setJointLimit(2, (float)Math.PI/2);
  chainLB.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainLB);
  Vector2 leftGoalBot = new Vector2(rootM.x, height);
  goals.add(leftGoalBot);
  
  //right bottom
  ArrayList<Float> lengthRB = new ArrayList<Float>();
  ArrayList<Float> rotateRB = new ArrayList<Float>();
  lengthRB.add(upperArmLength);
  lengthRB.add(lowerArmLength);
  lengthRB.add(handLength);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  rotateRB.add(rotateI);
  Chain chainRB = new CCD(lengthRB, rotateRB, rootRB, "RB");
  chainRB.setJointLimit(2, (float)Math.PI/2);
  chainRB.setJointLimit(3, (float)Math.PI/2);
  chains.add(chainRB);
  Vector2 rightGoalBot = new Vector2(rootM.x + rightArmXOffset, height);
  goals.add(rightGoalBot);

  //for (Vector2 goal : goals) println(goal);
}

void updateChainRoots() {
  chains.get(0).setRoot(rootLT);
  chains.get(1).setRoot(rootRT);
  chains.get(2).setRoot(rootLB);
  chains.get(3).setRoot(rootRB);
  //chains.get(0).setRoot(new Vector2(rootM.x, rootM.y + topArmYOffset)); //LT
  //chains.get(1).setRoot(new Vector2(rootM.x, rootM.y + botArmYOffset)); //LB
  //chains.get(2).setRoot(new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset)); //RT
  //chains.get(3).setRoot(new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset)); //RB
}

void setLeftRightRoot() {
  rootLT = new Vector2(rootM.x, rootM.y + topArmYOffset);
  rootLB = new Vector2(rootM.x, rootM.y + botArmYOffset);
  rootRT = new Vector2(rootM.x + rightArmXOffset, rootM.y + topArmYOffset);
  rootRB = new Vector2(rootM.x + rightArmXOffset, rootM.y + botArmYOffset);
  updateChainRoots();
}

void switchLinks() {
  goals.set(chainChoice, new Vector2(mouseX, mouseY));
  //println("Link for " + chains.get(chainChoice).name + ": " + goals.get(chainChoice)); //DEBUG
}

void crawlingBehavior() {
  for (int i = 0; i < chains.size(); i++){
    Chain curChain = chains.get(i);
    Vector2 endEffectorPosition = curChain.startPos.get(curChain.startPos.size() - 1);
    if (endEffectorPosition.distanceTo(goals.get(i)) > crawlTolerance){ //the goal of this chain is too far for the end effector. Reposition goal.
      Vector2 chainRoot = curChain.startPos.get(0);
      if (lastDirection.length() != 0){ //If movement caused the breakaway.
        Vector2 deviated = rotateRandom((float) Math.PI/2, lastDirection);
        goals.set(i, chainRoot.plus(deviated.times(curChain.getTotalLength() - crawlTolerance/2)));
      } else { //no movement caused the brakeaway (likely in the beginning.)
        goals.set(i, chainRoot.plus(new Vector2(random(-1,1), random(-1,1)).normalized().times(curChain.getTotalLength() - crawlTolerance/2)));
      }
    }
  }
}

//randomly rotates the given Vector within the range (range/2 of either side in radians)
private Vector2 rotateRandom(float range, Vector2 input){
  float angle = random(-range/2, range/2);
  float newX = input.x * cos(angle) - input.y * sin(angle);
  float newY = input.x * sin(angle) + input.y * cos(angle);
  return new Vector2(newX, newY);
}

void draw() {
  if (crawling){
    crawlingBehavior();
  } else {
    switchLinks();
  }
  chains.get(0).solve(goals.get(0));
  chains.get(1).solve(goals.get(1));
  chains.get(2).solve(goals.get(2));
  chains.get(3).solve(goals.get(3));
  
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
    for(int i=0; i<l.getNumLinks(); i++){
      pushMatrix();
      translate(l.getPos(i).x, l.getPos(i).y);
      rotate(l.getRotate(i+1));
      circle(0,0,10);
      rect(0, -armW / 3, l.getLength(i), armW);
      if (i + 1 >= l.numLinks){
        popMatrix();
        pushMatrix();
        translate(l.startPos.get(i+1).x, l.startPos.get(i+1).y);
        circle(0,0,10);
      }
      popMatrix();
    }
  }
  
  //goals
  for(Vector2 goal : goals) {
    pushMatrix();
    translate(goal.x, goal.y);
    fill(color(100,100,100));
    circle(0,0,30);
    popMatrix();
  }
}
float obstacleSpeed = 300;

void handleArrowKeys(float dt) {
  Vector2 obstacleDir = new Vector2(0,0);
  if (leftPressed) obstacleDir.add(new Vector2(-1, 0));
  if (rightPressed) obstacleDir.add(new Vector2(1, 0));
  if (downPressed) obstacleDir.add(new Vector2(0,1));
  if (upPressed) obstacleDir.add(new Vector2(0,-1));
  if (obstacleDir.length() != 0) obstacleDir.normalize();
  
  Vector2 obstacleVel = obstacleDir.times(obstacleSpeed);
  
  rootM.x += obstacleVel.x * dt;
  rootM.y += obstacleVel.y * dt;
  setLeftRightRoot();
  lastDirection = obstacleDir; //for crawling logic
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
  if (keyCode == 32) { //space.
    crawling = !crawling; //toggle crawling.
  }
  if (keyCode >= 49 && keyCode <= 57) { //1-9. Select limb manually.
    int num = keyCode - 49; //-49 because zero indexing
    if (num < chains.size()){
      chainChoice = num;
      println("Moving chain: " + chains.get(chainChoice).name);
    } else {
      println("This limb does not exist!");
    }
  }
}

void keyReleased() {
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false;
  if (keyCode == DOWN) downPressed = false;
}
