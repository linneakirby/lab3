/**
 **********************************************************************************************************************
 * @file       lab3.pde
 * @author     Linnea Kirby, Sri Gannavarapu
 * @version    V4.1.0
 * @date       12-March-2021
 * @brief      based off of "sketch_4_Wall_Physics" by Steve Ding, Colin Gallacher
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import java.util.*;
import controlP5.*;
/* end library imports *************************************************************************************************/

/* user-set parameters ***********/
public final String PORT = "/dev/cu.usbmodem14201";
public final boolean DEBUG = false;
public final boolean DEBUGMISLEAD = false;
public final boolean DEBUGARROGANT = false;
/* end user-set parameters **********/

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0);

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;
float             worldHeight                         = 10.0;

float             edgeTopLeftX                        = 0.0;
float             edgeTopLeftY                        = 0.0;
float             edgeBottomRightX                    = worldWidth;
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox[] wall = new FBox[3];
FBox region;
FCircle[] bubbles = new FCircle[10];
FBlob arrogance;
FCircle circle;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

ControlP5 cp5;
Boolean mislead = false;
Boolean inflate = false;
Boolean leth = false;
Boolean sharp = false;
float[][] positionArr = new float[2][2];

float circleSize = 1;
float blobSize = 2;
float speed =0;
/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1000, 400);

  /* device setup */

  /**
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem14201", 0);
   */
  haplyBoard = new Board(this, PORT, 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);


  widgetOne.device_set_parameters();


  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this);
  hAPI_Fisica.setScale(pixelsPerCentimeter);
  world               = new FWorld();


  float[] xyArr = new float[2];
  float x = edgeTopLeftX+worldWidth/2;
  float y = edgeTopLeftY+2;
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1));
  s.h_avatar.setDensity(4);
  s.init(world, x, y);

  positionArr[0] = new float[] { x, y };
  positionArr[1] = new float[] {x, y};


  /* If you are developing on a Mac users must update the path below
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png"
   */
  haplyAvatar = loadImage("./img/Haply_avatar.png");
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar);


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY));
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);

  cp5 = new ControlP5(this);
  cp5.addButton("mislead").setLabel("First Word").setPosition(40, 40);
  cp5.addButton("inflate").setLabel("Second Word").setPosition(40, 60);
  cp5.addButton("lethargy").setLabel("Third Word").setPosition(40, 80);
  //cp5.addButton("sharp").setLabel("Fourth Word").setPosition(40, 100);


  world.draw();


  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

float lastMillis = millis();
float deltaTime = 0f;
float growDelta = 0f;
ArrayList<FBody> isTouching;

/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  deltaTime = millis() - lastMillis;
  if (millis() - lastMillis >= 100) {
    positionArr = checkPosition(positionArr);
    if (DEBUGMISLEAD) {
      System.out.println("Position Array: {" + positionArr[0][0] + ", " + positionArr[0][1] + "}, {"
        + positionArr[1][0] + ", " + positionArr[1][1] + "}");
      System.out.println("Mislead? " + mislead);
    }
    lastMillis = millis();
  }



  if (renderingForce == false) {
    background(255);
    if (mislead) {
      if (checkPassThroughWall(positionArr)) {
        wall[2].setSensor(true);
        if(DEBUG){
          wall[2].setFillColor(color(0, 255, 0));
        }
      } else {
        wall[2].setSensor(false);
        if(DEBUG){
          wall[2].setFillColor(color(0, 0, 0));
        }
      }
    }
    if (inflate) {
      setBlobSize(arrogance, blobSize);
      circle.setSize(circleSize);
      delay(50);
      speed = Math.abs(s.h_avatar.getVelocityX());
      if (speed >10 && circleSize<20) {
        circleSize += deltaTime*0.01 + speed*0.005;
        blobSize += deltaTime*0.01 + speed*0.01;
        growDelta = 0f;
        if (DEBUGARROGANT) {
          print("grow");
        }
      } else {
        if (growDelta > 50 && circleSize > 1) {
          circleSize -= deltaTime*0.0005;
          blobSize -= deltaTime*0.0005;
        }
        growDelta += deltaTime;
      }
    }

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    hapticSimulationStep();
  }
}
/* end simulation section **********************************************************************************************/

/* helper functions section, place helper functions here ***************************************************************/

void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/mislead":
      if (DEBUG) {
        println("Button First Word Pressed");
      }
      mislead = true;
      clearInflate();
      clearLeth();
      clearSharp();
      beginMislead();

      break;
    case "/inflate":
      if (DEBUG) {
        println("Button Second Word Pressed");
      }
      inflate = true;
      clearMislead();
      clearLeth();
      clearSharp();
      beginInflate();
      break;
    case "/lethargy":
      if (DEBUG) {
        println("Button Third Word Pressed");
      }
      leth  = true;
      clearMislead();
      clearInflate();
      clearSharp();
      beginLeth();
      break;
    case "/sharp":
      if (DEBUG) {
        println("Button Fourth Word Pressed");
      }
      sharp = true;
      clearMislead();
      clearInflate();
      clearLeth();
      beginSharp();
      break;
    }
  }
}

private void setBlobSize(FBlob a, float s) {
  world.remove(arrogance);
  createBlob(a, s);
}

private Boolean checkPassThroughWall(float[][] positionArr) {
  // if moved from left to right
  if (positionArr[1][0] - positionArr[0][0] < 0f) {
    return true;
  }
  return false;
}

private float[][] checkPosition(float[][] positionArr) {
  //AvatarPosition or ToolPosition?
  float x = s.getAvatarPositionX();
  float y = s.getAvatarPositionY();

  //replace previous position with current position
  positionArr[0][0] = positionArr[1][0];
  positionArr[0][1] = positionArr[1][1];

  //update current position
  positionArr[1][0] = x;
  positionArr[1][1] = y;

  return positionArr;
}


void beginMislead() {
  createWall();
}

void clearMislead() {
  mislead = false;
  for(FBox w : wall){
    world.remove(w);
  }
}

void beginInflate() {
  createBlob(arrogance, 2f);
  createCircle();
  s.h_avatar.setDamping(800);
}

void clearInflate() {
  inflate = false;
  world.remove(circle);
  world.remove(arrogance);
  s.h_avatar.setDamping(40);
}

void beginLeth() {

  createRegion();
  s.h_avatar.setDamping(800);
}

void clearLeth() {
  leth = false;
  world.remove(region);
  s.h_avatar.setDamping(40);
}

void beginSharp() {
  createBubbles();
  s.h_avatar.setDamping(400);
}

void clearSharp() {
  sharp = false;
  for (FCircle c : bubbles) {
    world.remove(c);
  }
  s.h_avatar.setDamping(40);
}

void createWall() {
  /* creation of wall */
  wall[0]                   = new FBox(33, 0.1);   //width, 0.1
  wall[0].setPosition(edgeTopLeftX, edgeTopLeftY+2*worldHeight/3.0-2);
  wall[0].setStatic(true);
  if(DEBUG){
    wall[0].setFill(0);
  }
  else{
    wall[0].setFill(0, 0);
    wall[0].setNoStroke();
  }
  world.add(wall[0]);

  wall[1]                   = new FBox(7, 0.1);
  wall[1].setPosition(edgeTopLeftX+22, edgeTopLeftY+2*worldHeight/3.0-2);
  wall[1].setStatic(true);
  if(DEBUG){
    wall[1].setFill(0, 0, 0);
  }
  else{
    wall[1].setFill(0, 0);
    wall[1].setNoStroke();
  }
  world.add(wall[1]);

  wall[2]                   = new FBox(2, 0.1);
  wall[2].setPosition(edgeTopLeftX+17.5, edgeTopLeftY+2*worldHeight/3.0-2);
  wall[2].setStatic(true);
  if(DEBUG){
    wall[2].setFill(0,0,0);
  }
  else{
    wall[2].setFill(0,0);
    wall[2].setNoStroke();
  }
  world.add(wall[2]);
  positionArr = checkPosition(positionArr);
}

void createBlob(FBlob a, float s) {
  if (a != null) {
    world.remove(a);
  }
  a = new FBlob();
  a.setAsCircle(15, 5, s);
  if (DEBUG) {
    a.setFill(0, 255, 0, 50);
  } else {
    a.setFill(0, 0);
    a.setNoStroke();
  }
  a.setStatic(true);
  a.setSensor(true);
  arrogance = a;
  world.add(arrogance);
}

void createCircle() {
  if (circle != null) {
    world.remove(circle);
  }
  circle = new FCircle(1);
  circle.setPosition(15, 5);
  if (DEBUG) {
    circle.setFill(random(0, 255), random(0, 255), random(0, 255), 50);
  } else {
    circle.setFill(0, 0);
    circle.setNoStroke();
  }
  circle.setStatic(true);
  world.add(circle);
}

void createRegion() {
  region = new FBox(8, 8);
  region.setPosition(15, 5);
  region.setStatic(true);
  if(DEBUG){
    region.setFill(random(0, 255), random(0, 255), random(0, 255), 50);
  }
  else{
    region.setFill(0, 0);
    region.setNoStroke();
  }
  region.setSensor(true);
  world.add(region);
}

void createBubbles() {
  float x, y;
  for (int i = 0; i<10; i++) {
    bubbles[i] = new FCircle(0.5);
    HashSet xSet = new HashSet();
    HashSet ySet = new HashSet();
    x = random(10, 23);
    y = random(3, 8);
    while (xSet.contains(x)) {
      x = random(10, 23);
    }
    xSet.add(x);
    while (ySet.contains(y)) {
      y = random(3, 8);
    }
    ySet.add(y);
    bubbles[i].setPosition(x, y);
    if (DEBUG) {
      bubbles[i].setFill(random(0, 255), random(0, 255), random(0, 255));
    } else {
      bubbles[i].setFill(0, 0);
    }
    bubbles[i].setNoStroke();
    bubbles[i].setStatic(true);
    bubbles[i].setSensor(true);
    world.add(bubbles[i]);
  }
}
void hapticSimulationStep() {
  /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

  renderingForce = true;

  if (haplyBoard.data_available()) {
    getEndEffectorState();
  }

  s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7);


  s.updateCouplingForce();
  fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
  fEE.div(100000); //dynes to newtons

  torques.set(widgetOne.set_device_torques(fEE.array()));
  widgetOne.device_write_torques();

  world.step(1.0f/1000.0f);

  renderingForce = false;
}

void getEndEffectorState() {
  /* GET END-EFFECTOR STATE (TASK SPACE) */
  widgetOne.device_read_data();

  angles.set(widgetOne.get_device_angles());
  posEE.set(widgetOne.get_device_position(angles.array()));
  posEE.set(posEE.copy().mult(200));
}




/* end helper functions section ****************************************************************************************/
