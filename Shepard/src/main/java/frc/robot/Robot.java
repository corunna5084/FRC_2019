/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends IterativeRobot {

  // X52
  Joystick sticky = new Joystick(0);
  double axisX;// axis 1
  double axisY;// axis 2
  double axisZ;// axis 3
  double rotX; // axis 4
  double rotY; // axis 5
  double rotZ; // axis 6
  double slider;// axis 7
  int InAnalogInput = 1;
  int Fire = 2;
  int A = 3;
  int B = 4;
  int C = 5;
  int D = 7;
  int E = 8;
  int T1 = 9;
  int T2 = 10;
  int T3 = 11;
  int T4 = 12;
  int T5 = 13;
  int T6 = 14;
  int povUp = 16;
  int povRight = 17;
  int povDown = 18;
  int povLeft = 19;
  int thumbUp = 20;
  int thumbRight = 21;
  int thumbDown = 22;
  int thumbLeft = 23;
  int ModeG = 24;
  int ModeO = 25;
  int ModeR = 26;
  int i = 30;
  int button = 31;
  int scroll = 32;

  //logictech joystick
  Joystick logic = new Joystick(1);
  int lJoyX = 0;
  int lJoyY = 1;
  int lTrigger = 2;
  int rTrigger = 3;
  int rJoyX = 4;
  int rJoyY = 5;
  int a = 1;
  int b = 2;
  int x = 3;
  int y = 4;
  int lb = 5;
  int rb = 6;
  int back = 7;
  int start = 8;
  int lJoy = 9;
  int rJoy = 10;

  //the X52 override varibles over the Logictech
  boolean x52JackOverride;
  boolean x52ElevOverride;
  boolean x52HatchOverride;

  //These are the CAN Network Talons
  WPI_TalonSRX FR = new WPI_TalonSRX(0);
  WPI_TalonSRX FL = new WPI_TalonSRX(5);
  WPI_TalonSRX RR = new WPI_TalonSRX(1);
  WPI_TalonSRX RL = new WPI_TalonSRX(4);
  WPI_TalonSRX Jack = new WPI_TalonSRX(7);
  WPI_TalonSRX outBall = new WPI_TalonSRX(2);

  Victor Hatch = new Victor(9);

  // this is for the CAN CANSparkMax
  CANSparkMax inLift = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax inRun = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax Elevator = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

  //Spark Max integrated Encoders
  CANEncoder encoderIntake = new CANEncoder(inLift);
  CANEncoder encoderElevator = new CANEncoder(Elevator);

  //Mecanum Drive and Motor Safety
  MecanumDrive mecanum;
  boolean enabled = false;

  //Limit Switches as analog inputs because god knows why
  double upper = 3.5;
  AnalogInput upElev = new AnalogInput(0);
  AnalogInput lowElev = new AnalogInput(1);
  AnalogInput upIntake = new AnalogInput(2);
  AnalogInput lowIntake = new AnalogInput(3);
  boolean upLimitElev;
  boolean lowLimitElev;
  boolean upLimitIntake;
  boolean lowLimitIntake;

  //Magnetic Encoder for the Talon
  double elevatorPosition;
  double intakePosition;
  boolean encode;

  //Cameras
  UsbCamera cam1;
  UsbCamera cam2;

  //Navx Setup
  // AHRS ahrs;
  // boolean conn = true;

  //Arduino Setup
  SerialPort arduino;
  int communication = 115200;
  String message = "-1";
  boolean dist = false;
  int yay = 0;
  boolean connected = true; 

  int thing = 0;
  boolean ballPos;
  @Override
  public void robotInit() {
    //Connects to the Navx on the I2C Port
    // try{
    //   ahrs = new AHRS(I2C.Port.kOnboard);
    //   ahrs.enableLogging(true);
    // }catch(RuntimeException ex){
    //   System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
    //   conn = false;
    // }

    //Connects to the Arduino via USB
    try{
      arduino = new SerialPort(communication, SerialPort.Port.kUSB);
      System.out.println("Connected to Arduino");
    }catch (Exception e){
      System.out.println("Trying USB 1");
      try{
        arduino = new SerialPort(communication, SerialPort.Port.kUSB1);
        System.out.println("Connected to Arduino on USB1");
      }catch (Exception e1){
        System.out.println("Trying USB 2");
        try{
          arduino = new SerialPort(communication, SerialPort.Port.kUSB2);
          System.out.println("Connected to Arduino on USB2");
        }catch (Exception e2){
          System.out.println("Failed to Connect to Arduino, tried all available Ports");
          connected = false;
        }
      }
    }

    //Camera
    cam1 = CameraServer.getInstance().startAutomaticCapture(0);
    cam2 = CameraServer.getInstance().startAutomaticCapture(1);

    //Drive and Motor Setup
    mecanum = new MecanumDrive(FL, RL, FR, RR);
    mecanum.setSafetyEnabled(enabled);
    RR.setSafetyEnabled(enabled);
    RL.setSafetyEnabled(enabled);
    FL.setSafetyEnabled(enabled);
    FR.setSafetyEnabled(enabled);
    Jack.setSafetyEnabled(enabled);
    outBall.setSafetyEnabled(enabled);

    //Encoder Setup
    encoderElevator.setPosition(0.0);
    encoderIntake.setPosition(0.0);
    Elevator.setOpenLoopRampRate(0);
    inRun.setOpenLoopRampRate(0);
    inLift.setOpenLoopRampRate(0);
    inRun.setIdleMode(IdleMode.kCoast);
    inLift.setIdleMode(IdleMode.kCoast);
 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Use this for things that need to be constantly running,
    // i.e compressor
    smartDashboard();
    LimitSwitch();

    intakePosition = encoderIntake.getPosition();
    elevatorPosition = encoderElevator.getPosition();


  }

  public void smartDashboard() {
    // SmartDashboard Outputs
    // SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    // SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    // SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
    // SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    // SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    // SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    // SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    // SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    // SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    // SmartDashboard.putString("VOTES", message);
    // SmartDashboard.putBoolean("Distance Bool", dist);
    SmartDashboard.putNumber("Talon Current", Jack.getOutputCurrent());
    SmartDashboard.putBoolean("Upper Elevator",   upLimitElev);
    SmartDashboard.putBoolean("Lower Elevator",   lowLimitElev);
    SmartDashboard.putBoolean("Upper Intake",     upLimitIntake);
    SmartDashboard.putBoolean("Lower Intake",     lowLimitIntake);
    SmartDashboard.putNumber("Elevator Position", elevatorPosition);
    SmartDashboard.putNumber("Intake Lift Position", intakePosition);

    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    // Driver Station Outputs
  }

  // I do not think we need Autonomus code
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  public void autoPlatform(){

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    axisX = Math.pow(sticky.getRawAxis(0), 3);
    axisY = -1 * Math.pow(sticky.getRawAxis(1), 3);
    rotZ = sticky.getRawAxis(5) * 0.75;
    mecanum.driveCartesian(axisX, axisY, rotZ);


    // Subsystems
    Intake();
    elevator();
    intakeLift();
    ball();
    jack();
    hatch();
    logictechController();

    //Sensors
    if(connected){
      LIDAR();
    }
    // if(conn){
    //   NavX();
    // }

    Timer.delay(0.001);
  }

  // Sensors Start
  // public void NavX() {
        
  //   while(stillUp && isEnabled()) {
  //       double currentAngle = gyro.getPitch();
        
  //       if(abs(currentAngle - targetAngle) > abs(thresholdAngle)) {
  //         backLift.setMotor(Math.sin(currentAngle * Math.PI/180.0)); 
  //         // Might be necessary to multiply by -1 
  //         // Or multiplied by a constant, or adding a constant to make up for the robot's weight
  //       }
  //       else {
  //           backLift.setMotor(0);
  //       }
  //   }
    
  //   intakeLift.setMotor(0);
  //   backLift.setMotor(0);
    
  // }

  public void LIDAR() {
    message = arduino.readString();
    if (!message.isBlank() && !message.isEmpty()){
      if(message.charAt(0) == 'y'){
        dist = true;
        yay ++;
      }else{
        dist = false;
        yay --;
      }
    }else{
      System.out.println("No Message");
    }
  }

  public void LimitSwitch() {
    if (upElev.getAverageVoltage() < upper) {
      upLimitElev = false;
    } else {
      upLimitElev = true;
    }
    if (lowElev.getAverageVoltage() < upper) {
      lowLimitElev = false;
    } else {
      lowLimitElev = true;
    }
    if (upIntake.getAverageVoltage() < upper) {
      upLimitIntake = false;
      encoderIntake.setPosition(0.0);
    } else {
      upLimitIntake = true;
    }
    if (lowIntake.getAverageVoltage() < upper) {
      lowLimitIntake = false;
    } else {
      lowLimitIntake = true;
    }
  }
  // Sensors End

  // ------------------------------------------------------

  // Subsystem Begins
  public void logictechController() {
    if (logic.getRawAxis(rJoyY) < 0 && !x52ElevOverride && !upLimitElev) {
      Elevator.set(-1*logic.getRawAxis(rJoyY));
    } else if (logic.getRawAxis(rJoyY) > 0 && !x52ElevOverride && !lowLimitElev) {
      Elevator.set(-1*logic.getRawAxis(rJoyY));
    }
    
    if (!x52JackOverride) {
      // power the Jack up device down
      if (logic.getRawButton(a) && !x52ElevOverride) {
        Jack.set(ControlMode.PercentOutput, 0.65);
      } else if (logic.getRawButton(x) && !x52ElevOverride) {
        Jack.set(ControlMode.PercentOutput, -0.8);
      }
    }
	
	if(logic.getRawButton(lb) && !x52HatchOverride){
      Hatch.set(0.5);//closed
    }else if(logic.getRawButton(rb) && !x52HatchOverride){
      Hatch.set(-1);//open
    }
  }

  public void hatch(){
    if(sticky.getRawButton(povUp)){
      x52HatchOverride = true;
        Hatch.set(0.5);//closed
      }else if(sticky.getRawButton(povDown)){
      x52HatchOverride = true;
        Hatch.set(-1);//open
      }else{
      x52HatchOverride = false;
        Hatch.set(0.0);
      }
    }

  public void elevator() { // Fuction relating to the elevator

    if (sticky.getRawButton(thumbUp) && !upLimitElev) {
      Elevator.set(0.75);
      x52ElevOverride = true;
    } else if (sticky.getRawButton(thumbDown) && !lowLimitElev) {
      Elevator.set(-0.75);
      x52ElevOverride = true;
    } else {
      Elevator.set(0.0);
      x52ElevOverride = false;
    }
  }

  public void intakeLift() { // This is the function for the lift mechanism for the intake
    double ball = 149;
    
    //Needed to be reversed becauser idiots
    if (sticky.getRawButton(B) && !upLimitIntake) {
      inLift.set(1);
      thing = 0;
    } else if (sticky.getRawButton(A) && !lowLimitIntake) {
      inLift.set(-1);
      thing = 0;
    } else {
      inLift.set(0.0);
    }

    if(sticky.getRawButton(button)  && thing == 0){
      thing = 1;
    }else if(!sticky.getRawButton(button)  && thing == 1){
      thing = 2;
    }else if(sticky.getRawButton(button)  && thing == 2){
      thing = 3;
    }else if(!sticky.getRawButton(button) && thing == 3){
      thing = 0;
    }
    if(thing == 1 || thing == 2){
      
      
      //inLift.set( Speed(intakePosition, ball, 25, 1) );
    }
    if(intakePosition < ball-3){
      ballPos =false;
    }else if(intakePosition > ball+3){
      ballPos = false;
    }else{
      ballPos = true;
    }
    //System.out.println(thing);
    SmartDashboard.putBoolean("Intake Position", ballPos);
  }

  public void ball() { // this is the fuction for the ball shooter on the elevator

    if (sticky.getRawButton(Fire)) {
      outBall.set(ControlMode.PercentOutput, 1);
    } else if (sticky.getRawButton(D)) {
      outBall.set(ControlMode.PercentOutput, -0.5);
    } else {
      outBall.set(ControlMode.PercentOutput, 0.0);
    }

  }

  public void jack() { // this is the fuction for that thing on the back of the robot
    // that helps it get up the platform
    if (sticky.getRawButton(E)) {
      // power the Jack up device down
      Jack.set(ControlMode.PercentOutput, 0.75);
      x52JackOverride = true;
    } else if (sticky.getRawButton(i)) {
      // power the Jack up device up
      Jack.set(ControlMode.PercentOutput, -0.75);
      x52JackOverride = true;
    } else {
      Jack.set(ControlMode.PercentOutput, 0.0);
      x52JackOverride = false;
    }

  }

  public void Intake() { // the ball intake mechanism just the wheel part

    if (sticky.getRawButton(InAnalogInput)) {
      inRun.set(0.65);
    } else if (sticky.getRawButton(C)) {
      inRun.set(-0.5);
    } else {
      inRun.set(0.0);
    }
  }

  public double Speed(double current, double desired, double gap, double max){
    /*variable deffinitions:
    *current; the changing value for the function, like x
    *desired; set value that we want to move to
    *gap; how long you want to have a slow down, might have a relation to distance 
    */
    
    double distance = current - desired;
    int direction;
    double speed;
    if(distance < 0){
      direction = -1;
    }else{
      direction = 1;
    }
    if(max > 1 && max < 0){
      speed = 0.0;
    }else{
      double finPos = desired -( direction * (gap));
      speed = (direction * max)/(1+Math.pow(Math.E, (direction*(10/gap)*(current-finPos))));
    }

    return speed;
  }
  // Subsystem Ends

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    System.out.println("YOU ARE IN TEST MODE!!!");
    System.out.println("WHY ARE YOU IN TEST MODE???");
    System.out.println("WE NEVER USED THIS MODE BEFORE!!!" + '\n');
  }

}
