/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  //These are the CAN Network Talons
  WPI_TalonSRX FR = new WPI_TalonSRX(0);
  WPI_TalonSRX FL = new WPI_TalonSRX(5);
  WPI_TalonSRX RR = new WPI_TalonSRX(1);
  WPI_TalonSRX RL = new WPI_TalonSRX(4);
  WPI_TalonSRX Jack = new WPI_TalonSRX(7);
  WPI_TalonSRX outBall = new WPI_TalonSRX(2);

  // this is for the CAN CANSparkMax
  CANSparkMax inLift = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax inRun = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax Elevator = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANEncoder encoderIntake = new CANEncoder(inLift);
  CANEncoder encoderElevator = new CANEncoder(Elevator);

  //X52
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

  //Now i do understand tthat i used the axis variables differently between the joystick

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

  //Cameras
  UsbCamera cam1;
  UsbCamera cam2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // This is for the camera
    
      cam1 = CameraServer.getInstance().startAutomaticCapture(0);

      cam2 = CameraServer.getInstance().startAutomaticCapture(1);

    mecanum = new MecanumDrive(FL, RL, FR, RR);

    mecanum.setSafetyEnabled(enabled);
    RR.setSafetyEnabled(enabled);
    RL.setSafetyEnabled(enabled);
    FL.setSafetyEnabled(enabled);
    FR.setSafetyEnabled(enabled);
    Jack.setSafetyEnabled(enabled);
    outBall.setSafetyEnabled(enabled);

    encoderElevator.setPosition(0.0);
    encoderIntake.setPosition(0.0);

    Elevator.setOpenLoopRampRate(2);
    inRun.setOpenLoopRampRate(1);

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
    elevatorPosition = encoderElevator.getPosition();
    intakePosition = encoderIntake.getPosition();

  }

  public void smartDashboard() {
    // SmartDashboard Outputs
    SmartDashboard.putBoolean("Upper Elevator Limit Switch Activated", upLimitElev);
    SmartDashboard.putBoolean("Lower Elevator Limit Switch Activated", lowLimitElev);
    SmartDashboard.putBoolean("Upper Intake Lift Limit Switch Activated", upLimitIntake);
    SmartDashboard.putBoolean("Lower Intake Lift Limit Switch Activated", lowLimitIntake);
    SmartDashboard.putNumber("Elevator Position", elevatorPosition);
    SmartDashboard.putNumber("Intake Lift Position", intakePosition);
    // Driver Station Outputs
    System.out.println("Limit switches for intake: Upper " + upLimitIntake + " | Lower " + lowLimitIntake);
    System.out.println("Limit switches for elevator: Upper " + upLimitElev + " | Lower" + lowLimitElev);
       
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
    logictechController();

    System.out.println("");

    Timer.delay(0.001);
  }

  // Sensors Start
  public void NavX() {
  }

  public void PixyCam() {
  }

  public void LIDAR() {
  }

  public void USBcam() {
    // Put USB Camera periodic settings here

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
        Jack.set(ControlMode.PercentOutput, 1.0);
      } else if (logic.getRawButton(x) && !x52ElevOverride) {
        Jack.set(ControlMode.PercentOutput, -1.0);
      }

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

    //Needed to be reversed becauser idiots
    if (sticky.getRawButton(A) && !upLimitIntake) {
      inLift.set(1);
    } else if (sticky.getRawButton(B) && !lowLimitIntake) {
      inLift.set(-1);
    } else {
      inLift.set(0.0);
    }
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
      Jack.set(ControlMode.PercentOutput, 1);
      x52JackOverride = true;
    } else if (sticky.getRawButton(i)) {
      // power the Jack up device up
      Jack.set(ControlMode.PercentOutput, -1);
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
