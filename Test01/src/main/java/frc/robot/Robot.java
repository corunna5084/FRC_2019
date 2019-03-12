/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  private static final int TimeOut = 10;

  WPI_TalonSRX FR = new WPI_TalonSRX(0);
  WPI_TalonSRX FL = new WPI_TalonSRX(5);
  WPI_TalonSRX RR = new WPI_TalonSRX(1);
  WPI_TalonSRX RL = new WPI_TalonSRX(4);
  WPI_TalonSRX Elevator = new WPI_TalonSRX(6);
  WPI_TalonSRX Jack = new WPI_TalonSRX(7);
  WPI_TalonSRX inRun = new WPI_TalonSRX(3);
  WPI_TalonSRX outBall = new WPI_TalonSRX(2);

  CANSparkMax inLift = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANEncoder liftPosition = new CANEncoder(inLift);

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
  int lJoyX = 0;//?
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

  boolean x52Override;
  boolean logicOverride;

  MecanumDrive mecanum;
  boolean enabled = false;
  /*
   * SerialPort arduino; String message[] = new String[4]; double x = 0; double
   * angle = 0;
   int communication = 115200;*/
  
  int elevatorPosition;
  double inLiftPos;
  double position;

  boolean encodeLift;
  boolean encodeElev;

  double upper = 3.5;

  AnalogInput upElev = new AnalogInput(0);
  AnalogInput lowElev = new AnalogInput(1);
  AnalogInput upIntake = new AnalogInput(2);
  AnalogInput lowIntake = new AnalogInput(3);

  boolean upLimitElev;
  boolean lowLimitElev;
  boolean upLimitIntake;
  boolean lowLimitIntake;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // This is for the camera
    UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(0);
    UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture(1);

    mecanum = new MecanumDrive(FL, RL, FR, RR);

    /*
     * try { arduino = new SerialPort(communication, SerialPort.Port.kUSB);
     * System.out.println("Connected to Arduino"); } catch (Exception e) {
     * System.out.println("Failed to Connect to Arduino, Trying USB1");
     * 
     * try { arduino = new SerialPort(communication, SerialPort.Port.kUSB1);
     * System.out.println("Connected to Arduino"); } catch (Exception e1) {
     * System.out.println("Failed to Connect to Arduino, Trying USB2");
     * 
     * try { arduino = new SerialPort(communication, SerialPort.Port.kUSB2);
     * System.out.println("Connected to Arduino"); } catch (Exception e2) {
     * System.out.println("Failed to Connect to Arduino, No More Connections to Try"
     * ); } } }
     */
    mecanum.setSafetyEnabled(enabled);
    RR.setSafetyEnabled(enabled);
    RL.setSafetyEnabled(enabled);
    FL.setSafetyEnabled(enabled);
    FR.setSafetyEnabled(enabled);
    Elevator.setSafetyEnabled(enabled);
    Jack.setSafetyEnabled(enabled);
    outBall.setSafetyEnabled(enabled);
    inRun.setSafetyEnabled(enabled);

    Elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Elevator.setSelectedSensorPosition(0);

    encodeElev = false;
    encodeLift = false;

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
    // values for the encoder
    elevatorPosition = Elevator.getSelectedSensorPosition();
    inLiftPos = liftPosition.getPosition();

  }

  public void smartDashboard() {
    
    // SmartDashboard.putNumber("X Position", x);
    // SmartDashboard.putNumber("Angular Position", angle);
    SmartDashboard.putNumber("Lift Currennt Position", inLiftPos);
    SmartDashboard.putNumber("Percentage of the Intake lift Postion", position * 100);
    SmartDashboard.putBoolean("Using Encoder on Intake Lift", encodeLift);
    SmartDashboard.putBoolean("Using Encoder on Elevator", encodeElev);
    SmartDashboard.putNumber("Elevator Position", elevatorPosition);
    SmartDashboard.putBoolean("Upper Elevator Limit Switch Activated", upLimitElev);
    SmartDashboard.putBoolean("Lower Elevator Limit Switch Activated", lowLimitElev);
    SmartDashboard.putBoolean("Upper Intake Lift Limit Switch Activated", upLimitIntake);
    SmartDashboard.putBoolean("Lower Intake Lift Limit Switch Activated", lowLimitIntake);

    if(upElev.getAverageVoltage() < upper){
      upLimitElev = false;
    }else{
      upLimitElev = true;
    }
    if(lowElev.getAverageVoltage() < upper){
      lowLimitElev = false;
    }else{
      lowLimitElev = true;
    }
    if(upIntake.getAverageVoltage() < upper){
      upLimitIntake = false;
    }else{
      upLimitIntake = true;
    }
    if(lowIntake.getAverageVoltage() < upper){
      lowLimitIntake = false;
    }else{
      lowLimitIntake = true;
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */

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

    System.out.println(
        "Limit switches for intake up/low: " + upIntake.getAverageVoltage() + "/" + lowIntake.getAverageVoltage());
    System.out.println(
        "Limit switches for elevator up/low: " + upElev.getAverageVoltage() + "/" + lowElev.getAverageVoltage());

    // Subsystems
    Intake();
    elevator();
    intakeLift();
    ball();
    jack();
    EncodeState();
    logictechController();

    // Sensors
    // PixyCam();
    System.out.println("");

    Timer.delay(0.001);
  }

  // Sensors Start
  public void NavX() {
  }

  public void PixyCam() {
    /*
     * use PixyCamera.ino for testing to see if the message sender and the below
     * function can work together to be able to make use of the X position value and
     * Angle of an object
     */

    /*
     * if (arduino.readString() != "-1") { message =
     * arduino.readString().split(","); } if (message.length != 1) { x =
     * Double.parseDouble(message[0]); angle = Double.parseDouble(message[1]); }
     */
  }

  public void LIDAR() {
  }
  // Sensors End

  // ------------------------------------------------------

  // Subsystem Begins
  public void logictechController(){
    if (logic.getRawButton(x) && !x52Override && upElev.getAverageVoltage() < upper) {
      Elevator.set(0.75);
      logicOverride = true;
    } else if (logic.getRawButton(a) && !x52Override && lowElev.getAverageVoltage() < upper) {
      Elevator.set(-0.75);
      logicOverride = true;
    } else {
      logicOverride = false;
    }

    if (!x52Override) {
      // power the Jack up device down
      Jack.set(ControlMode.PercentOutput, logic.getRawAxis(rJoyY));
    }
  }

  public void EncodeState() {
    if (sticky.getRawButton(T1)) {
      encodeLift = true;
    } else if (sticky.getRawButton(T2)) {
      encodeLift = false;
    }

    if (sticky.getRawButton(T3)) {
      encodeElev = true;
    } else if (sticky.getRawButton(T4)) {
      encodeElev = false;
    }
  }

  public void elevator() { // Fuction relating to the elevator
    // will need to change the values
    // 42 parts per revolution
    int bottomGoal = 0; // Bottom goal on the rocket
    int middleGoal = -43040; // Middle goal of the rocket
    int topGoal = -72352; // Top goal of the rocket
    if (!encodeElev) {
      System.out.println("Using Manual Movement For Elevator");
      if (sticky.getRawButton(thumbUp) && upElev.getAverageVoltage() < upper) {
        Elevator.set(0.75);
        x52Override = true;
      } else if (sticky.getRawButton(thumbDown) && lowElev.getAverageVoltage() < upper) {
        Elevator.set(-0.75);
        x52Override = true;
      } else {
        Elevator.set(0.0);
        x52Override = false;
      }
    }
    if (encodeElev && !logicOverride) {
      System.out.println("Using Encoder Movement For Elevator");
      if (sticky.getRawButton(ModeG)) {
        if (lowElev.getAverageVoltage() < upper) {
          if (elevatorPosition < bottomGoal - 10) {
            if (elevatorPosition < bottomGoal - 4095) {
              Elevator.set(-1);
            } else {
              Elevator.set(-1 + ((bottomGoal - (elevatorPosition - 4095)) / (4095*2)));
            }
          } else {
            Elevator.set(0.0);
          }
        } else {
          Elevator.set(0.0);
        }
      } else if (sticky.getRawButton(ModeO)) {
        if (elevatorPosition > middleGoal + 10) {
          if (elevatorPosition > middleGoal + 4095) {
            Elevator.set(0.5);
          } else {
            Elevator.set(0.5 - ((middleGoal - (elevatorPosition + 4095)) / (4095*2)));
          }
        } else if (elevatorPosition < middleGoal - 10) {

          if (elevatorPosition < middleGoal - 4095) {
            Elevator.set(-0.5);
          } else {
            Elevator.set(-0.5 + ((middleGoal - (elevatorPosition - 4095)) / (4095*2)));
          }
        } else {

          Elevator.set(0.0);

        }
      } else if (sticky.getRawButton(ModeR)) {
        if (upElev.getAverageVoltage() < upper) {
          if (elevatorPosition > topGoal + 10) {
            if (elevatorPosition > topGoal + 4095) {
              Elevator.set(0.5);
            } else {
              Elevator.set(0.5 - ((topGoal - (elevatorPosition + 4095)) / (4095 * 2)));
            }
          } else {
            Elevator.set(0.0);
          }
        } else {
          Elevator.set(0.0);
        }
      }

    }

  }

  public void intakeLift() { // This is the function for the lift mechanism for the intake
    // use encoder for the motor, to be able to set position of
    // the up/down motion of the intake mecanism in respect of
    // the joystick throtle. use absolute position and see if i can set limits
    // to be able to use math to be able to relate a percent to the limits

    position = (sticky.getZ() + 1) / 2; // throtle axis
    // 'position' on a "0 to 1" scale instead of "-1 to 1"
    // example limits
    int topLim = 0;
    double ballPos = 454.5;
    double bottomLim = 1147.0777; // this is fully down
    double liftPos = (int) ((bottomLim - topLim) * position) + topLim;
    if (!encodeLift) {
      System.out.println("Using Manual Movement For Intake Lift");
      if (sticky.getRawButton(A) && upIntake.getAverageVoltage() < upper) {
        inLift.set(-1);
      } else if (sticky.getRawButton(B) && lowIntake.getAverageVoltage() < upper) {
        inLift.set(1);
      } else {
        inLift.set(0.0);
      }
    }

    if (encodeLift) {
      System.out.println("Using Encoder Movement For Intake Lift");

      if (inLiftPos < liftPos) {
        if (inLiftPos < liftPos - 63) {
          inLift.set(1);
        } else {
          inLift.set(1 - ((liftPos - (inLiftPos + 63)) / 63));
        }
      } else if (inLiftPos > liftPos) {
        if (inLiftPos > liftPos + 63) {
          inLift.set(-1);
        } else {
          inLift.set(-1 + ((liftPos - (inLiftPos - 63)) / 63));
        }
      } else {
        inLift.set(0.0);
      }

    }

  }

  public void ball() { // this is the fuction for the ball shooter on the elevator

    if (sticky.getRawButton(Fire)) {
      outBall.set(1);// sumting.set(Relay.Value.kForward);
    } else if (sticky.getRawButton(D)) {
      outBall.set(-0.5);
    } else {
      outBall.set(0.0);// sumting.set(Relay.Value.kOff);// or use kReverse if it needs to be powered in
    }

  }

  public void jack() { // this is the fuction for that thing on the back of the robot
    // that helps it get up the platform
    if (sticky.getRawButton(E)) {
      // power the Jack up device down
      Jack.set(ControlMode.PercentOutput, 1);
      x52Override = true;
    } else if (sticky.getRawButton(i)) {
      // power the Jack up device up
      Jack.set(ControlMode.PercentOutput, -1);
      x52Override = true;
    } else {
      Jack.set(ControlMode.PercentOutput, 0.0);
      x52Override = false;
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
