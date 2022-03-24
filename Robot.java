// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //imports library for running a differential drive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Joystick;// imports a library for communicating with a joystick

import com.revrobotics.CANSparkMax; //imports library for CAN controlled SparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //imports library for SmarkMax control functions (required)

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

//Camera Imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive myRobot; //names a differential drive named myRobot
  private Joystick leftStick; //names a joystick named driverStick
  private Joystick rightStick;
  private CANSparkMax leftFrontMotor; //names CAN speed controllers named with the motor they are attached to
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private CANSparkMax intakeMotor;
  private CANSparkMax lifterMotor;
  private CANSparkMax shooterMotor;
  public Double startTime;
  private RelativeEncoder shooterEncoder; //declares an encoder on the shooter
  private DigitalInput lifterSwitch; 
  boolean ballPresent; //decalres the ball present varible
  private SparkMaxPIDController shooterPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  Solenoid targetLights = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  private CANSparkMax climberMotor;

  //adjustable variables
  public final int shootSpeed = 5700;  //sets shooterspeed for launching the ball
  public final double liftSpeed = 0.35;

  
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); //creates double solenoid (solenoid2) on output 0 and 1
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless); //creates a CAN speed controller for leftFrontMotor w/CAN ID #1 set for brushless mode
    leftRearMotor = new CANSparkMax(2, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(4, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(3, MotorType.kBrushless);
    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor); //groups left motors together as a slingle object called left motors
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
    myRobot = new DifferentialDrive(leftMotors, rightMotors);
    leftStick = new Joystick(0); //creates a new joystick on USB input 0
    rightStick = new Joystick(1);
    intakeMotor = new CANSparkMax(5, MotorType.kBrushed);
    lifterMotor = new CANSparkMax(6, MotorType.kBrushless);
    shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
    climberMotor = new CANSparkMax(8, MotorType.kBrushed);
    shooterEncoder = shooterMotor.getEncoder(); 
    lifterSwitch = new DigitalInput(9);
    shooterPIDController = shooterMotor.getPIDController();
    rightMotors.setInverted(true);
    // set PID coefficients
    shooterPIDController.setP(0.0018);
    shooterPIDController.setI(0.0062);
    shooterPIDController.setD(0.000); 
    shooterPIDController.setIZone(0); 
    shooterPIDController.setFF(0.000015);
    shooterPIDController.setOutputRange(0, 1);
    maxRPM = 5700;
    CameraServer.startAutomaticCapture();

    

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder Velocity", shooterEncoder.getVelocity());
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double targetAquired = tv.getDouble(0.0);

    //post vision to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimeLightTargetAquired", targetAquired);
    if(targetAquired == 1){
      targetLights.set(true);
    }
    if(targetAquired == 0){
      targetLights.set(false);
    }
    double shooterSpeed = shooterEncoder.getVelocity();  //sets the shooter speed variable
    if(shooterSpeed > (shootSpeed-250)) { //runs the lifter motor only if the shooter motor has reached speed
      lifterMotor.set(liftSpeed);
    }
    else{
      lifterMotor.set(0); //shuts off the lifter if shooter is under speed
    }
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp(); //get start time of auto mode
    ballPresent = true; //set ball present to true because 1 will be pre-loaded
  }

  @Override
  public void autonomousPeriodic() { 
    double time = Timer.getFPGATimestamp(); //get current time

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //get vision variables
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double targetAquired = tv.getDouble(0.0);
    double heading_error = x;
    double distance_error = y;

    if(time - startTime < 2){ //for 2 seconds
      myRobot.arcadeDrive(0.5, 0); //drive forward
      intakeSolenoid.set(Value.kForward); //drop intake
      intakeMotor.set(0.5); //run intake wheels
    }
    else if((time - startTime > 2) && (time-startTime <4)){ //if time is greater than 2 and less than 4 seconds
      intakeSolenoid.set(Value.kReverse); //lift intake
      intakeMotor.set(0); //stop intake wheels
      myRobot.arcadeDrive(0, 0-.37);
      //rotate the robot
      }
    else if((time - startTime > 4) && (time-startTime <7)){ //if time is greater than 4 and less than 7
        myRobot.arcadeDrive(-distance_error*.05, heading_error*.05); //auto aim and range
      }
    else if((time - startTime > 7) && (time-startTime <14.5) && (targetAquired==1)){ //if time is greater than 7 and less than 11 and there is a visible target
      myRobot.arcadeDrive(0, .0); //stop the robot
      shooterPIDController.setReference(shootSpeed, CANSparkMax.ControlType.kVelocity); //shoot
      ballPresent=false; //set ball present to false because the ball will have been shot
    }
    //else if((time - startTime > 11) && (time-startTime <14.5)){ //if time is greater than 11 and less than 14.5
      //shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity); //turn off shooter
      //myRobot.arcadeDrive(-.6, 0); //drive as needed to leave start area for cross the line points
    //}
    else{ //failsafe to shut off all mechanisms
      intakeSolenoid.set(Value.kReverse); //lift intake
      intakeMotor.set(0); //stop intake wheels
      shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity); //turn off shooter
      myRobot.arcadeDrive(0, .0); //stop robot
    }



    }
    
    
  

  @Override
  public void teleopInit() {
    intakeSolenoid.set(Value.kReverse); //brings up the intake bar at teleop start
  }

  @Override
  public void teleopPeriodic() {
    if(rightStick.getRawButtonPressed(2)) {
      intakeMotor.set(0.5);
      intakeSolenoid.set(Value.kForward); //sets solenoid in forward position, can also set for kOff and kReverse

    }

    if(rightStick.getRawButtonReleased(2)){
      double startTime = Timer.getFPGATimestamp();
      double time = Timer.getFPGATimestamp();
      intakeSolenoid.set(Value.kReverse); //sets solenoid in reverse position, can also set for kOff and kForward
      while(!((time - startTime > 1.5) || (ballPresent))){ //runs the lifter until a ball is detected or 1.5 seconds have passed
        intakeMotor.set(0.6);
        lifterMotor.set(liftSpeed);
        time = Timer.getFPGATimestamp();
        ballPresent = lifterSwitch.get();
        myRobot.tankDrive(-leftStick.getY(), -rightStick.getY());
      }
      intakeMotor.set(0); //when while loop is exited shut off motors and set ball present to false
      lifterMotor.set(0);
      
    }
    if (rightStick.getRawButtonPressed(1)){
      shooterPIDController.setReference(shootSpeed, CANSparkMax.ControlType.kVelocity);
      
    }
    if(rightStick.getRawButtonReleased(1)){
      shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      ballPresent = false;  //assumes that the ball has been launched
    }
    if(leftStick.getRawButtonPressed(3)){ //climber in
      climberMotor.set(1);
    }
    if(leftStick.getRawButtonReleased(3)){ //climber stop
      climberMotor.set(0);
    }
    if(leftStick.getRawButtonPressed(2)){ //climber out
      climberMotor.set(-1);
    }
    if(leftStick.getRawButtonReleased(2)){ //climber stop
      climberMotor.set(0);
    }
    if(rightStick.getRawButtonPressed(4)){ //eject ball
      intakeMotor.set(-0.5);
      lifterMotor.set(-0.5);
    }
    if(rightStick.getRawButtonReleased(4)){
      lifterMotor.set(0);
      intakeMotor.set(0);
    }
    //Vision autopilot command
    while(rightStick.getRawButton(3)){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double heading_error = x;
      double distance_error = y;
      if(distance_error<0){
        myRobot.arcadeDrive(-distance_error*.15, heading_error*.05); //auto drive to zero error * a proportion constant, may need to add a minimum drive value
        //shooterPIDController.setReference((shootSpeed-750), CANSparkMax.ControlType.kVelocity); //spool up shooter to just under shoot speed

      }
      else{
        myRobot.arcadeDrive(-distance_error*.07, heading_error*.05); //auto drive to zero error * a proportion constant, may need to add a minimum drive value
        //shooterPIDController.setReference((shootSpeed-750), CANSparkMax.ControlType.kVelocity); //spool up shooter to just under shoot speed

      }

    }
myRobot.tankDrive(-leftStick.getY(), -rightStick.getY());
    //myRobot.tankDrive((Math.pow(-leftStick.getY(), 3)+-leftStick.getY()), (Math.pow(-rightStick.getY(), 3)+-rightStick.getY()));

  }

  @Override
public void disabledInit() {}
  

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
