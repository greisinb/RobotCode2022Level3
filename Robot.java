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
  private Joystick driverStick; //names a joystick named driverStick
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
    rightMotors.setInverted(true);
    myRobot = new DifferentialDrive(leftMotors, rightMotors);
    driverStick = new Joystick(0); //creates a new joystick on USB input 0
    intakeMotor = new CANSparkMax(5, MotorType.kBrushed);
    lifterMotor = new CANSparkMax(6, MotorType.kBrushless);
    shooterMotor = new CANSparkMax(7, MotorType.kBrushless);
    shooterEncoder = shooterMotor.getEncoder(); 
    lifterSwitch = new DigitalInput(9);
    shooterPIDController = shooterMotor.getPIDController();
    // set PID coefficients
    shooterPIDController.setP(0.006);
    shooterPIDController.setI(0);
    shooterPIDController.setD(0.005); //.005
    shooterPIDController.setIZone(0); //100
    shooterPIDController.setFF(0.000015);
    shooterPIDController.setOutputRange(0, 1);
    maxRPM = 5700;


  }

  @Override
  public void robotPeriodic() {
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
    if(shooterSpeed > 4000) { //runs the lifter motor only if the shooter motor has reached speed
      lifterMotor.set(0.4);
      ballPresent = false;  //assumes that the ball has been launched
    }
    else{
      lifterMotor.set(0); //shuts off the lifter if shooter is under speed
    }
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() { //drives the robot forward for 2 seconds during auto
    double time = Timer.getFPGATimestamp();
    if(time - startTime < 2){
      myRobot.arcadeDrive(-0.5, 0);
    }
    else {
      myRobot.arcadeDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    intakeSolenoid.set(Value.kReverse); //brings up the intake bar at teleop start
  }

  @Override
  public void teleopPeriodic() {
    if(driverStick.getRawButtonPressed(2)) {
      intakeMotor.set(0.5);
      intakeSolenoid.set(Value.kForward); //sets solenoid in forward position, can also set for kOff and kReverse

    }

    if(driverStick.getRawButtonReleased(2)){
      double startTime = Timer.getFPGATimestamp();
      double time = Timer.getFPGATimestamp();
      intakeSolenoid.set(Value.kReverse); //sets solenoid in reverse position, can also set for kOff and kForward
      while(!((time - startTime > 5) || (ballPresent))){ //runs the lifter until a ball is detected or 5 seconds have passed
        intakeMotor.set(0.5);
        lifterMotor.set(0.5);
        time = Timer.getFPGATimestamp();
        ballPresent = lifterSwitch.get();
        myRobot.arcadeDrive(-driverStick.getY(), driverStick.getX());
      }
      intakeMotor.set(0); //when while loop is exited shut off motors and set ball present to false
      lifterMotor.set(0);
      //ballPresent = false;
    }
    if (driverStick.getRawButtonPressed(1)){
      shooterPIDController.setReference(5700, CANSparkMax.ControlType.kVelocity);
      
    }
    if(driverStick.getRawButtonReleased(1)){
      shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      
    }
    //Vision autopilot command
    while(driverStick.getRawButton(3)){
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double heading_error = x;
      double distance_error = y;
      myRobot.arcadeDrive(-distance_error*.05, heading_error*.05); //auto drive to zero error * a proportion constant, may need to add a minimum drive value
     // shooterPIDController.setReference(4000, CANSparkMax.ControlType.kVelocity); //spool up shooter wheels for shot preperation

    }

   myRobot.arcadeDrive(-driverStick.getY(), driverStick.getX()); 

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
