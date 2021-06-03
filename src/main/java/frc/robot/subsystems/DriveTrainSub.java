/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrainSub extends SubsystemBase {

 
  Pose2d pose; 
  Field2d field;

  private final CANSparkMax rightAlpha, leftAlpha, rightBeta, leftBeta; 
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  DifferentialDrive robot;
  Gyro gyro = new AnalogGyro(0); 
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d()); 

    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    
    PIDController leftPIDController = new PIDController(Constants.kP,0,0);
    PIDController rightPIDController = new PIDController(Constants.kP,0,0);
   

 
  /**
   * Creates a new DriveTrainSub.
   */
  public DriveTrainSub() {
    //alpha motors 
    rightAlpha = new CANSparkMax(Constants.RIGHT_ALPHA_DRIVE, MotorType.kBrushless);
    rightAlpha.restoreFactoryDefaults();
    rightAlpha.setInverted(false);
    rightAlpha.setIdleMode(IdleMode.kBrake);
    rightAlpha.setClosedLoopRampRate(Constants.CLOSED_LOOP_RAMP_RATE);
    rightAlpha.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP_RATE);
    leftAlpha = new CANSparkMax(Constants.LEFT_ALPHA_DRIVE, MotorType.kBrushless);
    leftAlpha.restoreFactoryDefaults();
    leftAlpha.setInverted(false);
    leftAlpha.setIdleMode(IdleMode.kBrake);
    leftAlpha.setClosedLoopRampRate(Constants.CLOSED_LOOP_RAMP_RATE);
    leftAlpha.setOpenLoopRampRate(Constants.OPEN_LOOP_RAMP_RATE);
    //beta motors 
    rightBeta = new CANSparkMax(Constants.RIGHT_BETA_DRIVE, MotorType.kBrushless);
    rightBeta.restoreFactoryDefaults();
    rightBeta.setInverted(false);
    rightBeta.setIdleMode(IdleMode.kBrake);
    rightBeta.follow(rightAlpha);
    leftBeta = new CANSparkMax(Constants.LEFT_BETA_DRIVE, MotorType.kBrushless);
    leftBeta.restoreFactoryDefaults();
    leftBeta.setInverted(false);
    leftBeta.setIdleMode(IdleMode.kBrake);
    leftBeta.follow(leftAlpha);
    //controller groups
    leftMotors = new SpeedControllerGroup(leftAlpha, leftBeta);
    rightMotors = new SpeedControllerGroup(rightAlpha, rightBeta);
    robot = new DifferentialDrive(leftMotors, rightMotors);

    //create pose and field objects 
     pose = new Pose2d();
     field = new Field2d();
     resetGyro();
     resetEncoders(); 

  } 
 
  
  @Override
  public void periodic() {
    //display numbers so i can pretend to troubleshoot 
    SmartDashboard.putNumber("Left Side Distance", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right side Distance", getRightDistanceMeters());
    SmartDashboard.putNumber("gyro turn", -gyro.getAngle());
    SmartDashboard.putData("Field", field);
    //update robot position 
    pose = odometry.update(getHeading() , getLeftDistanceMeters()
    , getRightDistanceMeters());  
    //set field position(idk what this does)
    field.setRobotPose(odometry.getPoseMeters());
  }
  
  //retunrs the speed of the robot 
  public DifferentialDriveWheelSpeeds getSpeeds() {  
    return new DifferentialDriveWheelSpeeds(
      leftAlpha.getEncoder().getVelocity() / 4.01 * 2 * Math.PI * Units.inchesToMeters(1.565) / 60
    , rightAlpha.getEncoder().getVelocity() / 4.01 * 2 * Math.PI * Units.inchesToMeters(1.565) / 60
    );
  }
    //return PID stuff
    public SimpleMotorFeedforward getFeedForward() {
      return feedForward;
    }
    public PIDController getLeftController(){
        return leftPIDController;
    }
    public PIDController getRightController(){
      return rightPIDController;
    }
    public DifferentialDriveKinematics getKinematics(){
      return kinematics;
    }
    public DifferentialDriveOdometry getOdometry(){
      return odometry;
    }
    public Pose2d getPose(){
      return pose; 
   }
   //reset important things 
    public void resetGyro(){
        gyro.reset(); 
    }
    public void reset() {
      odometry.resetPosition(new Pose2d(), new Rotation2d());
    }
    public void resetEncoders(){
      leftAlpha.getEncoder().setPosition(0);
      rightAlpha.getEncoder().setPosition(0);
    }
   //update where the robot is looking(gyro)
   public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
   }
   //return left distance 
   public double getLeftDistanceMeters(){
    return Units.inchesToMeters(leftAlpha.getEncoder().getPosition()*1.565*2*Math.PI);
   }
   //return right distance 
   public double getRightDistanceMeters(){
    return Units.inchesToMeters(rightAlpha.getEncoder().getPosition()*1.565*2*Math.PI);
   }
   //set voltages, and flip motors cuz they do that 
   public void setOutputs(double leftVolts, double rightVolts){
    rightAlpha.setInverted(true);
    rightBeta.setInverted(true);
    rightAlpha.set(rightVolts / 12);
    leftAlpha.set(leftVolts / 12);
   }
    //arcade drive method 
   public void arcadeDrive(XboxController controller, double speed){
      rightAlpha.setInverted(false);
      rightBeta.setInverted(false);
      robot.arcadeDrive(controller.getRawAxis(Constants.XBOX_RIGHT_TRIGGER)*speed - controller.getRawAxis(Constants.XBOX_LEFT_TRIGGER)*speed
      , controller.getRawAxis(Constants.XBOX_LEFT_X_AXIS)*speed);
   }
    //curvature drive method that i could probably get rid of 
   public void curvatureDrive(XboxController controller, double speed){

      double xSpeed = controller.getRawAxis(Constants.XBOX_RIGHT_TRIGGER)-controller.getRawAxis(Constants.XBOX_LEFT_TRIGGER);
      double zRotation = controller.getRawAxis(Constants.XBOX_LEFT_X_AXIS);
      boolean isQuickTurn = true; 
      robot.curvatureDrive(xSpeed*speed, zRotation*speed, isQuickTurn);
    }
    //test command 
    public void driveForward(double speed){
        robot.tankDrive(speed, speed);
    }
    //self explanatory 
    public void stop(){
      robot.stopMotor();
    }

}
