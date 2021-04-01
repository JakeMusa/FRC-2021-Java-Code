/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.IndexBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SpinPlate;
import frc.robot.commands.AutoCommands.Shoot5Balls;
import frc.robot.commands.AutoCommands.TimedDrive;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
// The robot's subsystems and commands are defined here...

  private final DriveTrainSub driveTrain; 
  private final ArcadeDrive arcadeDrive;
  private final CurvatureDrive curvatureDrive; 
  private final Intake intake; 
  private final Index index; 
  private final IndexBall indexBall; 
  private final IntakeBall intakeBall;
  public static XboxController driver;
  private final Shooter shooter;
  private final ShootBall shootBall;
  private final Shoot5Balls shoot5Balls;
  private final TimedDrive timedDrive;
  private final SpinPlate spinPlate; 
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // drivetrain
    // Configure the button bindings
    driveTrain = new DriveTrainSub();
    arcadeDrive = new ArcadeDrive(driveTrain);
    curvatureDrive = new CurvatureDrive(driveTrain);
    arcadeDrive.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(arcadeDrive);
    curvatureDrive.addRequirements(driveTrain);

    

    

    driver = new XboxController(Constants.JOYSTICK_NUMBER);
    // shooter
    shooter = new Shooter();
    shootBall = new ShootBall(shooter);
    shootBall.addRequirements(shooter);
   

    // intake
    intake = new Intake();
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);

    //index 
    index = new Index();
    indexBall = new IndexBall(index);
    indexBall.addRequirements(index);
    spinPlate = new SpinPlate(index);
    spinPlate.addRequirements(index);

    //auto shoot
    shoot5Balls = new Shoot5Balls(index, shooter);
    shoot5Balls.addRequirements(index, shooter);

    //test auto 
    timedDrive = new TimedDrive(driveTrain, index, intake, shooter); 
    timedDrive.addRequirements(driveTrain);
    timedDrive.addRequirements(index);
    timedDrive.addRequirements(intake);
    timedDrive.addRequirements(shooter);
   


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      //shooter control 
      JoystickButton shootButton = new JoystickButton(driver, XboxController.Button.kA.value);
      shootButton.whileHeld(new ShootBall(shooter));
      //intake control
      JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kB.value);
      intakeButton.toggleWhenPressed(new IntakeBall(intake));
      //auto shoot control 
      JoystickButton autoShootButton = new JoystickButton(driver, XboxController.Button.kY.value);
      autoShootButton.toggleWhenPressed(new Shoot5Balls(index ,shooter));
      //index button 
      JoystickButton indexButton = new JoystickButton(driver, XboxController.Button.kX.value);
      indexButton.whileHeld(new IndexBall(index));
      //plate shit 
      JoystickButton plateButton = new JoystickButton(driver, XboxController.Button.kBumperRight.value);
      plateButton.whileHeld(new SpinPlate(index));

    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous



      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),Units.feetToMeters(2));
      config.setKinematics(driveTrain.getKinematics());
      
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1.0,0,new Rotation2d(0))),config);

        RamseteCommand command = new RamseteCommand(
          trajectory,
          driveTrain::getPose, 
          new RamseteController(2.0, 0.7), 
          driveTrain.getFeedForward(), 
          driveTrain.getKinematics(), 
          driveTrain::getSpeeds, 
          driveTrain.getLeftController(), 
          driveTrain.getRightController(), 
          driveTrain::setOutputs, 
          driveTrain);

          
          return command; 
  }

}
