/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TimedDrive extends CommandBase {
    DriveTrainSub driveTrain;
    Index index; 
    Intake intake; 
    Shooter shoot; 

    private boolean finish = false; 
    Timer timer; 
  /**
   * Creates a new TimedDrive.
   */
  public TimedDrive(DriveTrainSub dt, Index id, Intake it, Shooter s){
      driveTrain = dt; 
      index = id;
      intake = it; 
      shoot = s;
      addRequirements(driveTrain);
      addRequirements(index);
      addRequirements(intake);
      addRequirements(shoot);
      timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    //go forward and intake for 4 seconds
    while(timer.get()<Constants.TIMED_DRIVE){
        driveTrain.driveForward(Constants.AUTO_SPEED);
        intake.intakeBall(Constants.INTAKE_SPEED);
        
    }
    while(timer.get()<3){
      intake.intakeBall(Constants.INTAKE_SPEED);
    }
    driveTrain.stop();
    intake.stop();
    timer.reset();
    timer.start();
    //go forward and intake for 4 seconds
    while(timer.get()<10){
        shoot.shootBall(Constants.SHOOT_SPEED);
        index.spinPlate(Constants.PLATE_SPEED);
        if(timer.get()>=3){
          index.indexBall(Constants.INDEX_SPEED);
        }
    }

    finish = true;





  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stopIndex();
    shoot.stop();
    intake.stop();
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
