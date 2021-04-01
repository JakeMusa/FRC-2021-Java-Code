/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {


      private final CANSparkMax leftShooter,rightShooter;  

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    leftShooter = new CANSparkMax(Constants.LEFT_SHOOT, MotorType.kBrushless);
    leftShooter.restoreFactoryDefaults();
    leftShooter.setInverted(true);
    leftShooter.setIdleMode(IdleMode.kCoast);
    leftShooter.setClosedLoopRampRate(Constants.SHOOT_RAMP);
    leftShooter.setOpenLoopRampRate(Constants.SHOOT_RAMP);


    rightShooter = new CANSparkMax(Constants.RIGHT_SHOOT, MotorType.kBrushless);
    rightShooter.restoreFactoryDefaults();
    rightShooter.setInverted(false);
    rightShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.setClosedLoopRampRate(Constants.SHOOT_RAMP);
    rightShooter.setOpenLoopRampRate(Constants.SHOOT_RAMP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shootBall(double speed){
    leftShooter.set(speed);
    rightShooter.set(speed);
  }
  public void stop(){
    leftShooter.set(0);
    rightShooter.set(0);
  }

}
