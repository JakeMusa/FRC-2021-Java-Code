/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  VictorSP intakeMotor;
  DoubleSolenoid leftPiston,rightPiston;

  
  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor = new VictorSP(Constants.INTAKE_MOTOR);
    leftPiston = new DoubleSolenoid(0, 1);
    //rightPiston = new DoubleSolenoid(2, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeBall(double speed){
     //rightPiston.set(Value.kForward);
     leftPiston.set(Value.kReverse);
      intakeMotor.set(speed);
      }
  public void stop(){
    //rightPiston.set(Value.kReverse);
    leftPiston.set(Value.kForward);
    intakeMotor.set(0);
      }
      
  
}
