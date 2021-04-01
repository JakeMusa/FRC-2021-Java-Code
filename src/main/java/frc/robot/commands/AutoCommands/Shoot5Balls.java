// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IndexBall;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot5Balls extends ParallelCommandGroup {
  /** Creates a new Shoot5Balls. */
  public Shoot5Balls(Index i, Shooter s) {
    addCommands(new IndexBall(i),new ShootBall(s));
  }
}
