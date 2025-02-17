// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpSequence extends SequentialCommandGroup {
  /** Creates a new PickUpSequence. */
  public PickUpSequence(PivotSubsystem pivotsub, IntakeSubsystem intakesub,ShooterSubsystem shooter, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToIntakeHelperCmd(pivotsub, intakesub),

      new ParallelCommandGroup(
        new MoveToGroundSetCmd(pivotsub, intakesub),
        new IntakeSetCmd(intakesub, shooter, speed)
      ),
      new  MoveToReadySetCmd(pivotsub, intakesub, shooter)
    );
  }

  /*public PickUpSequence(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem,
      PivotSubsystem m_PivotSubsystem) {
    TODO Auto-generated constructor stub
  }/* */
}
