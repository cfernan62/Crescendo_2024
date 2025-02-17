// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.AutoPivotGround;
import frc.robot.commands.AutoPivotShoot;
import frc.robot.commands.IntakeSetCmd;
//import frc.robot.commands.IntakeSetCmd;
import frc.robot.commands.PickUpSequence;
import frc.robot.commands.ReverseIntakeSetCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.TurboCommand;
import frc.robot.commands.ShooterSetCmd;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final boolean fieldmode = true;
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(1);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Reset Gyro", new InstantCommand(
      () -> m_robotDrive.zeroHeading()));
    NamedCommands.registerCommand("Score", new ShooterSetCmd(m_ShooterSubsystem, m_IntakeSubsystem, -1, 5100));
    //NamedCommands.registerCommand("Pick Up Ring", new PickUpSequenceAuto(m_PivotSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 0.3));
    NamedCommands.registerCommand("Pick Up Ring", new ParallelCommandGroup(
        new AutoPivotGround(m_PivotSubsystem),
        new IntakeSetCmd(m_IntakeSubsystem, m_ShooterSubsystem, 0.45)));
    NamedCommands.registerCommand("Pivot to Score", new AutoPivotShoot(m_PivotSubsystem));
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
     m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive( //Left and Right sticks (Driver)
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldmode, true),
            m_robotDrive));
    
    m_robotClimb.setDefaultCommand( //Left Trigger (Driver)
        new RunCommand(
            () -> m_robotClimb.setmotor(
                m_driverController.getLeftTriggerAxis()),
                m_robotClimb));

      m_PivotSubsystem.setDefaultCommand(new RunCommand( //Right Stick (Operator)
        () -> m_PivotSubsystem.setmotor(m_operatorController.getRightY() * 0.5),
        m_PivotSubsystem));

    m_VisionSubsystem.setDefaultCommand(new RunCommand(
        () -> m_VisionSubsystem.GetSpeakerDistance(),
        m_VisionSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 4) //Right Bumper, X-State
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    new JoystickButton(m_driverController, 5) //Left Bumper, Reset Gyro
        .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading()));


    new JoystickButton(m_driverController, 6) //Y, Turbo
        .onTrue(new TurboCommand(m_robotDrive,true))
        .onFalse(new TurboCommand(m_robotDrive, false));

    new JoystickButton(m_driverController, 8) //Start, Go Right
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0.0, -0.2, 0, fieldmode, true),
         m_robotDrive));
    
    new JoystickButton(m_driverController, 7) //Back, Go Left
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0.0, 0.2, 0, fieldmode, true),
         m_robotDrive));
        
        //OPERATOR
    

    new JoystickButton(m_operatorController, 5) //Left Bumper, Intake
        .whileTrue(new PickUpSequence(m_PivotSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 0.45 ));

    /*new JoystickButton(m_operatorController, 5) //Left Bumper, Intake
        .whileTrue(new IntakeSetCmd(m_IntakeSubsystem,m_ShooterSubsystem,m_PivotSubsystem, 0.3 ));*/
        
    

    new JoystickButton(m_operatorController, 7) //Left Bumper, Intake
        .whileTrue(new ReverseIntakeSetCmd(m_IntakeSubsystem,m_ShooterSubsystem, -0.25));

    new JoystickButton(m_operatorController, 6) //Right Bumper, Shoot Speaker
        .whileTrue(new ShooterSetCmd(m_ShooterSubsystem, m_IntakeSubsystem,  -1, 5100));
    
    new JoystickButton(m_operatorController, 3) //X, Shoot Amp
        .whileTrue(new ShooterSetCmd(m_ShooterSubsystem, m_IntakeSubsystem, -0.25, 1000));

    new JoystickButton(m_operatorController, 1) // A, Pivot to Drive
        .whileTrue(new RunCommand(
        () -> m_PivotSubsystem.ToDrive()));

    new JoystickButton(m_operatorController, 4) // Back button, Pivot to Amp
        .whileTrue(new RunCommand(
        () -> m_PivotSubsystem.ToAmp()));

    new JoystickButton(m_operatorController, 2) // B, Pivot to Speaker
        .whileTrue(new IntakeSetCmd(m_IntakeSubsystem, m_ShooterSubsystem, 0.3));

    new JoystickButton(m_operatorController, 8) // Y, Pivot to Climb/Idle
        .whileTrue(new RunCommand(
        () -> m_PivotSubsystem.ToIdleState()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
