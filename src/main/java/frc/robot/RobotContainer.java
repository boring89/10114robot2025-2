// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;



public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(null, null);
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -m_Joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> true));
    
    new POVButton(m_Joystick, 270).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> visionSubsystem.reefXOutput(VisionConstants.kLreefPointX), 
        () -> visionSubsystem.reefYOutput(), 
        () -> visionSubsystem.reefTurnOutput(), 
        () -> false));

    new POVButton(m_Joystick, 90).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> visionSubsystem.reefXOutput(VisionConstants.kRreefPointX), 
        () -> visionSubsystem.reefYOutput(), 
        () -> visionSubsystem.reefTurnOutput(), 
        () -> false)
    );

    new JoystickButton(m_Joystick, 1).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> visionSubsystem.stationXOutput(), 
        () -> visionSubsystem.stationYOutput(), 
        () -> visionSubsystem.stationTurnOutput(), 
        () -> false)
    );
    new InstantCommand();
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    new JoystickButton(m_Joystick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new POVButton(m_Joystick, 0).whileTrue(new InstantCommand(() -> elevatorSubsystem.LevelUp()));
    new POVButton(m_Joystick, 180).whileTrue(new InstantCommand(() -> elevatorSubsystem.LevelDown()));
  }

  public Command getAutonomousCommand(){
    // return new SwerveJoystickCmd(swerveSubsystem, () -> -0.4, () -> 0.0, () -> 0.0, () -> false).withTimeout(6)
    // .andThen(new InstantCommand(() -> System.out.println("test"))).withTimeout(2)
    // .andThen(new RunCommand(() -> coralSubsystem.Shoot(), coralSubsystem)).withTimeout(3);
    return null;
  }
}

