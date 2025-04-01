// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Control.AutoSubsystem;
import frc.robot.subsystems.Control.ControlSubsystem;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Mechanism.CoralSubsystem;
import frc.robot.subsystems.Mechanism.ElevatorSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;



public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();  
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ControlSubsystem controlSubsystem = new ControlSubsystem(coralSubsystem, elevatorSubsystem, visionSubsystem);
  private final AutoSubsystem autoSubsystem = new AutoSubsystem(swerveSubsystem, visionSubsystem);
  private double MoveTimeX, MoveTimeY, Turn;

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
    controlSubsystem.periodic();
    new InstantCommand();
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    new JoystickButton(m_Joystick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new POVButton(m_Joystick, 0).whileTrue(new InstantCommand(() -> elevatorSubsystem.LevelUp()));
    new POVButton(m_Joystick, 180).whileTrue(new InstantCommand(() -> elevatorSubsystem.LevelDown()));
  }

  public Command getAutonomousCommand(){
    int Auto = autoSubsystem.StartPathChoose();

    if (Auto == 1) {
      MoveTimeX = 3;
      MoveTimeY = 4;
      Turn = 315;
    }else if (Auto == 2) {
      MoveTimeX = 3;
      MoveTimeY = 4;
      Turn = 45;
    }else {
      MoveTimeX = 5;
      MoveTimeY = 3.5;
      Turn = 315;
    }
    return new SwerveJoystickCmd(swerveSubsystem, 
      () -> visionSubsystem.reefXOutput(VisionConstants.kLreefPointX), 
      () -> visionSubsystem.reefYOutput(), 
      () -> visionSubsystem.reefTurnOutput(), 
      () -> false).withTimeout(3)
      .andThen(new SwerveJoystickCmd(swerveSubsystem, 
      () -> 0.4, 
      () -> 0.0, 
      () -> autoSubsystem.AutoTurn(Turn), 
      () -> true).withTimeout(MoveTimeX))
      .andThen(new SwerveJoystickCmd(swerveSubsystem, 
      () -> 0.0, 
      () -> 0.7, 
      () -> autoSubsystem.AutoTurn(Turn), 
      () -> true).withTimeout(MoveTimeY)
      .andThen(new SwerveJoystickCmd(swerveSubsystem, 
      () -> -visionSubsystem.stationXOutput(), 
      () -> -visionSubsystem.stationYOutput(), 
      () -> visionSubsystem.stationTurnOutput(), 
      () -> false))).withTimeout(3)
      .andThen(new SwerveJoystickCmd(swerveSubsystem, 
      () -> visionSubsystem.reefXOutput(VisionConstants.kRreefPointX), 
      () -> visionSubsystem.reefYOutput(), 
      () -> visionSubsystem.reefTurnOutput(), 
      () -> false).withTimeout(4));
  }
}

