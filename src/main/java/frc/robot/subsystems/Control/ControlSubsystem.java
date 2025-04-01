package frc.robot.subsystems.Control;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Mechanism.CoralSubsystem;
import frc.robot.subsystems.Mechanism.ElevatorSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class ControlSubsystem extends SubsystemBase {

    private CoralSubsystem coralSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private VisionSubsystem visionSubsystem;
    private double nowLevel;

    public ControlSubsystem(CoralSubsystem coralSubsystem, 
        ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {

        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    public void CoralControl() {
        if (Math.abs(elevatorSubsystem.getPosition() - elevatorSubsystem.desiredPosition()) < 0.5 && 
            Math.abs(visionSubsystem.getReefTA() - VisionConstants.kReefPointY) < 0.5) {
            coralSubsystem.Shoot();
        }else if (elevatorSubsystem.getPosition() <= 1 && coralSubsystem.DetectCoral() == false) {
            coralSubsystem.Intake();
        }else {
            coralSubsystem.Wait();
        }
    }

    public double changeAngle() {
        if (nowLevel == 1) {
            return CoralConstants.kL1Angle;
        }else if (nowLevel == 2) {
            return CoralConstants.kL2Angle;
        }else if (nowLevel == 3) { 
            return CoralConstants.kL2Angle;
        }else if (nowLevel == 4) {
            return CoralConstants.kL4Angle;
        }else {
            return 0.0;
        }
    }

    public void AngleControl() {
        coralSubsystem.setAngle(changeAngle());
    }

    public void ElevatorControl() {
        boolean hasCoral = coralSubsystem.DetectCoral();
        boolean nearReef = visionSubsystem.getReefTA() > 4;
        if (hasCoral && nearReef) {
            nowLevel = elevatorSubsystem.desiredPosition();
            elevatorSubsystem.setPosition(elevatorSubsystem.desiredPosition());
        }else {
            nowLevel = 0;
            elevatorSubsystem.setPosition(0);
        }
        elevatorSubsystem.updateLevelSelection();
    }

    @Override   
    public void periodic() {
        CoralControl();
        ElevatorControl();
        AngleControl();
    }
    
}
