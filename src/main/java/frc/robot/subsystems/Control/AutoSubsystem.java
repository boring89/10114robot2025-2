package frc.robot.subsystems.Control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetain.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class AutoSubsystem extends SubsystemBase {

    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final PIDController AutoController;
    
    public AutoSubsystem(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        AutoController = new PIDController(0.1, 0.1, 0.1);
    }

    public int StartPathChoose() {
        if (visionSubsystem.getApriltagID() == 9 || visionSubsystem.getApriltagID() == 22) {
            return 1; //right
        }else if (visionSubsystem.getApriltagID() == 11 || visionSubsystem.getApriltagID() == 20) {
            return 2; //left
        }else {
            return 3; // middle
        }

    }

    public double AutoTurn(double setAngle) {
        return AutoController.calculate(swerveSubsystem.getHeading(), setAngle);
    }
}
