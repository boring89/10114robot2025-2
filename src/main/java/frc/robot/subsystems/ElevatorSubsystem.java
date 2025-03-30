package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX ElevatorMotor;
    private final PIDController ElevatorController;
    private int level = 1;
    private double desiredPosition = 0.0;
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private boolean[] Level;

    public ElevatorSubsystem() {

        ElevatorMotor = new TalonFX(ElevatorConstants.ElevatorMotorId);
        ElevatorController = new PIDController(
            ElevatorConstants.kP, 
            ElevatorConstants.kI, 
            ElevatorConstants.kD);
        Level = new boolean[4];
    }
    //get-elevator-position
    public double getPosition() {
        return ElevatorMotor.getPosition().getValueAsDouble();
    }
    //set-to-desired-position
    public void setPosition(double setPoint) {
        ElevatorMotor.set(ElevatorController.calculate(getPosition(), setPoint));
    }

    public void LevelUp() {
        if (level < 4) {
            level++;
        }
    }
    public void LevelDown() {
        if (level > 1) {
            level--;
        }
    }

    public void setLevel() {
        switch (level) {
            case 1:
                desiredPosition = ElevatorConstants.kL1;
                break;
            case 2:
                desiredPosition = ElevatorConstants.kL2;
                break;
            case 3:
                desiredPosition = ElevatorConstants.kL3;
                break;
            case 4:
                desiredPosition = ElevatorConstants.kL4;
                break;
            default:
                desiredPosition = 0.0;
                break;
            
        }

    }

    public void updateLevelSelection() {
        for (int i = 0; i < 3; i++) {
            Level[i] = false;
        }
        Level[level - 1] = true;
        for(int j = 1; j < 4; j++) {
            SmartDashboard.putBoolean("Level" + j, Level[j - 1]);
        }
    }

    public double desiredPosition() {
        return desiredPosition;
    }
 
    @Override
    public void periodic() {

        boolean hasCoral = coralSubsystem.DetectCoral();
        boolean nearReef = visionSubsystem.getReefTA() > 4;
        
        setLevel();
        if (hasCoral && nearReef) {
            setPosition(desiredPosition);
        }else {
            setPosition(0);
        }
        updateLevelSelection();
    }
}