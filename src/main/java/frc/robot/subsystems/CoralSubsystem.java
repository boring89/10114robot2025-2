package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase{
    
    private final SparkMax CoralMotor;
    private final AnalogInput IntakeIR;
    private double IntakeSpeed;
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public CoralSubsystem() {

        CoralMotor = new SparkMax(20, MotorType.kBrushless);
        IntakeIR = new AnalogInput(0);
    }

    public void Wait() {
        IntakeSpeed = 0;
    }

    public void Intake() {
        IntakeSpeed = 0.3;
    }

    public void Shoot() {
        IntakeSpeed = 0.5;
    }

    public boolean DetectCoral() {
        if (getIRVoltage() < CoralConstants.IRVoltage) {
            return false;
        }else {
            return true;
        }
    }

    public double getIRVoltage() {
        return IntakeIR.getVoltage();
    }

    @Override
    public void periodic() {
        CoralMotor.set(IntakeSpeed);

        if (elevatorSubsystem.getPosition() >= elevatorSubsystem.desiredPosition()) {
            Shoot();
        }else if (elevatorSubsystem.getPosition() <= 1 && DetectCoral() == false) {
            Intake();
        }else {
            Wait();
        }
    }
}