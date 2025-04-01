package frc.robot.subsystems.Mechanism;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase{
    
    private final SparkMax CoralMotor;
    private final SparkMax AngleMotor;

    private final RelativeEncoder AngleEncoder;

    private final AnalogInput IntakeIR;

    private final PIDController AngleController;

    private double IntakeSpeed;

    public CoralSubsystem() {

        CoralMotor = new SparkMax(20, MotorType.kBrushless);
        AngleMotor = new SparkMax(21, MotorType.kBrushless);
        AngleEncoder = AngleMotor.getEncoder();
        IntakeIR = new AnalogInput(0);

        AngleController = new PIDController(
            CoralConstants.kPAngle, 
            CoralConstants.kIAngle, 
            CoralConstants.kDAngle);
    }

    public double getPosition() {
        return AngleEncoder.getPosition();
    }

    public void setAngle(double setPoint) {
        AngleMotor.set(AngleController.calculate(getPosition(), setPoint));
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
    }
}