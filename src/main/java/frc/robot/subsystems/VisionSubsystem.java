package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable reef, station;
    private NetworkTableEntry reefTA, reefTY, reefTID, stationTX, stationTY, stationTID;
    private PIDController MoveControllerX, MoveControllerY, TurnController;
    private double[] reefPose, stationPose;
    private double reefXOutput, reefYOutput, reefTurnOutput, stationXOutput, stationYOutput, stationTurnOutput, ApriltagID;


    public VisionSubsystem() {
        //get-Limelight
        reef = NetworkTableInstance.getDefault().getTable("limelight-reef");
        station = NetworkTableInstance.getDefault().getTable("limelight-station");
        //get-Reef-Apriltag-ID
        reefTID = reef.getEntry("tid");
        //get-Station-Apriltag-ID
        //get-REEF-Value
        reefTA = reef.getEntry("ta");
        reefTY = reef.getEntry("ty");
        reefPose = reef.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        //get-Station-value
        stationTX = station.getEntry("tx");
        stationTY = station.getEntry("ty");
        stationPose = station.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        //Move-PID-Controller (X-Axis)
        MoveControllerX = new PIDController(
            VisionConstants.kPX, VisionConstants.kIX, VisionConstants.kDX);
        //Move-PID-Controller (Y-Axis)
        MoveControllerY = new PIDController(
            VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY);
        //Turn-PID-Controller
        TurnController = new PIDController(
            VisionConstants.kPT, VisionConstants.kIT, VisionConstants.kDT);
    }
    //Reef-Aim
    public double reefXOutput(double setPoint) {
        if (reefTA.getDouble(0) < 3) {
            reefXOutput = MoveControllerX.calculate(reefTA.getDouble(0.0), 0.0);
        }else {
            reefXOutput = MoveControllerX.calculate(reefTA.getDouble(0.0), setPoint);
        }
        return reefXOutput;
    }

    public double reefYOutput() {
        reefYOutput = MoveControllerY.calculate(
            reefTY.getDouble(0), VisionConstants.kReefPointY);
        return reefYOutput;
    }
    
    public double reefTurnOutput() {
        reefTurnOutput = TurnController.calculate(reefPose[4], 0.0);
        return reefTurnOutput;
    }
    //Station-Aim
    public double stationXOutput() {
        stationXOutput = MoveControllerX.calculate(stationTX.getDouble(0), VisionConstants.kStationPointX);
        return stationXOutput;
    }

    public double stationYOutput() {
        stationYOutput = MoveControllerY.calculate(stationTY.getDouble(0), VisionConstants.kStationPointY);
        return stationYOutput;
    }

    public double stationTurnOutput() {
        stationTurnOutput = TurnController.calculate(stationPose[4], 0.0);
        return stationTurnOutput;
    }
    //get-Apriltag-ID
    public double getApriltagID() {
        ApriltagID =  reefTID.getDouble(0);
        if (ApriltagID == 0) {
            return 0;
        }else {
            return ApriltagID;
        }
    }
    //Detect-Station-Apriltag
    public boolean isStationDetected() {
        if (stationTID.getDouble(0) == 0) {
            return false;
        }else {
            return true;
        }
    }

    public double getReefTA() {
        return reefTA.getDouble(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ApriltagID", getApriltagID());
        SmartDashboard.putBoolean("isStationDetected", isStationDetected());
    }

}