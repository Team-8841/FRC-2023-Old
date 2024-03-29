package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.helpers.Orientation;
import frc.robot.helpers.Vector2;

public class Vision extends SubsystemBase{

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Start in drive mode (color camera no leds)
    private boolean driveMode = true;

    // gets
    private final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private final NetworkTableEntry ty = limelightTable.getEntry("ty");
    private final NetworkTableEntry ta = limelightTable.getEntry("ta");
    private final NetworkTableEntry tc = limelightTable.getEntry("tc");

    // sets
    private final NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
    private final NetworkTableEntry camMode = limelightTable.getEntry("camMode");
    // private final NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
    // private final NetworkTableEntry stream = limelightTable.getEntry("stream");
    // private final NetworkTableEntry snapshot = limelightTable.getEntry("snapshot");


    public Vision() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateStatus();
    }

    public double isTargetValid() {
        return tv.getDouble(0.0);
    }

    public double getTargetHorizontalOffset() {
        if (isTargetValid() > 0.5) {
            return tx.getDouble(0.0);
        } else {
            return 0;
        }
    }

    public double getTargetVerticalOffset() {
        if (isTargetValid() > 0.5) {
            return ty.getDouble(0.0);
        } else {
            return 0;
        }
    }
    //TODO: test to make sure elements are in correct positions
    public Orientation getOrientation() {

        double[] res = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        Orientation orientation = new Orientation(res[0], res[1], res[2], new Vector2(res[3], res[4]));
        SmartDashboard.putNumber("vision roll", orientation.roll);
        SmartDashboard.putNumber("vision pitch", orientation.roll);
        SmartDashboard.putNumber("vision yaw", orientation.yaw);
        SmartDashboard.putNumber("vision test", Math.random());
        SmartDashboard.putString("vision position", orientation.position.toString(3));
        SmartDashboard.putNumber("vision test2", ty.getDouble(69));
        /*if(res != null) {
            return new Orientation(res[0], res[1], res[2], new Vector2(res[3], res[4]));
        } else {
            return null;
        }*/
        return null;
    }
    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public void toggleDriveMode() {

        if (!driveMode) {
            // Drive mode (Color camera no leds)
            camMode.setNumber(1);
            ledMode.setNumber(1);
            driveMode = true;
        } else {
            // vision mode (Black and white camera all leds)
            camMode.setNumber(0);
            ledMode.setNumber(3);
            driveMode = false;
        }
    }

    public double getDistance() {

        double angleofTarget = getTargetVerticalOffset();

        return (VisionConstants.heightOfTarget - VisionConstants.heightOfCamera) 
                / Math.tan(Math.toRadians(VisionConstants.angleOfCamera + angleofTarget));
    }

    public void updateStatus() {
        /*SmartDashboard.putNumber("[Vision] Valid Target", isTargetValid());
        // SmartDashboard.putNumber("[Vision] Target Horizontal Offset", tx.getDouble(0.0));
        // SmartDashboard.putNumber("[Vision] Target Vertical Offset", ty.getDouble(0.0));
        // SmartDashboard.putNumber("[Vision] Target Area", getTargetArea());
        SmartDashboard.putNumber("[Vision] Distance", getDistance());*/
    }

    
}
