package frc.robot.Libs.Kinematics.Elevator;

public class SingleJoint {
    private double maxHeight;
    private double appendLength;
    private double appendAngle;

    public SingleJoint(double maxHeightMeter, double appendLengthMeter, double appendAngleRadians) {
        this.maxHeight = maxHeight;
        this.appendLength = appendLength;
        this.appendAngle = appendAngle;
    }

    public double[] forwardKinematics(double xMeters) {
        double[] position = new double[2];

        

        return position;
    }
 }