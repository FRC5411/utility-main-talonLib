package frc.robot.Libs.Kinematics.Elevator;

public class AngledJoint {
    private double maxHeightMeter;
    private double tiltRadians;

    public AngledJoint(double maxHeightMeter, double tiltRadians) {
        this.maxHeightMeter = maxHeightMeter;
        this.tiltRadians = tiltRadians;
    }

    public double forwardKinematics(double yMeters) {
        double position = yMeters / Math.sin(tiltRadians);

        position = Math.min(position, maxHeightMeter);

        return position;
    }
}