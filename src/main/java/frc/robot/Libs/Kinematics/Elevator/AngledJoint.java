package frc.robot.Libs.Kinematics.Elevator;

public class AngledJoint {
    private double maxHeightMeter;
    private double tiltRadians;

    // For tilt radians, the angle of the joint from the horizontal, so degrees would be on the ground
    // and 90 degrees would be straight up
    public AngledJoint(double maxHeightMeter, double tiltRadians) {
        this.maxHeightMeter = maxHeightMeter;
        this.tiltRadians = tiltRadians;
    }

    public double forwardKinematics(double yMeters) {
        double position = yMeters / Math.sin(tiltRadians);

        position = Math.min(position, maxHeightMeter);

        return position;
    }

    public double inverseKinematics(double heightMeters) {
        double position = heightMeters * Math.sin(tiltRadians);

        return position;
    }

    // Getters
    public double getMaxHeightMeter() {
        return maxHeightMeter;
    }

    public double getTiltRadians() {
        return tiltRadians;
    }

    // Setters
    public void setMaxHeightMeter(double maxHeightMeter) {
        this.maxHeightMeter = maxHeightMeter;
    }

    public void setTiltRadians(double tiltRadians) {
        this.tiltRadians = tiltRadians;
    }
}