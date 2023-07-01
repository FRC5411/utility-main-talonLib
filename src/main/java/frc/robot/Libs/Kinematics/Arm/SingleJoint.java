package frc.robot.Libs.Kinematics.Arm;

public class SingleJoint {
    private double length;
    private double maxAngle;
    private double minAngle;

    public SingleJoint(double length, double maxAngle, double minAngle) {
        this.length = length;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }

    public double[] forwardKinematics(double angleRadians) {
        double[] position = new double[2];
        position[0] = length * Math.cos(angleRadians);
        position[1] = length * Math.sin(angleRadians);
        return position;
    }

    // Radians, returns an angle based of 2 points, even if the x,y is unreachable
    // That is just a limitation of this type of arm
    public double inverseKinematics(double[] position) {
        double angle = Math.atan2(position[1], position[0]);
        return angle;
    }

    // Getters
    public double getLength() {
        return length;
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public double getMinAngle() {
        return minAngle;
    }


    // Setters
    public void setLength(double length) {
        this.length = length;
    }

    public void setMaxAngle(double maxAngle) {
        this.maxAngle = maxAngle;
    }

    public void setMinAngle(double minAngle) {
        this.minAngle = minAngle;
    }
}
