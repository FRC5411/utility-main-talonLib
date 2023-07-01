package frc.robot.Libs.Kinematics.Arm;

public class DoubleJoint {
    private double length1;
    private double length2;

    private double maxAngle1;
    private double maxAngle2;

    private double minAngle1;
    private double minAngle2;

    public DoubleJoint(double length1, double length2, double maxAngle1, double maxAngle2, double minAngle1, double minAngle2) {
        this.length1 = length1;
        this.length2 = length2;

        this.maxAngle1 = maxAngle1;
        this.maxAngle2 = maxAngle2;

        this.minAngle1 = minAngle1;
        this.minAngle2 = minAngle2;
    }

    public double[] forwardKinematics(double angleRadians1, double angleRadians2) {
        double[] position = new double[2];
        position[0] = length1 * Math.cos(angleRadians1) + length2 * Math.cos(angleRadians1 + angleRadians2);
        position[1] = length1 * Math.sin(angleRadians1) + length2 * Math.sin(angleRadians1 + angleRadians2);
        return position;
    }

    // Radians
    public double[] inverseKinematics (double x, double y) {
            double[] output = new double[3];

            double l1 = length1;
            double l2 = length2;
            double l3 = Math.hypot(x, y);

            double thetaA = Math.toDegrees(Math.acos((l2*l2-l1*l1-l3*l3)/(-2*l1*l3)));
            double thetaB = Math.toDegrees(Math.acos((l3*l3-l1*l1-l2*l2)/(-2*l1*l2)));

            output[0] = (360 + Math.toDegrees(Math.atan2(y, x)) + thetaA) % 360;
            output[1] = (360 + output[0] + thetaB + 180) % 360;

        return output;
    }

    // Getters
    public double getLength1() {
        return length1;
    }

    public double getLength2() {
        return length2;
    }

    public double getMaxAngle1() {
        return maxAngle1;
    }

    public double getMaxAngle2() {
        return maxAngle2;
    }

    public double getMinAngle1() {
        return minAngle1;
    }

    public double getMinAngle2() {
        return minAngle2;
    }

    // Setters
    public void setLength1(double length1) {
        this.length1 = length1;
    }

    public void setLength2(double length2) {
        this.length2 = length2;
    }

    public void setMaxAngle1(double maxAngle1) {
        this.maxAngle1 = maxAngle1;
    }

    public void setMaxAngle2(double maxAngle2) {
        this.maxAngle2 = maxAngle2;
    }

    public void setMinAngle1(double minAngle1) {
        this.minAngle1 = minAngle1;
    }

    public void setMinAngle2(double minAngle2) {
        this.minAngle2 = minAngle2;
    }
}