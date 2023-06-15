package frc.robot.Libs.Kinematics.Arm;

public class TripleJoint {
    private double length1;
    private double length2;
    private double length3;

    private double maxAngle1;
    private double maxAngle2;
    private double maxAngle3;

    private double minAngle1;
    private double minAngle2;
    private double minAngle3;

    private DoubleJoint doubleJoint;

    public TripleJoint(double length1, double length2, double length3, 
                       double maxAngle1, double maxAngle2, double maxAngle3,
                       double minAngle1, double minAngle2, double minAngle3) {
        this.length1 = length1;
        this.length2 = length2;
        this.length3 = length3;

        this.maxAngle1 = maxAngle1;
        this.maxAngle2 = maxAngle2;
        this.maxAngle3 = maxAngle3;

        this.minAngle1 = minAngle1;
        this.minAngle2 = minAngle2;
        this.minAngle3 = minAngle3;

        doubleJoint = new DoubleJoint(length1, length2, maxAngle1, maxAngle2, minAngle1, minAngle2);
    }

    public double[] forwardKinematics(double angleRadians1, double angleRadians2, double angleRadians3) {
        double[] position = new double[2];
        double[] doubleJointPosition = doubleJoint.forwardKinematics(angleRadians1, angleRadians2);
        position[0] = doubleJointPosition[0] + length3 * Math.cos(angleRadians1 + angleRadians2 + angleRadians3);
        position[1] = doubleJointPosition[1] + length3 * Math.sin(angleRadians1 + angleRadians2 + angleRadians3);
        return position;
    }

    // Radians
    public double[] inverseKinematics (double x, double y, double thetaRadians) {
        double[] output = new double[3];

        double l1 = length1;
        double l2 = length2;
        double l3 = Math.hypot(x, y);
        double thetaA = Math.toDegrees(Math.acos((l2*l2-l1*l1-l3*l3)/(-2*l1*l3)));
        double thetaB = Math.toDegrees(Math.acos((l3*l3-l1*l1-l2*l2)/(-2*l1*l2)));

        output[0] = (360 + Math.toDegrees(Math.atan2(y, x)) + thetaA) % 360;
        output[1] = (360 + output[0] + thetaB + 180) % 360;
        output[2] = (360 + thetaRadians) % 360;

        return output;
    }

    // Getters
    public double getLength1() {
        return length1;
    }

    public double getLength2() {
        return length2;
    }

    public double getLength3() {
        return length3;
    }

    public double getMaxAngle1() {
        return maxAngle1;
    }

    public double getMaxAngle2() {
        return maxAngle2;
    }

    public double getMaxAngle3() {
        return maxAngle3;
    }


    public double getMinAngle1() {
        return minAngle1;
    }

    public double getMinAngle2() {
        return minAngle2;
    }

    public double getMinAngle3() {
        return minAngle3;
    }

    // Setters
    public void setLength1(double length1) {
        this.length1 = length1;
    }

    public void setLength2(double length2) {
        this.length2 = length2;
    }

    public void setLength3(double length3) {
        this.length3 = length3;
    }

    public void setMaxAngle1(double maxAngle1) {
        this.maxAngle1 = maxAngle1;
    }

    public void setMaxAngle2(double maxAngle2) {
        this.maxAngle2 = maxAngle2;
    }

    public void setMaxAngle3(double maxAngle3) {
        this.maxAngle3 = maxAngle3;
    }

    public void setMinAngle1(double minAngle1) {
        this.minAngle1 = minAngle1;
    }

    public void setMinAngle2(double minAngle2) {
        this.minAngle2 = minAngle2;
    }

    public void setMinAngle3(double minAngle3) {
        this.minAngle3 = minAngle3;
    }
}