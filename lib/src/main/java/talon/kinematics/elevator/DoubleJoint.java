package talon.kinematics.elevator;

public class DoubleJoint {
    private SingleJoint joint1;
    private SingleJoint joint2;

    // An elevator with 2 joints, acheiving complete x, y control
    public DoubleJoint(SingleJoint joint1, SingleJoint joint2) {
        this.joint1 = joint1;
        this.joint2 = joint2;
    }

    public double[] forwardKinematics(double x, double y) {
        double Goal1 = joint1.FKXwithAppendage(x, joint2.getMaxLengthMeter(), joint2.getTiltRadians());
        double Goal2 = joint2.forwardKinematicsY(y);

        double[] output = {Goal1, Goal2};
        return output;
    }

    public double[] FKXwithAppendage(double x, double y, double appendageLengthMeters, double appendageAngleRadians) {
        double Goal1 = joint1.FKXwithAppendage(joint2.getMaxLengthMeter(), joint2.getMaxLengthMeter(), joint2.getTiltRadians());
        double Goal2 = joint2.FKYwithAppendage(y, appendageLengthMeters, appendageAngleRadians);

        double[] output = {Goal1, Goal2};
        return output;
    }

    public double[] inverseKinematics(double x, double y) {
        double[] output = new double[2];
        output[0] = joint1.inverseKinematics(Math.hypot(x, y))[0];
        output[1] = joint2.inverseKinematics(Math.hypot(x, y))[1];
        return output;
    }
}