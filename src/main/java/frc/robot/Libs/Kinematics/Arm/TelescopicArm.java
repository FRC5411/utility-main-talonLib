package frc.robot.Libs.Kinematics.Arm;

public class TelescopicArm {
    private double maxLengthMeters;

    public TelescopicArm(double maxLengthMeters) {
        this.maxLengthMeters = maxLengthMeters;
    }

    public double[] forwardKinematics(double x, double y) {
        double armX = x;
        double armY = y;
    
        double thetaRadians = Math.atan2(armY, armX);
        double magnitude = Math.hypot(armX, armY);
    
        double[] polarVals = {magnitude, thetaRadians};
    
        if(magnitude > maxLengthMeters) {
          polarVals[0] = maxLengthMeters;
        }
    
        return polarVals;
    }

    public double[] inverseKinematics(double magnitude, double thetaRadians) {
        double[] output = new double[2];
        output[0] = magnitude * Math.cos(thetaRadians);
        output[1] = magnitude * Math.sin(thetaRadians);
        return output;
    }
}
