package talon.kinematics.elevator;

public class SingleJoint {
    private double maxLengthMeter;
    private double tiltRadians;

    // For tilt radians, the angle of the joint from the horizontal, so degrees would be on the ground
    // and 90 degrees would be straight up
    public SingleJoint(double maxLengthMeter, double tiltRadians) {
        this.maxLengthMeter = maxLengthMeter;
        this.tiltRadians = tiltRadians;
    }

    public double forwardKinematicsX(double xMeters) {
        double position = xMeters / Math.cos(tiltRadians);

        return position;
    }

    public double FKXwithAppendage(double xMeters, double appendLengthMeters, double appendAngleRadians) {
        double position = (xMeters -
                    (appendLengthMeters * Math.cos(appendAngleRadians))) 
                    / Math.cos(tiltRadians) ;

        return position;
    }

    public double forwardKinematicsY(double yMeters) {
        double position = yMeters / Math.sin(tiltRadians);

        return position;
    }

    public double FKYwithAppendage(double yMeters, double appendLengthMeters, double appendAngleRadians) {
        double position = (yMeters -
                    (appendLengthMeters * Math.sin(appendAngleRadians))) 
                    / Math.sin(tiltRadians) ;

        return position;
    }

    public double[] inverseKinematics(double hypotMeters) {
        double[] position = {hypotMeters * Math.sin(tiltRadians), 
                            hypotMeters * Math.cos(tiltRadians)};

        return position;
    }

    public double[] IKwithAppendage(double hypotMeters, double appendLengthMeters, double appendAngleRadians) {
        double[] position = {hypotMeters * Math.sin(tiltRadians) + (appendLengthMeters * Math.sin(appendAngleRadians)), 
                            hypotMeters * Math.cos(tiltRadians) + (appendLengthMeters * Math.cos(appendAngleRadians))};

        return position;
    }

    // Getters
    public double getMaxLengthMeter() {
        return maxLengthMeter;
    }

    public double getTiltRadians() {
        return tiltRadians;
    }

    // Setters
    public void setMaxLengthMeter(double maxLengthMeter) {
        this.maxLengthMeter = maxLengthMeter;
    }

    public void setTiltRadians(double tiltRadians) {
        this.tiltRadians = tiltRadians;
    }
}