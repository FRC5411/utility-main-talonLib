package frc.robot.Libs.Kinematics.Elevator;

public class SingleJoint {
    private double maxHeightMeter;
    private double minHeightMeter;
    private double appendLengthMeter;
    private double appendAngleRadians;

    // Constructor, this class is a single jointed elevator with some sort of appendage, 
    // such as an arm or angled piece of metal
    public SingleJoint(double maxHeightMeter, double minHeightMeter, double appendLengthMeter, double appendAngleRadians) {
        this.maxHeightMeter = maxHeightMeter;
        this.minHeightMeter = minHeightMeter;
        this.appendLengthMeter = appendLengthMeter;
        this.appendAngleRadians = appendAngleRadians;
    }

    // Returns the y position the elevator needs to go to based of joint details
    public double forwardKinematics(double yMeters) {
        double position = yMeters - (appendLengthMeter * Math.sin(appendAngleRadians));

        position = Math.max(Math.min(position, maxHeightMeter), minHeightMeter);

        return position - yMeters;
    }

    // Same kinematics as above, but for dynamically changing joints(moving arm)
    public double forwardKinematics(double yMeters, double appendLengthMeter, double appendAngleRadians) {
        double position = yMeters - (appendLengthMeter * Math.sin(appendAngleRadians));

        position = Math.max(Math.min(position, maxHeightMeter), minHeightMeter);

        return position - yMeters;
    }

    // Returns x, y position of the joint
    public double[] inverseKinematics(double heightMeters) {
        double[] position = new double[2];

        position[0] = appendLengthMeter * Math.cos(appendAngleRadians);

        position[1] = heightMeters + (appendLengthMeter * Math.sin(appendAngleRadians));

        return position;
    }

    // Returns x, y position of the joint for dynamically changing joints(moving arm)
    public double[] inverseKinematics(double heightMeters, double appendLengthMeter, double appendAngleRadians) {
        double[] position = new double[2];

        position[0] = appendLengthMeter * Math.cos(appendAngleRadians);

        position[1] = heightMeters + (appendLengthMeter * Math.sin(appendAngleRadians));

        return position;
    }
}