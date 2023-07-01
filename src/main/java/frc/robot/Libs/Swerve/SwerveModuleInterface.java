package frc.robot.Libs.Swerve;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleInterface {
    // Drive Commands
    public void setDesiredState(SwerveModuleState modules, boolean openLoop);
    public void setDriveMPS(SwerveModuleState state, boolean openLoop);
    public void setAngleDegrees(SwerveModuleState state);

    // Encoder fetches
    public CANCoder getEncoder();

    // Odometry fetches
    public double getDriveMeters();
    public Rotation2d getAngleRads();
    public double getAnshulFactor();

    // Resets
    public void resetToAbsolute();
    public void resetToZero();
}