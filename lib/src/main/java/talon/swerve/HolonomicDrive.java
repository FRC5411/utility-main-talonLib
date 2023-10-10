package talon.swerve;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolonomicDrive {
    private Pigeon2 gyro;
    private SwerveModuleInterface modules[];
    private SwerveDriveKinematics kinematics;
    private double maxSpeed;
    private boolean invertGyro;

    // Constructs a holonomic(Swerve) drive using my swerve module interface, gyro, kinematics and choosen max speed
    // Works similar to Differential drive class from wpilib, I named it Holonomic drive to sound more interesting than swerve
    // Does not work with mecanum drive newbies
    public HolonomicDrive(SwerveModuleInterface[] modules, Pigeon2 gyro,
    SwerveDriveKinematics kinematics, double maxSpeed) {
        invertGyro = false;
        this.gyro = gyro;

        this.kinematics = kinematics;

        this.modules = modules;

        this.maxSpeed = maxSpeed;
    }

    // Function use in teleop mode
    // The math is directly copy pasted from 364's code as its been tested math wise and works
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        // Debug info on the module goals in speed and degrees(Not optimized)
        for(int i = 0; i <= modules.length - 1; i++) {
            SmartDashboard.putNumber("Module " + i + " Angle", swerveModuleStates[i].angle.getDegrees());
            SmartDashboard.putNumber("Module " + i + " Speed", swerveModuleStates[i].speedMetersPerSecond);
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }    

    // Function use in autonomous mode
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for(int i = 0; i < modules.length - 1; i++) {
            modules[i].setDesiredState(desiredStates[i], false);
        }
    }

    // Not used in 2023 season but nice to have for the future
    public void xLock() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        swerveModuleStates[0] = new SwerveModuleState(0, new Rotation2d(315));
        swerveModuleStates[1] = new SwerveModuleState(0, new Rotation2d(45));
        swerveModuleStates[2] = new SwerveModuleState(0, new Rotation2d(225));
        swerveModuleStates[3] = new SwerveModuleState(0, new Rotation2d(135));

        for(int i = 0; i < modules.length - 1; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], false);
        }
    }

    // Invert logic worked in sim
    public void invertGyro(boolean invert) {
        invertGyro = invert;
    }

    // Not tested, but math wise should work
    public Rotation2d getYaw() {
        return invertGyro ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // Get functions mainly for use with swerve utils
    public Pigeon2 getGyro() {
        return gyro;
    }

    public SwerveModuleInterface[] getModules() {
        return modules;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}