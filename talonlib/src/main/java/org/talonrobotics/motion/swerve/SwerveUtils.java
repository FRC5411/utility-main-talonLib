package talon.motion.swerve;
import java.util.HashMap;
import java.util.List;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveUtils {
    public SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator odometry;
    public SwerveModuleInterface[] modules;
    public SwerveModulePosition[] positions;
    public SwerveAutoBuilder builder;
    public PIDConstants tranPID;
    public PIDConstants rotPID;
    public HashMap <String, Commands> eventMap;
    public Pigeon2 gyro;
    public HolonomicDrive drive;

    // Utiliites class can do many swerve features, mainly pathplanner and odometry
    public SwerveUtils( PIDConstants tranPID, PIDConstants rotPID, HolonomicDrive drive ) {
        this.modules = drive.getModules();
        this.kinematics = drive.getKinematics();

        this.positions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
        };

        this.tranPID = tranPID;
        this.rotPID = rotPID;

        this.gyro = drive.getGyro();

        this.drive = drive;

        this.odometry = createOdometry();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    // Creates a swerve drive pose estimator at 0,0 Pathplanner should set the pose after this and has done so in simulation
    public SwerveDrivePoseEstimator createOdometry() {
        return new SwerveDrivePoseEstimator(
            kinematics,
            getRotation2d(),
            positions,
            new Pose2d()
        );
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), positions, pose);
    }


    // No need for input pulls straignt from passed in odules
    public void updateOdometry() {
        for(int i = 0; i <= positions.length - 1; i++) {
            double pos = modules[i].getDriveMeters() * modules[i].getAnshulFactor();
            positions[i] = new SwerveModulePosition(pos, modules[i].getAngleRads());
            i++;
        }
        odometry.update(getRotation2d(), positions);
    }

    // For use with vision, remember to have an if condition to only run when an apriltag is visible
    public void updateOdometry(Pose2d vision) {
        odometry.addVisionMeasurement(vision, Timer.getFPGATimestamp());
        updateOdometry();
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    // For following generated non - inline paths
    public Command followPath(String path, HashMap<String, Command> eventMap, 
                                boolean isBlue, Subsystem requirements) {

        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(path, PathPlanner.getConstraintsFromPath(path));

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            this::getPose,
            this::resetOdometry,
            kinematics,
            tranPID,
            rotPID,
            drive::setModuleStates,
            eventMap,
            isBlue,
            requirements);

        return builder.fullAuto(trajectory);
    }

    // For use with april tag alignment or such endeavors
    // Most use sequantial commands and such since you can't have event markers on generated paths
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isBlue, Subsystem requirements) {
        return 
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, 
                kinematics,
                new PIDController(tranPID.kP, tranPID.kI, tranPID.kD), 
                new PIDController(tranPID.kP, tranPID.kI, tranPID.kD),
                new PIDController(rotPID.kP, rotPID.kI, rotPID.kD),
                drive::setModuleStates,
                isBlue,
                requirements
             );
     }

    // Creates swerve drive kinematics based of the wheel base and track width
    public static SwerveDriveKinematics createKinematics(double wheelBaseMeters, double trackWidthMeters) {
        return new SwerveDriveKinematics(
            new Translation2d(wheelBaseMeters/2, trackWidthMeters/2),
            new Translation2d(wheelBaseMeters/2, -trackWidthMeters/2),
            new Translation2d(-wheelBaseMeters/2, trackWidthMeters/2),
            new Translation2d(-wheelBaseMeters/2, -trackWidthMeters/2)
          );
    }

    public static SwerveDriveKinematics createSquareKinematics(double robotWidthMeters) {
        return createKinematics(robotWidthMeters, robotWidthMeters);
    }
}