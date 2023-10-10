package frc.robot.Libs;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Configs {
    public static CANSparkMax NEO550(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(20);
        motor.setSecondaryCurrentLimit(20);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static CANSparkMax NEO(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static DutyCycleEncoder AbsEncbore(DutyCycleEncoder encoder, int port, double conversionFactor) {
        encoder = new DutyCycleEncoder(port);
        encoder.setDistancePerRotation(conversionFactor);
        return encoder;
    }

    public static Encoder relativeEncbore(Encoder encoder, int port, int port2, double conversionFactor) {
        encoder = new Encoder(port, port2);
        encoder.setDistancePerPulse(conversionFactor);
        return encoder;
    }

    public static CANCoder configPosition (CANCoder encoder, double offset) {
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.setPositionToAbsolute();
        return encoder;
    }

    public static WPI_TalonFX configDrive(WPI_TalonFX motor, double DRIVE_kP, double DRIVE_kF) {
        StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
            true, 
            60, 
            60, 
            0
        );
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
        motor.config_kP(0, DRIVE_kP);
        motor.config_kF(0, DRIVE_kF);
        return motor;
    }

    public static WPI_TalonFX configAzimuth (WPI_TalonFX motor, CANCoder position, double AZIMUTH_kP, double AZIMUTH_kD) {
        StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = 
            new StatorCurrentLimitConfiguration(
                true, 
                20, 
                20, 
                0
            );
        
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configRemoteFeedbackFilter(position, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
        motor.setSelectedSensorPosition(position.getAbsolutePosition());
        motor.config_kP(0, AZIMUTH_kP);
        motor.config_kD(0, AZIMUTH_kD);

        return motor;
    }
}