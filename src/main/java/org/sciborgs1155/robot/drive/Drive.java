package org.sciborgs1155.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {

    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);
    private final DifferentialDriveOdometry odometry;

    //kS term accoutns for constant voltage to overcome friction
    //kV accounts for the voltage to maintain a certain velocity
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);

    private final PIDController leftPIDController =
    new PIDController(PID.kP, PID.kI, PID.kD);

    private final PIDController rightPIDController =
    new PIDController(PID.kP, PID.kI, PID.kD);

    private final DifferentialDrivetrainSim driveSim;
    // start
    public Drive() {
        for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
            spark.restoreFactoryDefaults();
            spark.setIdleMode(IdleMode.kBrake);
        }

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftLeader.setInverted(true);

        leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);

        leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        gyro.reset();

        odometry = new DifferentialDriveOdometry(
          new Rotation2d(), 
          0, 
          0, 
          new Pose2d());
        
        driveSim =
          new DifferentialDrivetrainSim(
            DCMotor.getMiniCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.DRIVE_MASS,
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            DriveConstants.STD_DEVS);
        
    }

    private void updateOdometry(Rotation2d rotation) {
      odometry.update(rotation, leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public void resetOdometry(Pose2d pose) {
      odometry.resetPosition(pose, gyro.getRotation2d());
    }
    
    public Pose2d pose() {
      return odometry.getPoseMeters();
    }

    @Override 
    public void periodic() {
      updateOdometry(Robot.isReal() ? gyro.getRotation2d() :  
							        driveSim.getHeading());
    }

    @Override
    public void simulationPeriodic() {
      // sim.update() tells the simulation how much time has passed
      driveSim.update(Constants.PERIOD.in(Seconds));
      leftEncoder.setPosition(driveSim.getLeftPositionMeters());
      rightEncoder.setPosition(driveSim.getRightPositionMeters());
    }
    
    private void drive(double leftSpeed, double rightSpeed) {
        leftLeader.set(leftSpeed);
        rightLeader.set(rightSpeed);
    }

    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }

    public void drive(double leftSpeed, double rightSpeed) {
      final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
      final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
      
      final double leftFeedforward = feedforward.calculate(realLeftSpeed);
      final double rightFeedforward = feedforward.calculate(realRightSpeed);
    
      final double leftPID = 
        leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
      final double rightPID = 
        rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

        double leftVoltage = leftPID + leftFeedforward;
        double rightVoltage = rightPID + rightFeedforward;
    
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);

        driveSim.setInputs(leftVoltage, rightVoltage);
      }

}
