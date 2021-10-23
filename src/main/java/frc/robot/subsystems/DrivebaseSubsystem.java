package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

/**
 * DrivebaseSubsystem
 */
public class DrivebaseSubsystem extends SubsystemBase {

	private static final double m_kGearRatio = 12.27;

	// The mag encoders are on the wheel-side so we don't need to multiply the EPR by the gear ratio here.
	private static final double m_kEdgesPerRotation = 2048 * m_kGearRatio;

	private static final double m_kWheelDiameterInInches = 7.50;
	private static final double m_kWheelCircumferenceInMeters = Units.inchesToMeters(m_kWheelDiameterInInches) * Math.PI;

    private static final double m_kEdgesToMetersAdjustment = (m_kWheelCircumferenceInMeters / m_kEdgesPerRotation);

	private static final double m_kTrackWidthInInches = 24.5; 
    private static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
		Units.inchesToMeters(m_kTrackWidthInInches));

	// Baseline values for a RAMSETE follower in units of meters and seconds
	private static final double m_kRamseteB = 2;
	private static final double m_kRamseteZeta = 0.7;

	// Voltage needed to overcome the motors static friction. kS 
	private static final double m_kS = 0.544; // From Robot Characterization Analysis

	// Voltage needed to hold (or "cruise") at a given constant velocity. kV
	private static final double m_kV = 2.19; // From Robot Characterization Analysis 

	// Voltage needed to induce a given acceleration in the motor shaft. kA 
	private static final double m_kA = 0.265; // From Robot Characterization Analysis

	// PID "Proportional" term from Robot Characterization Analysis. We don't need
	// this yet but if we use the more complex Ramsete constructor, it takes two
	// PIDControllers as input.
	private static final double m_pValue = 2.42; // From Characterization Analysis

	private static final int m_kDrivebaseTimeoutMs = 10;

	private static final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(m_kS, m_kV, m_kA);
	
	private static final double m_kMaxSpeedMetersPerSecond = 2.5;
	private static final double m_kMaxAccelerationMetersPerSecondSquared = 2.2;
	private static final double m_kDifferentialDriveConstraintMaxVoltage = 12.0;

	private WPI_TalonFX m_rightMaster = new WPI_TalonFX(Constants.RIGHT_FRONT_MOTOR_CAN_ID);
	private WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.RIGHT_REAR_MOTOR_CAN_ID);
	private WPI_TalonFX m_leftMaster = new WPI_TalonFX(Constants.LEFT_FRONT_MOTOR_CAN_ID);
	private WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.LEFT_REAR_MOTOR_CAN_ID);

	private boolean m_encodersAreAvailable;

	private DifferentialDrive m_differentialDrive;
	private DifferentialDriveOdometry m_odometry;

	private RamseteController m_ramseteController = new RamseteController(m_kRamseteB, m_kRamseteZeta);

	private Pose2d m_savedPose;

	private CT_Gyro m_gyro = new CT_Gyro(Constants.PIGEON_IMU_CAN_ID);

	private PIDController m_leftPIDController = new PIDController(m_pValue, 0, 0);
	private PIDController m_rightPIDController = new PIDController(m_pValue, 0, 0);

	private Pose2d m_initialPose = new Pose2d();

	private boolean m_allowDriving;

	public DrivebaseSubsystem() {

		// You need to register the subsystem to get it's periodic
		// method to be called by the Scheduler
		register();

		m_leftMaster.configFactoryDefault();
		m_rightMaster.configFactoryDefault();
		m_leftFollower.configFactoryDefault();
		m_rightFollower.configFactoryDefault();

		// Set Neutral Mode 
		m_leftMaster.setNeutralMode(NeutralMode.Brake);
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		m_leftFollower.setNeutralMode(NeutralMode.Brake);
		m_rightFollower.setNeutralMode(NeutralMode.Brake);

		enableEncoders();

		// Set open and closed loop values 
		m_leftMaster.configOpenloopRamp(0.5); // TODO - this doesn't do anything at 0. If we don't need it get rid of it.
		m_leftMaster.configClosedloopRamp(0.0); // TODO - this doesn't do anything at 0. If we don't need it get rid of it.

		m_rightMaster.configOpenloopRamp(0.5); // TODO - this doesn't do anything at 0. If we don't need it get rid of it.
		m_rightMaster.configClosedloopRamp(0.0); // TODO - this doesn't do anything at 0. If we don't need it get rid of it.

		m_leftMaster.setInverted(true);
		m_leftFollower.setInverted(true);

		m_rightMaster.setInverted(false);
        m_rightMaster.setInverted(false);

		m_leftMaster.setSensorPhase(false);
		m_leftFollower.setSensorPhase(false);

		m_rightMaster.setSensorPhase(false);
		m_rightFollower.setSensorPhase(false);

		m_leftFollower.follow(m_leftMaster);
		m_rightFollower.follow(m_rightMaster);

		// TODO - these methods aren't available or are different for TalonFX
		/*
		m_rightMaster.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
		m_rightFollower.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
		m_rightMaster.configPeakCurrentDuration(Constants.DRIVE_CURRENT_DURATION);
		m_rightFollower.configPeakCurrentDuration(Constants.DRIVE_CURRENT_DURATION);

		m_rightMaster.configContinuousCurrentLimit(Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT);
		m_rightFollower.configContinuousCurrentLimit(Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT);

		m_rightMaster.enableCurrentLimit(true);
		m_rightFollower.enableCurrentLimit(true);

		m_leftMaster.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
		m_leftFollower.configPeakCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
		m_leftMaster.configPeakCurrentDuration(Constants.DRIVE_CURRENT_DURATION);
		m_leftFollower.configPeakCurrentDuration(Constants.DRIVE_CURRENT_DURATION);

		m_leftMaster.configContinuousCurrentLimit(Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT);
		m_leftFollower.configContinuousCurrentLimit(Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT);

		m_leftMaster.enableCurrentLimit(true);
		m_leftFollower.enableCurrentLimit(true);
		*/

		m_differentialDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
		m_differentialDrive.setDeadband(0.01);

		m_gyro.resetYaw();

		m_odometry = new DifferentialDriveOdometry(m_gyro.getHeading());
		resetOdometry(new Pose2d());

		m_allowDriving = true;
	}

	/* Zero all sensors used */
	public void zeroSensors() {
		m_leftMaster.setSelectedSensorPosition(0, Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs);
		m_rightMaster.setSelectedSensorPosition(0, Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs);
		m_leftFollower.setSelectedSensorPosition(0, Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs);
		m_rightFollower.setSelectedSensorPosition(0, Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs);
		System.out.println("All sensors are zeroed.\n");
	}

	/**
	 * Enables the encoders
	 * 
	 * Reports an error to the drive station and prints out a message if the encoders are not available
	 */
	private void enableEncoders() {
		m_encodersAreAvailable = 
		((m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 
		Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs) == ErrorCode.OK) &&
		  (m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 
		  Constants.PID_PRIMARY, m_kDrivebaseTimeoutMs) == ErrorCode.OK));

		if (!m_encodersAreAvailable) {
			DriverStation.reportError("Failed to configure TalonSRX encoders!!", false);
			System.out.println("Encoders aren't available");
		}
	}
	
	private double deadband(final double value) {
		/* Upper deadband */
		if (value >= 0.1)
			return value;

		/* Lower deadband */
		if (value <= -0.1)
			return value;

		/* Inside deadband */
		return 0.0;
	}

	/**
	 * Arcade drive that takes in the left joystick for forward and the right joystick for turn
	 * 
	 * Uses the deadband and the PercentOutput control mode
	 */
	// private void arcadeDrive() {
	// 	double forward = OI.getXboxLeftJoystickY();
	// 	double turn = OI.getXboxRightJoystickX(); 

	// 	forward = deadband(forward);
	// 	turn = deadband(turn) * 0.3; // Where did .65 come from??????

	// 	m_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn); 
	// 	m_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
	// }

	// Only used by ShooterSubsystem in teleop
	public double getCurrentMoveSpeedAverage() {
		return (m_leftMaster.get() + m_rightMaster.get()) / 2;
	}

	@Override
	public void periodic() {
		if (RobotState.isAutonomous()) {
			//System.out.println("LeftMasterEncoderValue: " + -m_leftMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
			//System.out.println("RightMasterEncoderValue: " + -m_rightMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));

			m_savedPose = m_odometry.update(m_gyro.getHeading(), 
			-m_leftMaster.getSelectedSensorPosition(Constants.PID_PRIMARY) * m_kEdgesToMetersAdjustment, 
			-m_rightMaster.getSelectedSensorPosition(Constants.PID_PRIMARY) * m_kEdgesToMetersAdjustment);
		}
		else {
			if(m_allowDriving) {
				m_differentialDrive.curvatureDrive(-OI.getXboxRightJoystickX() * 0.4, OI.getXboxLeftJoystickY(), true);
			} else {
				if(Math.abs(OI.getXboxRightJoystickX()) > 0.1 || Math.abs(OI.getXboxLeftJoystickY()) > 0.1) {
					System.out.println("Rumbling because driver is trying to drive when the robot is auto adjusting");
					RobotContainer.getRumbleCommand(0.5).schedule();
				}
			}
			
		}

		m_differentialDrive.feed();
	}


	public void allowDriving(boolean allowDriving) {
		m_allowDriving = allowDriving;
	}

	public void setOutputVolts(double leftVolts, double rightVolts) {
		//System.out.println("leftVolts: " + leftVolts);
		//System.out.println("rightVolts: " + rightVolts);
		m_leftMaster.set(-leftVolts / 12.0);
		m_rightMaster.set(-rightVolts / 12.0);
	}

	/**
	 * Stops the drivetrain
	 */
	public void stop() {
		m_rightMaster.stopMotor();
		m_leftMaster.stopMotor();
	}

	public DifferentialDriveWheelSpeeds getSpeeds() {
	   return new DifferentialDriveWheelSpeeds(
		 -m_leftMaster.getSelectedSensorVelocity(Constants.PID_PRIMARY) * 10.0 * m_kEdgesToMetersAdjustment,
		 -m_rightMaster.getSelectedSensorVelocity(Constants.PID_PRIMARY) * 10.0 * m_kEdgesToMetersAdjustment
		 );
    }

	/**
	 * @return boolean
	 */
	public boolean areEncodersAvailable() {
		return m_encodersAreAvailable;
	}

	/**
	 * @return Pose2d
	 */
	public Pose2d getCurrentPose() {
		return m_savedPose;
	}

	/**
	 * Gets the Rameste Controller
	 */
	public RamseteController getRamseteController() {
		return m_ramseteController;
	}

	public DifferentialDriveKinematics getDriveKinematics() {
		return m_driveKinematics;
	}

	public SimpleMotorFeedforward getFeedforward() {
		return m_feedForward;
	}
	
	public double getMaxSpeed() {
		return m_kMaxSpeedMetersPerSecond;
	}

	public double getMaxAcceleration() {
		return m_kMaxAccelerationMetersPerSecondSquared;
	}

	public double getMaxVoltage() {
		return m_kDifferentialDriveConstraintMaxVoltage;
	}

	public PIDController getLeftPIDController() {
		return m_leftPIDController;
	}

	public PIDController getRightPIDController() {
		return m_rightPIDController;
	}

	public void setCoastMode() {
		m_leftMaster.setNeutralMode(NeutralMode.Coast);
		m_rightMaster.setNeutralMode(NeutralMode.Coast);
		m_leftFollower.setNeutralMode(NeutralMode.Coast);
		m_rightFollower.setNeutralMode(NeutralMode.Coast);
	}

	public void setBrakeMode() {
		m_leftMaster.setNeutralMode(NeutralMode.Brake);
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		m_leftFollower.setNeutralMode(NeutralMode.Brake);
		m_rightFollower.setNeutralMode(NeutralMode.Brake);
	}

	public CT_Gyro getGyro() {
		return m_gyro;
	}

	public void resetOdometry(Pose2d pose) {
		zeroSensors();
		resetHeading();
		m_odometry.resetPosition(pose, m_gyro.getHeading());
	}

	public void resetHeading() {
		m_gyro.resetYaw();
	}

	public void setInitialPose(Pose2d pose){
		m_initialPose = pose;
	}

	public void turnLeft(double speed){
		m_rightMaster.set(0);
		m_leftMaster.set(speed);
	}

	public void turnRight(double speed){
		m_leftMaster.set(0);
		m_rightMaster.set(speed);
	}

	public void turnLeftOnAxis(double speed) {
		m_leftMaster.set(speed);
		m_rightMaster.set(-speed);
	}

	public void turnRightOnAxis(double speed) {
		m_rightMaster.set(speed);
		m_leftMaster.set(-speed);
	}
}