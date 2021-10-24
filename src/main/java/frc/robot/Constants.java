/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int CONTROL_PANEL_INTERRUPT_DIO = 0;
	public static final int ACQUIRE_BALL_DIO = 2;
	public static final int SHOOTER_BALL_DIO = 3;
	public static final int SHOOTER_POSITION_DIO = 4;
	public static final int CLIMBER_UPPER_PROX_DIO = 5;
	public static final int CLIMBER_LOWER_PROX_DIO = 6;
	public static final int DIGITAL_IO_7 = 7;
	public static final int DIGITAL_IO_8 = 8;
	public static final int DIGITAL_IO_9 = 9;

	public static final int ANALOG_INPUT_0 = 0;
	public static final int ANALOG_INPUT_1 = 1;
	public static final int ANALOG_INPUT_2 = 2;
	public static final int ANALOG_INPUT_3 = 3;

	public static final int BOPPER_PCM_PORT = 0;
	public static final int ACQUIRER_DEPLOY_PCM_PORT = 2;
	public static final int CONTROL_PANEL_DEPLOY_PCM_PORT = 5;
	public static final int PCM_PORT_6 = 6;
	public static final int PCM_PORT_7 = 7;
	public static final int PCM_CAN_ID = 0;

	public static final int PWM_PIN_0 = 0;
	public static final int PWM_PIN_1 = 1;
	public static final int PWM_PIN_2 = 2;
	public static final int PWM_PIN_3 = 3;
	public static final int PWM_PIN_4 = 4;
	public static final int PWM_PIN_5 = 5;
	public static final int PWM_PIN_6 = 6;
	public static final int PWM_PIN_7 = 7;
	public static final int PWM_PIN_8 = 8;
	public static final int PWM_PIN_9 = 9;

	public static final int CONTROL_PANEL_MOTOR_CAN_ID = 31;
	public static final int RIGHT_FRONT_MOTOR_CAN_ID = 11;
	public static final int RIGHT_REAR_MOTOR_CAN_ID = 12;
	public static final int LEFT_FRONT_MOTOR_CAN_ID = 13;
	public static final int LEFT_REAR_MOTOR_CAN_ID = 14;
	public static final int SPINNING_BAR_MOTOR_CAN_ID = 21;

	public static final int CLIMBING_NEO_SPARK_MAX_CAN_ID = 42;

	public static final int PIGEON_IMU_CAN_ID = 61;
	public static final int CANIFIER_CAN_ID = 62;

	public static final int LEFT_DISTANCE_SENSOR_ID = 1;
	public static final int RIGHT_DISTANCE_SENSOR_ID = 2;

	public static final int RELAY_PIN_0 = 0;
	public static final int RELAY_PIN_1 = 1;
	public static final int RELAY_PIN_2 = 2;
	public static final int RELAY_PIN_3 = 3;

	public static final double VISION_TARGET_TOLERANCE_IN_INCHES = 1.0;

	public static final double ARCADE_DRIVE_TURN_DEADBAND = .03;

	public static final double XBOX_RUMBLE_SPEED = 1.0;

	public static final double WHEEL_MOTOR_VELOCITY = 0.1;

	public static final int XBOX_RUMBLE_COMMAND_TIMEOUT = 1;

	public static final double CONTROL_PANEL_MOTOR_VELOCITY_FAST = 0.27;
	public static final double CONTROL_PANEL_MOTOR_VELOCITY_SLOW = 0.2;

	public static final double DRUM_MOTOR_VELOCITY_VERY_SLOW = 0.25;
	public static final double DRUM_MOTOR_VELOCITY_SLOW = 0.35; 
	public static final double DRUM_MOTOR_VELOCITY_FAST = 0.45;

	public static final int SHOOT_MODE_SINGLE_CELL = 0;
	public static final int SHOOT_MODE_ALL_CELLS = 1;

	public static final int SHOOTER_SLOT = 2;

	public static final int SHOOTER_CAN_ID = 41;
	public static final int DRUM_SPARK_PWM_ID = 0;

	public static final int LOOPS_TO_WAIT = 10;
	public static final double SHOOTER_MOTOR_SPEED = 120000.0;
	public static final int MIN_SHOOTING_DISTANCE = 61;
	public static final int MAX_SHOOTING_DISTANCE = 0;

	public static final double TIME_BETWEEN_SHOTS = 1.25; 
	public static final double BOPPER_WAIT_TIME = 0.2;

	public static final int DRIVE_CURRENT_LIMIT = 60;
	public static final int DRIVE_CURRENT_DURATION = 100;
	public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;

	public static final int ACQUIRE_CURRENT_LIMIT = 30;
	public static final int ACQUIRE_CURRENT_DURATION = 100;
	public static final int ACQUIRE_CONTINUOUS_CURRENT_LIMIT = 18;

	public static final int SHOOTER_CURRENT_LIMIT = 40;
	public static final int SHOOTER_CURRENT_DURATION = 100;
	public static final int SHOOTER_CONTINUOUS_CURRENT_LIMIT = 35;

	public static final int ACQUIRER_CURRENT_THRESHOLD = 2;
	
	public static final double ELEVATOR_DEPLOY_SPEED_LOWER = 0.5;

	public static final double TC_UPPER_MAGNITUDE_BOUND = 480.0;
	public static final double TC_LOWER_MAGNITUDE_BOUND = 5;
	public static final double TC_TURN_FAST_SPEED = 0.1;
	public static final double TC_TURN_SLOW_SPEED = 0.075; // .075

	/**
	 * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;

	/** ---- Flat constants, you should not need to change these ---- */
	/*
	 * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
	 * can have up to 2 devices assigned remotely to a talon/victor]
	 */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/*
	 * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
	 * is auxiliary
	 */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/*
	 * Firmware currently supports slots [0, 3] and can be used for either PID Set
	 */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

	public static final double ACQUIRER_MOTOR_SPEED = 0.70;

	public static final int IDLE_STATE = 0; // Not acquiring or shooting
	public static final int ACQUIRING_STATE = 1;
	public static final int SHOOTING_STATE = 2;
	public static final int TEST_STATE = 3;

	public static final double TIME_TO_WAIT_FOR_DIAL_SLOP = 0.1;
}
