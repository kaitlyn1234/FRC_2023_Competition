// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// TO DO:
// Copy PID tunings from arm pivot to grabber pivot to accomodate new motor
// Change grabber pivot gear ratio
// add deadband to stop linear movement when charge station angle <5 degrees
// move charge station auto-level autonomous to autonomous section
// merge with main & simple timed drive autonomous
// set up autonomous mode with no action

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup; 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Charging Station";
  private static final String kCustomAuto = "Leave Community";
  private static final String kCustomAuto2 = "Nothing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// DRIVE TRAIN MOTORS
  WPI_TalonSRX left_motor_front = new WPI_TalonSRX(4);
  WPI_TalonSRX right_motor_front = new WPI_TalonSRX(3);
  WPI_VictorSPX right_motor_back = new WPI_VictorSPX(1);
  WPI_VictorSPX left_motor_back = new WPI_VictorSPX(2);

// EXTENSION, PIVOT, ETC. MOTORS
  CANSparkMax right_lift_motor = new CANSparkMax(5, MotorType.kBrushless);;
  CANSparkMax left_lift_motor = new CANSparkMax(6, MotorType.kBrushless);;
  CANSparkMax extension = new CANSparkMax(7, MotorType.kBrushless);;
  CANSparkMax grabber_pivot = new CANSparkMax(8, MotorType.kBrushless);;
  CANSparkMax grabber_arms = new CANSparkMax(9, MotorType.kBrushless);;

  PIDController extension_vel_pid = new PIDController(0.15, 0.25, 0.0);
  PIDController lift_pivot_group_vel_pid = new PIDController(1.1, 2.0, 0.0);
  PIDController grabber_pivot_vel_pid = new PIDController(1.1, 1.5, 0.0);
 // original    PIDController grabber_pivot_vel_pid = new PIDController(0.5, 0.5, 0.0);

  // Control velocity of drivetrain wheels
  PIDController ang_drivetrain_vel_pid = new PIDController(0.1, 1, 0.0);
  PIDController lin_drivetrain_vel_pid = new PIDController(0.00025, 0.0008, 0.0);

  final double ticks_per_meter = 447.388; // 1/( (1/10.71) * 2 * Math.PI * 0.0762 * (1/20));
  final double wheel_base_width = 0.562; // Distance between the left and right wheels

  // Control yaw of the robot
  PIDController drivetrain_yaw_pos_pid = new PIDController(0.5, 0.0, 0.0);

  // Positional PID used for charge station alignment
  PIDController drivetrain_leveling_pid = new PIDController(0.10, 0.00, 0.0);

  Timer drive_up_timer = new Timer();
  Timer autonomous_timer = new Timer();
  final double AUTO_DRIVE_UP_TIME = 7;
  final double AUTO_DRIVE_UP_VEL = 0.75;

  final double AUTO_LEVEL_MAX_LIN_VEL = 0.25;
  final double AUTO_LEVEL_MAX_ANG_VEL = 1.25;
  final double AUTO_LEVEL_DEADBAND_ANG = 6.5; // no deadband for now

  private final MotorControllerGroup right_Motor_Group = new MotorControllerGroup(right_motor_front, right_motor_back);
  private final MotorControllerGroup left_Motor_Group = new MotorControllerGroup(left_motor_front, left_motor_back);
  private final MotorControllerGroup lift_pivot_group = new MotorControllerGroup(right_lift_motor, left_lift_motor);
  DifferentialDrive differential_drive = new DifferentialDrive(left_Motor_Group, right_Motor_Group);

  Joystick logitechController = new Joystick(0);
  Joystick stick = new Joystick(1);

  final double VELOCITY_CALCULATION_DT = 0.1;
  final double ANG_VELOCITY_CALCULATION_DT = 0.02;

  double yaw_vel = 0;
  double prev_yaw = 0;

  double left_drivetrain_prev_pos = 0;
  double right_drivetrain_prev_pos = 0;

  double left_drivetrain_vel = 0;
  double right_drivetrain_vel = 0;

  final int LIFT_BUTTON_DOWN = 1;
  final int LIFT_BUTTON_UP = 2;
  final int EXTENSION_BUTTON_IN = 3;
  final int EXTENSION_BUTTON_OUT = 4;
  final int GRABBER_ARMS_BUTTON_OUT = 5;
  final int GRABBER_ARMS_BUTTON_IN = 6;
  final int GRABBER_PIVOT_BUTTON_DOWN = 7;
  final int GRABBER_PIVOT_BUTTON_UP = 8;

  final double grabber_pivot_max_setpoint = .2;
  final double lift_pivot_group_max_setpoint = .1;
  final double joystick_deadband_constant= .05;
  final double extension_max_setpoint = .5;

  final double grabber_pivot_gear_ratio = 60 * 48;
  final double extension_gear_ratio = 60.0 * 27.35;
  final double lift_pivot_group_gear_ratio = 60 * 100;

  enum DrivetrainMode { DriveUp, AutoLevel, Normal }

  DrivetrainMode drivetrain_mode = DrivetrainMode.Normal;

  AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Charging Station", kDefaultAuto);
    m_chooser.addOption("Leave Community", kCustomAuto);
    m_chooser.addOption("Nothing", kCustomAuto2);
    SmartDashboard.putData("Auto choices", m_chooser);


    right_Motor_Group.setInverted(true);

    CameraServer.startAutomaticCapture();
    left_lift_motor.setInverted(true);
    SmartDashboard.putData("Auto choices", m_chooser);
    ahrs = new AHRS(SPI.Port.kMXP);

    extension_vel_pid.setIntegratorRange(-0.2, 0.2);
    lift_pivot_group_vel_pid.setIntegratorRange(-0.2, 0.2);
    grabber_pivot_vel_pid.setIntegratorRange(-0.2, 0.2);

    ang_drivetrain_vel_pid.setIntegratorRange(-0.5, 0.5);
    lin_drivetrain_vel_pid.setIntegratorRange(-0.5, 0.5);
    
    drivetrain_leveling_pid.setIntegratorRange(-0.2, 0.2);

    // See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html#setting-continuous-input
    drivetrain_yaw_pos_pid.enableContinuousInput(-Math.PI, Math.PI);

    
    addPeriodic(() -> {
      final double left_current_pos = left_motor_front.getSelectedSensorPosition();
      final double right_current_pos = right_motor_front.getSelectedSensorPosition();

      left_drivetrain_vel = (left_current_pos - left_drivetrain_prev_pos) / VELOCITY_CALCULATION_DT;
      right_drivetrain_vel = -(right_current_pos - right_drivetrain_prev_pos) / VELOCITY_CALCULATION_DT;

      left_drivetrain_prev_pos = left_current_pos;
      right_drivetrain_prev_pos = right_current_pos;
      
    }, VELOCITY_CALCULATION_DT, 0.005);


    addPeriodic(() -> {
      double current_yaw = Math.toRadians(ahrs.getYaw());
      double angle_diff = current_yaw - prev_yaw;
      double wrapped_ang = Math.atan2(Math.sin(angle_diff), Math.cos(angle_diff));
      yaw_vel = -wrapped_ang / ANG_VELOCITY_CALCULATION_DT;
      prev_yaw = current_yaw;      
    }, ANG_VELOCITY_CALCULATION_DT, 0.005);
  }

  @Override
  public void robotPeriodic() {
   // System.out.println("rpt " + right_lift_motor.getMotorTemperature());
   // System.out.println("lpt " + left_lift_motor.getMotorTemperature());
   // System.out.println("et " + extension.getMotorTemperature());
    System.out.println("GPt " + grabber_pivot.getMotorTemperature());
    System.out.println("gat " + grabber_arms.getMotorTemperature());
    System.out.println("lvel" + left_drivetrain_vel);
    System.out.println("rvel" + right_drivetrain_vel);
    System.out.println("yaw_vel" + yaw_vel);
    System.out.println("yaw" + Math.toRadians(ahrs.getYaw()));

    //System.out.println("joyx" + stick.getX());
    //System.out.println("joyy" + stick.getY());

    // System.out.println("rpc " + right_lift_motor.getOutputCurrent());
    // System.out.println("lpc " + left_lift_motor.getOutputCurrent());
    // System.out.println("GPc " + grabber_pivot.getOutputCurrent());
    // System.out.println("exc " + extension.getOutputCurrent());
    //System.out.println(ahrs.getRoll());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
    switch (m_autoSelected) {
    case kCustomAuto: ;
      //leave community
      differential_drive.feedWatchdog();
  
      if (autonomous_timer.hasElapsed(4.0)) {
        left_Motor_Group.set(0.0);
        right_Motor_Group.set(0.0);
      }
      else {
        left_Motor_Group.set(0.3);
        right_Motor_Group.set(0.3);
      


        break;
      }
      switch (m_autoSelected) {
      case kCustomAuto2: ;
        //robot stays still
        if (autonomous_timer.hasElapsed(0.5)) {
          left_Motor_Group.set(0.0);
          right_Motor_Group.set(0.0);
        }

        
      case kDefaultAuto:
      default:
      //charging station
      if (autonomous_timer.hasElapsed(0.5)){
        drive_up_timer.reset();
        drive_up_timer.start();
        drivetrain_mode = DrivetrainMode.DriveUp;
        prev_yaw = 0.0;
        yaw_vel = 0.0;
        ahrs.zeroYaw();
  
      }
      else if (autonomous_timer.hasElapsed(7.5))  {
        System.out.println("EXECUTING DRIVEUP");
        if (drive_up_timer.hasElapsed(AUTO_DRIVE_UP_TIME)) {
          drivetrain_mode = DrivetrainMode.AutoLevel;
          drive_up_timer.reset();
          straightDrive(0);
        } else {
          straightDrive(AUTO_DRIVE_UP_VEL);
        }
      }
      else if (drivetrain_mode == DrivetrainMode.AutoLevel) {
        System.out.println("EXECUTING AUTOLEVEL");
        autoLevel();
      }
        break;
    }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {

    //  LIFT PIVOT
    double lift_pivot_group_setpoint = logitechController.getRawAxis(5);
    if (Math.abs(lift_pivot_group_setpoint) < joystick_deadband_constant) {
      lift_pivot_group_setpoint = 0;
    }
    lift_pivot_group_setpoint = lift_pivot_group_setpoint * lift_pivot_group_max_setpoint;

    if (left_lift_motor.getMotorTemperature() > 100 || right_lift_motor.getMotorTemperature() > 100) {
      lift_pivot_group.set(0);
    }
    else {
      lift_pivot_group.set(lift_pivot_group_vel_pid.calculate(right_lift_motor.getEncoder().getVelocity() / lift_pivot_group_gear_ratio, lift_pivot_group_setpoint));
    }

    //  EXTENSION
    double extension_setpoint = logitechController.getRawAxis(3) - logitechController.getRawAxis(2);
    if (Math.abs(extension_setpoint) < joystick_deadband_constant) {
      extension_setpoint = 0;
    }
    extension_setpoint = extension_setpoint * extension_max_setpoint;

    if (extension.getMotorTemperature() > 100) {
      extension.set(0);
    } else {
      extension.set(extension_vel_pid.calculate(extension.getEncoder().getVelocity() / extension_gear_ratio, extension_setpoint));
    }

    // GRABBER PIVOT
    double grabber_pivot_setpoint = -logitechController.getRawAxis(1);
    if (Math.abs(grabber_pivot_setpoint) < joystick_deadband_constant) {
      grabber_pivot_setpoint = 0;
    }
    grabber_pivot_setpoint = grabber_pivot_setpoint * grabber_pivot_max_setpoint;

    if (grabber_pivot.getMotorTemperature() > 100) {
      grabber_pivot.set(0);
    } else {
      grabber_pivot.set(grabber_pivot_vel_pid.calculate (grabber_pivot.getEncoder(). getVelocity() / grabber_pivot_gear_ratio, grabber_pivot_setpoint));
    }
    // GRABBER ARMS
    if (logitechController.getRawButton(GRABBER_ARMS_BUTTON_OUT)) {
      grabber_arms.set(-0.1);
    }
    else if (logitechController.getRawButton(GRABBER_ARMS_BUTTON_IN)) {
      grabber_arms.set(0.2);
    }
    else{
      grabber_arms.set(0);
    }
    //AUTO LEVEL
    if (logitechController.getRawButton (7)) {
      drive_up_timer.reset();
      drive_up_timer.start();
      drivetrain_mode = DrivetrainMode.DriveUp;
      prev_yaw = 0.0;
      yaw_vel = 0.0;
      ahrs.zeroYaw();
    } else if (logitechController.getRawButton(8)) {
      drivetrain_mode = DrivetrainMode.Normal;
    }

    if (drivetrain_mode == DrivetrainMode.Normal) {
      differential_drive.arcadeDrive(-stick.getY(), -stick.getZ());
    }
    else if (drivetrain_mode == DrivetrainMode.DriveUp) {
      System.out.println("EXECUTING DRIVEUP");
      if (drive_up_timer.hasElapsed(AUTO_DRIVE_UP_TIME)) {
        drivetrain_mode = DrivetrainMode.AutoLevel;
        drive_up_timer.reset();
        straightDrive(0);
      } else {
        straightDrive(AUTO_DRIVE_UP_VEL);
      }
    }
    else if (drivetrain_mode == DrivetrainMode.AutoLevel) {
      System.out.println("EXECUTING AUTOLEVEL");
      autoLevel();
    }
  }

  public void autoLevel() {
    double pitch = ahrs.getRoll();
    double linear_velocity_setpoint = 0;
    // Add a deadband to only control if angle is larger than threshold
    if (Math.abs(pitch) > AUTO_LEVEL_DEADBAND_ANG) {
      // We always want zero pitch. Note: this is assuming 90 rotation of roborio
      linear_velocity_setpoint = drivetrain_leveling_pid.calculate(pitch, 0.0);
    }

    // Clamp linear velocity output
    if (linear_velocity_setpoint > AUTO_LEVEL_MAX_LIN_VEL) { linear_velocity_setpoint = AUTO_LEVEL_MAX_LIN_VEL; }
    if (linear_velocity_setpoint < -AUTO_LEVEL_MAX_LIN_VEL) { linear_velocity_setpoint = -AUTO_LEVEL_MAX_LIN_VEL; }

    // We zero the yaw angle when starting level control mode, so try to reach zero degrees yaw
    double angular_velocity_setpoint = drivetrain_yaw_pos_pid.calculate(Math.toRadians(-ahrs.getYaw()), 0.0);

    // Clamp angular velocity output
    if (angular_velocity_setpoint > AUTO_LEVEL_MAX_ANG_VEL) { angular_velocity_setpoint = AUTO_LEVEL_MAX_ANG_VEL; }
    if (angular_velocity_setpoint < -AUTO_LEVEL_MAX_ANG_VEL) { angular_velocity_setpoint = -AUTO_LEVEL_MAX_ANG_VEL; }

    // Control the drivetrain with these velocities
    controlDrivetrain(linear_velocity_setpoint, angular_velocity_setpoint);
  }

  public void straightDrive(double lin_vel) {
    // We zero the yaw angle when starting level control mode, so try to reach zero degrees yaw
    double angular_velocity_setpoint = drivetrain_yaw_pos_pid.calculate(Math.toRadians(-ahrs.getYaw()), 0.0);

    // Clamp angular velocity output
    if (angular_velocity_setpoint > AUTO_LEVEL_MAX_ANG_VEL) { angular_velocity_setpoint = AUTO_LEVEL_MAX_ANG_VEL; }
    if (angular_velocity_setpoint < -AUTO_LEVEL_MAX_ANG_VEL) { angular_velocity_setpoint = -AUTO_LEVEL_MAX_ANG_VEL; }

    // Control the drivetrain with these velocities
    controlDrivetrain(lin_vel, angular_velocity_setpoint);
  }

  public void controlDrivetrain(double linear_velocity, double angular_velocity) {
    // The setpoints are in encoder ticks per second. We need to convert linear and angular
    // velocities to encoder ticks per second on each side of the drivetrain.
    double wheel_base_radius = wheel_base_width / 2;

    double linear_ticks_per_sec = linear_velocity * ticks_per_meter;
    double angular_ticks_per_sec = angular_velocity * wheel_base_radius * ticks_per_meter;

    double avg_vel = (left_drivetrain_vel + right_drivetrain_vel) / 2.0;

    // Calculate feedback term to compensate for error
    // encoders might need to be negated differently 
    double ang_cmd_fb = ang_drivetrain_vel_pid.calculate(yaw_vel, angular_velocity);
    double lin_cmd_fb = lin_drivetrain_vel_pid.calculate(avg_vel, linear_ticks_per_sec);
    
    double left_setpoint = linear_ticks_per_sec - angular_ticks_per_sec;
    double right_setpoint = linear_ticks_per_sec + angular_ticks_per_sec;

    // Calculate a feedforward command based on max freerunning velocity
    double left_cmd_ff = left_setpoint / 7000.0;
    double right_cmd_ff = right_setpoint / 7000.0;

    double left_cmd = left_cmd_ff - ang_cmd_fb + lin_cmd_fb;
    double right_cmd = right_cmd_ff + ang_cmd_fb + lin_cmd_fb;

    System.out.println("left_cmd" + left_cmd);
    System.out.println("right_cmd" + right_cmd);

    differential_drive.tankDrive(left_cmd, right_cmd);
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
