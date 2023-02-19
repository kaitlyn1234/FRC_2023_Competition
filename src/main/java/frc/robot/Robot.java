// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
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

  PIDController extension_vel_pid = new PIDController(0.15, 0.3, 0.0);
  PIDController lift_pivot_group_vel_pid = new PIDController(0.9, 2.0, 0.0);
  PIDController grabber_pivot_vel_pid = new PIDController(0.1, 0.2, 0.0);


  private final MotorControllerGroup right_Motor_Group = new MotorControllerGroup(right_motor_front, right_motor_back);
  private final MotorControllerGroup left_Motor_Group = new MotorControllerGroup(left_motor_front, left_motor_back);
  private final MotorControllerGroup lift_pivot_group = new MotorControllerGroup(right_lift_motor, left_lift_motor);
  DifferentialDrive differential_drive = new DifferentialDrive(left_Motor_Group, right_Motor_Group);

  Joystick logitechController = new Joystick(0);
  Joystick stick = new Joystick(1);


  final int LIFT_BUTTON_DOWN = 1;
  final int LIFT_BUTTON_UP = 2;
  final int EXTENSION_BUTTON_IN = 3;
  final int EXTENSION_BUTTON_OUT = 4;
  final int GRABBER_ARMS_BUTTON_OUT = 5;
  final int GRABBER_ARMS_BUTTON_IN = 6;
  final int GRABBER_PIVOT_BUTTON_DOWN = 7;
  final int GRABBER_PIVOT_BUTTON_UP = 8;


  final double grabber_pivot_gear_ratio = 60 * 37.66;
  final double extension_gear_ratio = 60.0 * 27.35;
  final double lift_pivot_group_gear_ratio = 60 * 100;

  //AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    left_Motor_Group.setInverted(true);
    CameraServer.startAutomaticCapture();
    left_lift_motor.setInverted(true);
    SmartDashboard.putData("Auto choices", m_chooser);
    //ahrs = new AHRS(SPI.Port.kMXP);

    extension_vel_pid.setIntegratorRange(-0.2, 0.2);
    lift_pivot_group_vel_pid.setIntegratorRange(-0.2, 0.2);
    grabber_pivot_vel_pid.setIntegratorRange(-0.2, 0.2);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    System.out.println("pivot 1 temp: " + right_lift_motor.getMotorTemperature());
    System.out.println("pivot 2 temp: " + left_lift_motor.getMotorTemperature());
    System.out.println("extension temp: " + extension.getMotorTemperature());
    System.out.println("grabber pivot temp: " + grabber_pivot.getMotorTemperature());
    System.out.println("right pivot current: " + right_lift_motor.getOutputCurrent());
    System.out.println("lift pivot current: " + left_lift_motor.getOutputCurrent());
    System.out.println("grabber pivot current: " + grabber_pivot.getOutputCurrent());
    System.out.println("extension current: " + extension.getOutputCurrent());
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
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
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
    double lift_pivot_group_setpoint = 0;
    if (logitechController.getRawButton(LIFT_BUTTON_UP)) {
      lift_pivot_group_setpoint = 0.1;
    }
    else if (logitechController.getRawButton(LIFT_BUTTON_DOWN)) {
      lift_pivot_group_setpoint = -0.1;
    }
    else {
      lift_pivot_group_setpoint = 0;
    }
   
    if (left_lift_motor.getMotorTemperature() > 100 || right_lift_motor.getMotorTemperature() > 100) {
      lift_pivot_group.set(0);
    }
    else {
      lift_pivot_group.set(lift_pivot_group_vel_pid.calculate(right_lift_motor.getEncoder().getVelocity() / lift_pivot_group_gear_ratio, lift_pivot_group_setpoint));
    }

//  EXTENSION
    double extension_setpoint = 0;
    if (logitechController.getRawButton(EXTENSION_BUTTON_OUT)) {
      extension_setpoint = 1;
    }
    else if (logitechController.getRawButton(EXTENSION_BUTTON_IN)) {
      extension_setpoint = -1;
    }
    else {
      extension_setpoint = 0;
    }

    if (extension.getMotorTemperature() > 100) {
      extension.set(0);
    } else {
      extension.set(extension_vel_pid.calculate(extension.getEncoder().getVelocity() / extension_gear_ratio, extension_setpoint));
    }

// GRABBER PIVOT
  double grabber_pivot_setpoint = 0;
    if (logitechController.getRawButton(GRABBER_PIVOT_BUTTON_DOWN)) {
      grabber_pivot_setpoint = .2;
    }
    else if (logitechController.getRawButton(GRABBER_PIVOT_BUTTON_UP)) {
      grabber_pivot_setpoint = -.2;
    }
    else {
      grabber_pivot_setpoint = 0;
    }

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
      grabber_arms.set(0.1);
    }
    else{
      grabber_arms.set(0.05);
    }

    differential_drive.arcadeDrive(stick.getY(), stick.getX());
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
