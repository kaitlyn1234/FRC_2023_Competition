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


  private final MotorControllerGroup right_Motor_Group = new MotorControllerGroup(right_motor_front, right_motor_back);
  private final MotorControllerGroup left_Motor_Group = new MotorControllerGroup(left_motor_front, left_motor_back);
  private final MotorControllerGroup lift_pivot_Group = new MotorControllerGroup(right_lift_motor, left_lift_motor);
  DifferentialDrive differential_drive = new DifferentialDrive(left_Motor_Group, right_Motor_Group);

  Joystick logitechController = new Joystick(0);
  Joystick stick = new Joystick(1);

  final int LIFT_BUTTON_DOWN = 1;
  final int LIFT_BUTTON_UP = 2;
  final int EXTENSION_BUTTON_IN = 3;
  final int EXTENSION_BUTTON_OUT = 4;
  final int GRABBER_ARMS_BUTTON_OUT = 5;
  final int GRABBER_ARMS_BUTTON_IN = 6;
  final int GRABBER_PIVOT_BUTTON_DOWN = 9;
  final int GRABBER_PIVOT_BUTTON_UP = 10;

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
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    if (logitechController.getRawButton(LIFT_BUTTON_UP)) {
      lift_pivot_Group.set(.05);
    }
    else if (logitechController.getRawButton(LIFT_BUTTON_DOWN)) {
      lift_pivot_Group.set(-.05);}
    
    else {
      lift_pivot_Group.set(0);
    }

//  EXTENSION
    if (logitechController.getRawButton(EXTENSION_BUTTON_OUT)) {
      extension.set(.1);
    }
    else if (logitechController.getRawButton(EXTENSION_BUTTON_IN)) {
      extension.set(-.1);
    }
    else {
      extension.set(0);
    }

// GRABBER PIVOT
    if (logitechController.getRawButton(GRABBER_PIVOT_BUTTON_DOWN)) {
      grabber_pivot.set(.1);
    }
    else if (logitechController.getRawButton(GRABBER_PIVOT_BUTTON_UP)) {
      grabber_pivot.set(-.1);
    }
    else {
      grabber_pivot.set(0);
    }

// GRABBER ARMS
    if (logitechController.getRawButton(GRABBER_ARMS_BUTTON_OUT)) {
      grabber_arms.set(.1);
    }
    else if (logitechController.getRawButton(GRABBER_ARMS_BUTTON_IN)) {
      grabber_arms.set(-.1);
    }
    else{
      grabber_arms.set(0);
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
