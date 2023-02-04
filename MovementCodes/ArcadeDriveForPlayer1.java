// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.AbstractDocument.LeafElement;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(2);
  private WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(4);
  private WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(1);
  private WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(3);

  private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

  private DifferentialDrive myDrive = new DifferentialDrive(leftMotors, rightMotors);
  private XboxController xboxController = new XboxController(0);

  private double defaultVelocity = 0.5;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    leftMotors.setInverted(true);
    StopMotors();
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
   * below with additional strings. If using the SendableC hooser make sure to add them to the
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
  public void teleopPeriodic() {
    myDrive.setMaxOutput(defaultVelocity);

    if (xboxController.getLeftTriggerAxis() > 0) {
      leftMotors.set(leftMotors.get());
      myDrive.tankDrive(leftMotors.get(), rightMotors.get());
    }

    if (xboxController.getRightTriggerAxis() == 0) {
      myDrive.setMaxOutput(xboxController.getRightTriggerAxis() + 0.45);
    }
    else {
      myDrive.setMaxOutput(xboxController.getRightTriggerAxis());
    }

    myDrive.arcadeDrive(xboxController.getLeftY(), xboxController.getLeftX());
  }

  private void AcionarMotoresTurbo(double velocidade)
  {
    myDrive.setMaxOutput(1);
    myDrive.tankDrive(velocidade, velocidade);
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

  private void StopMotors(){
    myDrive.stopMotor();
  };
}
