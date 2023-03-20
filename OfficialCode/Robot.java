// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/*
  ID Controles:
    • Movimento de Tank: 0
    • Movimento do Intake: 1

  ID PDP: 0

  IDs de Tank:
    • 1 e 3: Direita
    • 2 e 4: Esquerda e Invertido

  IDs de Intake:
    • 5: Elevação Vertical
    • 6: Estender Horizontal (movimento horizontal)
    • 7: Angulação do braço (movimento angular)

  ID PneumaticHUB: 8

*/

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  //#region Definindo variáveis de controles, motores, pneumática e câmera

  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerAttachments = new XboxController(1);

  private DifferentialDrive mydrive;
  private Motor motores;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4, IDMOTOR5 = 5, IDMOTOR6 = 6, IDMOTOR7 = 7,IDPNEUMATICHUB = 8;
  private WPI_VictorSPX motorElevation = new WPI_VictorSPX(IDMOTOR7);
  private WPI_VictorSPX motorExtendArm = new WPI_VictorSPX(IDMOTOR6);
  private WPI_VictorSPX motorArmController = new WPI_VictorSPX(IDMOTOR5);

  private final Compressor compressor = new Compressor(IDPNEUMATICHUB, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(IDPNEUMATICHUB, PneumaticsModuleType.REVPH, 0, 2);

  //#endregion

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // UsbCamera armCamera = CameraServer.startAutomaticCapture(0);
    // UsbCamera tankCamera = CameraServer.startAutomaticCapture(1);
    // armCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);
    // tankCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);

    motores = new Motor(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3); // Iniciar os motores
    mydrive = new DifferentialDrive(motores.GetMotorLeft(), motores.GetMotorRight()); // Define o direcionador

    compressor.enableDigital();  // Ativa o compressor

    motorElevation.setInverted(true);

    doubleSolenoid.set(Value.kOff);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 3.5) {
      // Drive forwards half speed, make sure to turn input squaring off
      mydrive.tankDrive(-0.7, -0.7);
    } else {
      mydrive.stopMotor();
      mydrive.tankDrive(0.01, 0.01);; // stop robot
    }
  }

  @Override
  public void teleopInit() {
    mydrive.setSafetyEnabled(false);
  }

  @Override
  public void teleopPeriodic() {
    TankController(); // Controla a movimentação do robô
    ControlBody(); // Controla a elevação vertical, o movimento do braço  e o de coleta
    ControlCompressor();// Controla o compressor
    
  }

  //#region Control Compressor
  public void ControlCompressor(){
    SmartDashboard.putBoolean("Compressor", compressor.getPressureSwitchValue());

    if (xboxControllerAttachments.getAButton()){
      compressor.enableDigital();
    }
  }
  //#region

  //#region TankControlller
  
  private void TankController() {
    SetVelocityMode();
    MovimentationTank();
  }

  private void MovimentationTank(){
    if (xboxControllerTank.getAButton()) {
      mydrive.stopMotor();
    }
    else if (Math.abs(xboxControllerTank.getLeftY()) >= 0.05 || Math.abs(xboxControllerTank.getLeftX()) >= 0.05) {  // Movendo Joystick
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), xboxControllerTank.getLeftX()*1.2);
    }
    else {
      mydrive.stopMotor();
    }
  }

  private void SetVelocityMode(){
    if (xboxControllerTank.getLeftTriggerAxis() > 0) {
      mydrive.setMaxOutput(0.3);
    }
    else if (xboxControllerTank.getRightTriggerAxis() > 0) {
      mydrive.setMaxOutput(1);
    }
    else {
      mydrive.setMaxOutput(0.5);
    }
  }

  //#endregion

  //#region ControlBody

  private void ControlBody() {
    ControlElevation();
    ExtendArm();
    ControlArm();
    ControlIntake();
  }

  private void ControlElevation() {  
    // Apenas um poderá ser acionado por vez
    if ((xboxControllerAttachments.getLeftTriggerAxis() > 0 && xboxControllerAttachments.getRightTriggerAxis() == 0) || (xboxControllerAttachments.getLeftTriggerAxis() == 0 && xboxControllerAttachments.getRightTriggerAxis() > 0)) {
      if (xboxControllerAttachments.getLeftTriggerAxis() > 0) { // Descer elevador
        motorElevation.set(ControlMode.PercentOutput, xboxControllerAttachments.getLeftTriggerAxis() * -1);
      }
      else if (xboxControllerAttachments.getRightTriggerAxis() > 0) { // Subir elevador
        motorElevation.set(ControlMode.PercentOutput, xboxControllerAttachments.getRightTriggerAxis());
      }
    }
    else {
      motorElevation.stopMotor();
    }
  }

  private void ExtendArm() {
    if (xboxControllerAttachments.getLeftY() != 0) {
      motorExtendArm.set(ControlMode.PercentOutput, xboxControllerAttachments.getLeftY());
    }
    else {
      motorExtendArm.stopMotor();
    }
  }

  private void ControlArm() {
    if (xboxControllerAttachments.getRightY() != 0) {
      motorArmController.set(ControlMode.PercentOutput, xboxControllerAttachments.getRightY());
    }
    else {
      motorArmController.stopMotor();
    }
  }

  private void ControlIntake() {
    if (xboxControllerAttachments.getLeftBumper()) {  // Abrir
      doubleSolenoid.set(Value.kReverse);
    }
    else if (xboxControllerAttachments.getRightBumper()) {  // Fecahr
      doubleSolenoid.set(Value.kForward);
    }
  }

  //#endregion

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