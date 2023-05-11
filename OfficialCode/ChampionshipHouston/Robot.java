// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#region Bibliotecas

package frc.robot;
import java.util.Map;
import java.util.concurrent.CancellationException;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//#endregion

/*
  ID Controles:
    • Movimento de Tank: 0
    • Movimento do Intake: 1

  ID PDP: 0

  IDs de Tank:
    • 1 e 3: Direita
    • 2 e 4: Esquerda e Invertido

  IDs de Body:
    • 5: Angulação do braço (movimento angular)
    • 6: Estender Horizontal (movimento horizontal)
    • 7: Elevação Vertical

  ID PneumaticHUB: 
    • 8

  ID Pigeon 2.0: 
    • 9
*/

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //#region Definindo variáveis de controles, motores, encoders, pneumática, câmera, etc.

  // Controles
  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerAttachments = new XboxController(1);

  // Motores
  private DifferentialDrive mydrive;
  private Motor motores;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4, IDMOTOR5 = 5, IDMOTOR6 = 6, IDMOTOR7 = 7,IDPNEUMATICHUB = 8, IDPIGEON = 9;
  private CANSparkMax motorArmController = new CANSparkMax(IDMOTOR5, MotorType.kBrushless);
  private WPI_VictorSPX motorExtendArm = new WPI_VictorSPX(IDMOTOR6);
  private WPI_VictorSPX motorElevation = new WPI_VictorSPX(IDMOTOR7);

  // Encoder(s)
  private double valueEncoderMotorArmController = 0;
  private double valueEncoderMotorExtendArm = 0;

  // Pneumática

  private final Compressor compressor = new Compressor(IDPNEUMATICHUB, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(IDPNEUMATICHUB, PneumaticsModuleType.REVPH, 0, 2);

  // Pigeon 2.0
  private Pigeon2 pigeon2 = new Pigeon2(IDPIGEON);
  private double angleRobot = 0;
  private double angleRobotRounded = 0;

  // Timer
  private final Timer esperaTimer = new Timer();

  // Interruptores
  private boolean climb = false;
  private boolean stop = false;
  private boolean stopSolenoid = false;
  private boolean conditionIdAutonomo3 = false;

  // ID Autonomo
  private int idAutonomo = 0;

  //#endregion

  //#region Robot

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);
    camera.setBrightness(30);

    motores = new Motor(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3); // Iniciar os motores
    mydrive = new DifferentialDrive(motores.GetMotorLeft(), motores.GetMotorRight()); // Define o direcionador
    mydrive.setSafetyEnabled(false);

    motorArmController.setInverted(true); // Inverte o sentido do motor do cabo de aço

    compressor.enableDigital();  // Ativa o compressor
  }

  @Override
  public void robotPeriodic() {
    //#region Updates

    //Att encoders dos motores
    valueEncoderMotorArmController = motorArmController.getEncoder().getPosition();
    
    // Att valores Piegon 2.0
    angleRobot = (pigeon2.getRoll() * -1) - 5;
    angleRobotRounded = (angleRobot * 100) / 100;

    //#endregion

    //#region SmartDashboard

    // Encoder dos motores 
    SmartDashboard.putNumber("Encoder Left", motores.GetLeftEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Right", motores.GetRightEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Arm", valueEncoderMotorArmController);
    SmartDashboard.putNumber("Extend Arm", valueEncoderMotorExtendArm);

    // Variáveis usadas na ChargeStation() e ReverseAnd180()
    SmartDashboard.putNumber("Roll", pigeon2.getRoll()); 
    SmartDashboard.putNumber("Yaw (EixoX)", pigeon2.getYaw());
    SmartDashboard.putNumber("Ângulo", angleRobotRounded);
    SmartDashboard.putBoolean("Subiu", climb);
    SmartDashboard.putBoolean("Parar", stop);

    //#endregion
  }

  //#endregion

  //#region Structure Autonomous

  @Override
  public void autonomousInit() {
    pigeon2.setYaw(0);
    pigeon2.configFactoryDefault();

    esperaTimer.restart();
    mydrive.setMaxOutput(1.0);
    
    climb = false;
    stop = false;
    stopSolenoid = false;
    conditionIdAutonomo3 = false;

    doubleSolenoid.set(Value.kForward); // Fecha a solenoide por padrão

    motores.leftEncoder1.setPosition(0);
    motorArmController.getEncoder().setPosition(0);

    /*==================================
    *   MUDE O PERÍODO AUTÔNOMO AQUI   *
    *    !!!!!!!!!!!!!!!!!!!!!!!!!     *
    ===================================*/
    idAutonomo = 1;
  }

  @Override
  public void autonomousPeriodic() {
    switch (idAutonomo) {
      case 1:
        DeliverMiddleAndExitCommunity(); // Entrega no mid e sai da comunidade
        break;

      case 2: 
        CubeInLowAndChargeStation(); // Entrega o cubo no low e sobre na charge station
        break;

      case 3: 
        DeliverMiddleAndReverseAnd180AndChargeStation();// Entrega no mid, chega para trás, vira 180° e sobe na charge station
        break;
    }
  }

  //#endregion
  
  //#region Autonomous methods

  private void DeliverMiddleAndExitCommunity(){
    
    if (valueEncoderMotorArmController < 78 && stopSolenoid == false){    // 78
      motorArmController.set(1);
    }
    else if (valueEncoderMotorArmController >= 78 && stopSolenoid == false){    // 78
      motorArmController.set(0);

      if (stopSolenoid == false) {
        doubleSolenoid.set(Value.kReverse);

        esperaTimer.restart();
        while (esperaTimer.get() < 0.5){
          motorArmController.set(-0.1);
        }

        stopSolenoid = true;
      }
    }
    else if (stopSolenoid == true && valueEncoderMotorArmController > 10) {
        motorArmController.set(-1);  
    }
    else{
      motorArmController.stopMotor();

      // Exit Community
      if (motores.leftEncoder1.getPosition() > -79.5) { //3,5M //Conta = ((QtdEmCmQueDeseja / 47,1) * -10,71)
        mydrive.tankDrive(0.5, 0.5);
      }
      else {
        mydrive.stopMotor();
      }
    }
  }

  private void CubeInLowAndChargeStation(){

    while (esperaTimer.get() < 1.5) {
      mydrive.tankDrive(0.3, 0.3);
    }

    while (esperaTimer.get() < 1.7) {
      mydrive.tankDrive(-0.6, -0.6);
    }

    if (angleRobotRounded > 4) {
      climb = true;
    }

    if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
      stop = true;
    }
    
    if (climb == false) {
      mydrive.tankDrive(-0.5, -0.5);
    }
    else if (angleRobotRounded < 10 && stop == false) {
      mydrive.tankDrive(-0.4, -0.4);
    }
    else if (angleRobotRounded >= 10 && stop == false) {
      mydrive.tankDrive(-0.35, -0.35);
    }
    else if (stop == true) {
      if (angleRobotRounded > 4) {  // 4
        mydrive.tankDrive(-0.25, -0.25);
      }
      else if (angleRobotRounded < -4){ // -4
        mydrive.tankDrive(0.3, 0.3);  // 0.25
      }
      else {
        mydrive.tankDrive(0, 0);
      }
    }
  }

  private void DeliverMiddleAndReverseAnd180AndChargeStation() {

    if (valueEncoderMotorArmController < 78 && stopSolenoid == false){    // 78
      motorArmController.set(1);
    }
    else if (valueEncoderMotorArmController >= 78 && stopSolenoid == false){    // 78
      motorArmController.set(0);

      if (stopSolenoid == false) {
        doubleSolenoid.set(Value.kReverse);

        esperaTimer.restart();
        while (esperaTimer.get() < 0.5){
          motorArmController.set(-0.1);
        }

        stopSolenoid = true;
      }
    }
    else if (stopSolenoid == true && valueEncoderMotorArmController > 10) {
        motorArmController.set(-1);  
    }
    else{
      motorArmController.stopMotor();

      if (conditionIdAutonomo3 == false){
        // ReverseAnd180
        while (motores.GetLeftEncoder().getPosition() > -7) {
          mydrive.tankDrive(0.5, 0.5);
        }

        while (pigeon2.getYaw() < 160){
          mydrive.tankDrive(0.5, -0.5);
        }

        conditionIdAutonomo3 = true;
      }
      else{
        // ChargeStation
        if (angleRobotRounded > 4) {
          climb = true;
        }

        if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
          stop = true;
        }
        
        if (climb == false) {
          mydrive.tankDrive(-0.5, -0.5);
        }
        else if (angleRobotRounded < 10 && stop == false) {
          mydrive.tankDrive(-0.4, -0.4);
        }
        else if (angleRobotRounded >= 10 && stop == false) {
          mydrive.tankDrive(-0.35, -0.35);
        }
        else if (stop == true) {
          if (angleRobotRounded > 4) {  // 4
            mydrive.tankDrive(-0.25, -0.25);
          }
          else if (angleRobotRounded < -4){ // -4
            mydrive.tankDrive(0.3, 0.3);  // 0.25
          }
          else {
            mydrive.tankDrive(0, 0);
          }
        }
      }
    }  
  }
  
  //#endregion

  //#region Structure Teleop

  @Override
  public void teleopInit() {
    mydrive.setSafetyEnabled(false);
  }

  @Override
  public void teleopPeriodic() {
    TankController(); // Controla a movimentação do robô
    ControlBody(); // Controla a elevação vertical, o movimento do braço  e o de coleta
  }

  //#endregion

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

  private Double GetSensorPosition() {
    return ((motores.GetLeftEncoder().getPosition() * 2 * Math.PI * 7.6) / motores.GetLeftEncoder().getCountsPerRevolution()) * 400;
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
        motorElevation.set(xboxControllerAttachments.getLeftTriggerAxis());
      }
      else if (xboxControllerAttachments.getRightTriggerAxis() > 0) { // Subir elevador
        motorElevation.set(xboxControllerAttachments.getRightTriggerAxis() * -1);
      }
    }
    else {
      motorElevation.stopMotor();
    }
  }

  private void ExtendArm() {  // 225
    if (xboxControllerAttachments.getRightY() != 0) { // Estende
      motorExtendArm.set(xboxControllerAttachments.getRightY()*-1);
    }
    else {
      motorExtendArm.stopMotor();
      motorExtendArm.set(0);
    }    
  }
  

  private void ControlArm() { // -1 para cima → recolher                   1 para baixo → estender

    if (Math.abs(xboxControllerAttachments.getLeftY()) > 0.1)
    {
      motorArmController.set(xboxControllerAttachments.getLeftY());
    }
    else{
      motorArmController.set(0);
    }

    /* 
    double valorMaximoEncoder = 153;
    if (Math.abs(xboxControllerAttachments.getLeftY()) > 0.1)
    {
      if (xboxControllerAttachments.getLeftY() < 0 && valueEncoderMotorArmController > 0) { // Estende
        motorArmController.set(xboxControllerAttachments.getLeftY());
      }
      else if (xboxControllerAttachments.getLeftY() > 0 && valueEncoderMotorArmController < valorMaximoEncoder) {  // Recolhe
        motorArmController.set(xboxControllerAttachments.getLeftY());
      }
      else {
        motorArmController.set(0);
        motorArmController.stopMotor();
      }
    }
    else {
      motorArmController.stopMotor();
      motorArmController.set(0);
    }
  */
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

  //#region Methods Off

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

  //#endregion
}