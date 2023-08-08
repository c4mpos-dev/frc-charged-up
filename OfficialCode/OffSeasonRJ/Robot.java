// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#region Bibliotecas

package frc.robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//#endregion

public class Robot extends TimedRobot {
  //#region Variáveis para escolha no Shuffleboard

  private static final String kCustomAuto1 = "Auto 1";
  private static final String kCustomAuto2 = "Auto 2";
  private static final String kCustomAuto3 = "Auto 3";
  private static final String kCustomAuto4 = "Auto 4";

  private static final String kCustomRedAlliance = "Red Alliance";
  private static final String kCustomBlueAlliance = "Blue Alliance";

  private String m_autoSelected;
  private final SendableChooser<String> m_autonomous_chooser = new SendableChooser<>();

  private String m_colorTeamSelected;
  private final SendableChooser<String> m_color_team_chooser = new SendableChooser<>();
  //#endregion

  //#region Controles
  
  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerArm = new XboxController(1);
  //#endregion
  
  //#region Definir variáveis de controles e PIDs

  private DifferentialDrive mydrive;
  private MotorTank motoresTank;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4;
  PIDController pidTank = new PIDController(0.035, 0.01, 0);

  private DifferentialDrive myArm;
  private MotorArm motoresArm;
  private final int IDMOTOR5 = 5, IDMOTOR7 = 7, IDMOTOR8 = 8, IDMOTOR9 = 6;
  PIDController pidArm  = new PIDController(0.25, 0.5, 0);   // 0.25   -      0.5

  private MotorExtend motoresExtend;
  private final int IDMOTOR6 = 9;
  PIDController pidExtend  = new PIDController(0.04, 0.1, 0); // 0.03           - 0.1

  PIDController pidChargeStation  = new PIDController(0.048, 0, 0);
  PIDController pidFollowLimelight  = new PIDController(0.04, 0, 0);
  PIDController pidTurnLimelight  = new PIDController(0.07, 0.01, 0.0);
  //#endregions

  //#region Definir Pneumática e Solenóide

  private final int IDPNEUMATICHUB = 11;
  private final Compressor compressor = new Compressor(IDPNEUMATICHUB, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(IDPNEUMATICHUB, PneumaticsModuleType.REVPH, 0, 2);

  //#endregion

  //#region Limmelight e Pigeon 2

  Map<String, Integer> listPipelines = new HashMap<String, Integer>();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");

  Pigeon2 pigeon = new Pigeon2(10);
  PIDController pidYaw = new PIDController(0.05, 0, 0);

  //#endregion

  //#region Variáveis para HOLD/BRAÇO

  boolean movimentandoArm;
  double positionArmHold;
 
  boolean movimentandoExtend;
  double positionExtendHold;

  //#endregion

  // Variável para autônomo
  private int etapasAutonomo = 1;

  private String posicaoRobo = "default";


  double setpointRoll;

  @Override
  public void robotInit() {

    //#region Shuffleboard Opções Autônomo

    m_autonomous_chooser.setDefaultOption("Auto 1", kCustomAuto1);
    m_autonomous_chooser.addOption("Auto 2", kCustomAuto2);
    m_autonomous_chooser.addOption("Auto 3", kCustomAuto3);
    m_autonomous_chooser.addOption("Auto 4", kCustomAuto4);

    SmartDashboard.putData("Auto choices", m_autonomous_chooser);

    //#endregion

    //#region Shuffleboard Cor da Aliança

    m_color_team_chooser.setDefaultOption("Red Alliance", kCustomRedAlliance);
    m_color_team_chooser.addOption("Blue Alliance", kCustomBlueAlliance);
    SmartDashboard.putData("Team Color", m_color_team_chooser);

    //#endregion

    //#region Iniciar motores e DifferentialDrive

    motoresTank = new MotorTank(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3, 0.6); // Iniciar os motoresTank
    mydrive = new DifferentialDrive(motoresTank.GetMotorLeft(), motoresTank.GetMotorRight()); // Define o direcionador

    motoresArm = new MotorArm(IDMOTOR8, IDMOTOR9, IDMOTOR5, IDMOTOR7);
    myArm = new DifferentialDrive(motoresArm.GetMotorLeft(), motoresArm.GetMotorRight());

    motoresExtend = new MotorExtend(IDMOTOR6);

    //#endregion

    //#region Iniciar Pneumática

    compressor.enableDigital();

    //#endregion
    
    //#region Resetar encoders

    motoresTank.ResetEncoderTank();
    motoresArm.ResetEncoderArm();
    motoresExtend.ResetEncoderExtend();

    //#endregion
  
    //#region Resetar Pigeon2
    
    pigeon.setYaw(0);

    //#endregion
  
    //#region iniciar limelight3

    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    //#endregion

    //#region Lista de pipelines

    listPipelines.put("cubo", 7);
    listPipelines.put("cone", 1);
    listPipelines.put("aprilTags", 2);

    //#endregion

    //#region Safety Disabled

    mydrive.setSafetyEnabled(false);
    myArm.setSafetyEnabled(false);

    //#endregion

    setpointRoll = pigeon.getRoll();

  }

  @Override
  public void robotPeriodic() {

    //#region Shuffleboard Pigeon2

    SmartDashboard.putNumber("Pigeon (Yaw)", pigeon.getYaw());
    SmartDashboard.putNumber("Pigeon (Pitch)", pigeon.getPitch());
    SmartDashboard.putNumber("Pigeon (Roll)", pigeon.getRoll()+3);
    //#endregion

    //#region Shufflebard Limelight

    SmartDashboard.putNumber("LL3 (tx)", tx.getDouble(0));
    SmartDashboard.putNumber("LL3 (ty)", ty.getDouble(0));
    SmartDashboard.putNumber("LL3 (ta)", ta.getDouble(0));
    SmartDashboard.putNumber("LL3 (ID)", tid.getDouble(0));

    //#endregion

    //#region Encoders
    
    SmartDashboard.putNumber("EncoderArm", motoresArm.GetEncoderArm());
    SmartDashboard.putNumber("EncoderExtend", motoresExtend.GetEncoderExtend());
    SmartDashboard.putNumber("Encoder Tank", motoresTank.GetEncoderTank());
    //#endregion
  
    //#region Atualizações contínuas
    
    ResetYaw(); // Reseta o Yaw do Pigeon caso aperte o RightBumper

    //#endregion

  }
 
  private void ResetYaw() {
    if (Math.abs(pigeon.getYaw()) >= 360 || xboxControllerTank.getRightBumper()) {
      pigeon.setYaw(0);
    }
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_autonomous_chooser.getSelected();  // Coleta qual autônomo foi selecionado
    m_colorTeamSelected = m_color_team_chooser.getSelected(); // Coleta a cor do time
    
    System.out.println("Auto selected: " + m_autoSelected);

    //#region Reset Encoders pré-autônomo

    motoresExtend.ResetEncoderExtend();
    motoresArm.ResetEncoderArm();
    motoresTank.ResetEncoderTank();
    //#endregion

    // Configura o PID do braço para o autônomo
    pidArm.setPID(0.3, 0.5, 0);
    pidTank.reset();
    pidArm.reset();
    pidExtend.reset();

    // Escolha do autônomo
    etapasAutonomo = 1;
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) { // Switch para opções de autônomo
      case kCustomAuto1:
        PrimeiroAutonomo(); // Entregar cone e TENTAR pegar cubo
        break;
      case kCustomAuto2:
        SegundoAutonomo();  // Entregar cone e sair da community
        break;
      case kCustomAuto3:
        TerceiroAutonomo(); // Entregar cubo e subir da charge station
        break;
    }
  }

  //#region Primeiro Autônomo

  private void PrimeiroAutonomo() {
    switch (etapasAutonomo) {
      case 1:
        PrimeiraAutoEntrega();
        motoresTank.ResetEncoderTank();
        pigeon.setYaw(0);
        break;
      case 2:
        SeguirRetoPrimeiroAuto();
        if (Math.abs(motoresTank.GetEncoderTank()) >= 20) {  // 72
          SetPositionArm(140);
        } else {
          SetPositionArm(0);
        }
        break;
      case 3:
        if (Math.abs(motoresArm.GetEncoderArm()) > 135) {
          solenoid.set(Value.kReverse);
          etapasAutonomo = 4;
        }
        break;
      case 4:
        SetPositionArm(0);
        if (Math.abs(motoresArm.GetEncoderArm()) < 6) {
          etapasAutonomo = 5;
        }
        else {
          etapasAutonomo = 4;
        }
        break;
      case 5:
        break;
    }
  }

  //#region Case1 - Entregar cone e recolher

  private void PrimeiraAutoEntrega() {
    Cone();
    if (solenoid.get() == Value.kForward) {
      Recolher();
      if (Math.abs(motoresArm.GetEncoderArm()) < 10 && motoresExtend.GetEncoderExtend() < 4) {
        etapasAutonomo = 2;
        motoresExtend.motorExtend.set(ControlMode.PercentOutput, 0);
        motoresExtend.ResetEncoderExtend();
      }
    }
  }

  private void Cone() {     // Entregar cone de frente
    mydrive.stopMotor();
    pidArm.setPID(0.3, 0.5, 0);
    pidExtend.setPID(0.03, 0.1, 0);

    myArm.setMaxOutput(0.2);
    SetPositionArm(-50); //55
    
    if (motoresArm.GetEncoderArm() <= -48){
      SetPositionExtend(165);
    }

    if (motoresExtend.GetEncoderExtend() >= 120){ // 16-
      SetPositionArm(-55);
    }

    if (motoresArm.GetEncoderArm() <= -53 && motoresExtend.GetEncoderExtend() >= 160){
      solenoid.set(Value.kForward);
    }
  }
  
  private void Recolher() {
    SetPositionArm(0);
    if (motoresArm.GetEncoderArm() >= -40) {
      SetPositionExtend(0);
    }
    if (motoresExtend.GetEncoderExtend() < 0) {
      motoresExtend.ResetEncoderExtend();
    }
  }
  //#endregion

  //#region Case2 - Seguir Reto

  private void SeguirRetoPrimeiroAuto() {
    boolean moveu = MoveForCMPrimeiroAuto(-350);
    if (moveu == true) {
      etapasAutonomo = 3;
    } else {
      etapasAutonomo = 2;
    }
  }

  private boolean MoveForCMPrimeiroAuto(double setpointCM) { // Andar somente por centímetros
    double distance = (setpointCM / 47.1) * 10.71;
    
    pidTank.setPID(0.02, 0.01, 0);  // 0.035
    double outputSpeed = pidTank.calculate(motoresTank.GetEncoderTank(), distance);

    mydrive.arcadeDrive(-outputSpeed, 0);  // 0.15
    if (Math.abs(outputSpeed) < 0.04) {
      pidTank.close();
      return true;
    } else {
      return false;
    }
  }
  //#endregion
  
  //#region Case3 - Ir para april tag

  private void FollowAprilTag(double setpointTx, double setpointTa) { // Andar por graus e angulo
    mydrive.setMaxOutput(0.8);
    double outputSpeed;
    pidFollowLimelight.setPID(0.14, 0, 0.06);
    if (Math.abs(tx.getDouble(0)) < 5) {
      outputSpeed = pidFollowLimelight.calculate(ta.getDouble(0), setpointTa);
    }
    else {
      outputSpeed = 0;
    }

    pidTurnLimelight.setPID(0.07, 0.01, 0.0);
    double outputDirection = pidTurnLimelight.calculate(tx.getDouble(0), setpointTx);
    mydrive.arcadeDrive(-outputSpeed, -outputDirection);
  }
  //#endregion


  //#region Segundo Autônomo

  private void SegundoAutonomo() {
    switch (etapasAutonomo) {
      case 1:
        PrimeiraEntregaCone(); // Método reaproveitado do Primeiro autônomo
        break;
      case 2:
        SairCommunity();
        break;
    }
  }

  //#region Case1 - Entregar cone e recolher
  
  private void PrimeiraEntregaCone() {
    EntregarCone();
    if (solenoid.get() == Value.kForward) {
      RecolherPosEntrega();
      if ((motoresArm.GetEncoderArm() < 6 && motoresArm.GetEncoderArm() > -6) && motoresExtend.GetEncoderExtend() < 4) {
        etapasAutonomo = 2;
        motoresExtend.motorExtend.set(ControlMode.PercentOutput, 0);
        motoresExtend.ResetEncoderExtend();
      }
    }
  }

  private void EntregarCone() {
    mydrive.stopMotor();
    pidArm.setPID(0.3, 0.5, 0);

    myArm.setMaxOutput(0.15);
    SetPositionArm(55); //55
    
    if (motoresArm.GetEncoderArm() >= 44){
      SetPositionExtend(165);
    }

    if (motoresExtend.GetEncoderExtend() >= 130){ // 16-
      SetPositionArm(74);
    }

    if (motoresArm.GetEncoderArm() >= 72 && motoresExtend.GetEncoderExtend() >= 163){
      solenoid.set(Value.kForward);
    }
  }

  private void RecolherPosEntrega() {
    SetPositionArm(0);
    if (motoresArm.GetEncoderArm() < 60) {
      SetPositionExtend(0);
    }
    if (motoresExtend.GetEncoderExtend() < 0) {
      motoresExtend.ResetEncoderExtend();
    }
  }
  //#endregion

  //#region Case2 - Sair da community
  private void SairCommunity() {
    pidTank.setPID(0.03, 0, 0);
    mydrive.setMaxOutput(0.6);
    MoveForCM(450);
    SetPositionArm(0);
  }

  private boolean MoveForCM(double setpointCM) { // Andar somente por centímetros

    double distance = (setpointCM / 47.1) * 10.71;

    double outputSpeed = pidTank.calculate(motoresTank.GetEncoderTank(), distance);

    mydrive.arcadeDrive(-outputSpeed, 0);
    if (motoresTank.GetEncoderTank() > 74) {
      pidTank.close();
      return true;
    } else {
      return false;
    }
  }

  //#endregion
  //#endregion
  
  //#region Terceiro Autônomo

  private void TerceiroAutonomo(){
    switch (etapasAutonomo) {
      case 1:
        PrimeiraEntregaCone();
        break;
      case 2:
        SetPositionArm(0);
        if (pigeon.getRoll()+4 > -15) {
          mydrive.setMaxOutput(0.7);
          mydrive.arcadeDrive(-0.7, -0.3);  // 0.5
        }
        else {
          etapasAutonomo = 3;
        }
        break;
      case 3:
        SetPositionArm(0);
        if (Math.abs(pigeon.getRoll()) > Math.abs(setpointRoll+1.5)) {   // Equilibrar
          mydrive.setMaxOutput(0.6);
          pidChargeStation.setPID(0.032, 0, 0);
          double outputSpeed = pidChargeStation.calculate(pigeon.getRoll(), setpointRoll);
          mydrive.arcadeDrive(-outputSpeed, 0.1);
        }
        else {  // está equilibrado
          mydrive.stopMotor();
        }
        
        // ChargeStationAutonomo();

        break;
    }
  }

  //#region Case1 - Entregar cubo
  private void EntregaCubo() {
    if (solenoid.get() == Value.kForward) {
      RecolherPosEntrega();
      if (Math.abs(motoresArm.GetEncoderArm()) < 6 && motoresExtend.GetEncoderExtend() < 4) {
        motoresExtend.motorExtend.set(ControlMode.PercentOutput, 0);
        motoresExtend.ResetEncoderExtend();
        etapasAutonomo = 2;
      }
    }
    else {
      EntregarCubo();
    }
  }

  private void EntregarCubo() {
    mydrive.stopMotor();
    pidArm.setPID(0.3, 0.5, 0);
    pidExtend.setPID(0.03, 0.1, 0);   // 0.04

    myArm.setMaxOutput(0.15);
    SetPositionArm(-54); //-55
    
    if (motoresArm.GetEncoderArm() <= -44){
      SetPositionExtend(145);
    }

    if (motoresArm.GetEncoderArm() <= -52 && motoresExtend.GetEncoderExtend() >= 143){
      solenoid.set(Value.kForward);
    }
  }
  //#endregion

  //#region Case3 - Equilibrar

  private void ChargeStationAutonomo() {
    /* 
    pidChargeStation.setPID(0.04, 0.005, 0.01);  //0.04, 0.005, 0.01
    mydrive.setMaxOutput(0.4);
    double outputSpeed = pidChargeStation.calculate(pigeon.getRoll()+3, 0);
    mydrive.arcadeDrive(-outputSpeed, 0);

    */


    if (pigeon.getRoll()+4 < -15){
      mydrive.arcadeDrive(-0.2, 0);
    }
    else{
      mydrive.stopMotor();
    }
    


  }
  //#endregion

  //#endregion

  @Override
  public void teleopInit() {
    pidArm.setPID(0.3, 0.5, 0);
  }

  @Override
  public void teleopPeriodic() {
    // Controlar Tank
    TankController(); // Controla a movimentação do robô
    OrientationController(); // Define se está de frente ou costas

    // Controlar Braço
    ArmController();
    ExtendController();
    PneumaticController();
    VerificarHolds();

    // Pré posição para resetar Arm e Extend
    SetPosition(0, 0); // Arm-Y -> Ir para a posição, ARM e EXTEND

    // Pré posição para coleta de Game Pieces
    GetGamePieces();  // Arm-X → Ir para posição de -43°

    //#region Testes para autônomo

    ChargeStation(); // Tank-LeftBumper -> Equilibrar na charge station
    //#endregion


    if (xboxControllerTank.getXButton()) {
      FollowAprilTag(0, 5.756);
    }
  }

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
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), xboxControllerTank.getLeftX());
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
      mydrive.setMaxOutput(xboxControllerTank.getRightTriggerAxis()+ 0.5);
    }
    else {
      mydrive.setMaxOutput(0.5);  // 0.5
    }
  }

  //#endregion

  //#region OrientationController

  private void OrientationController() {
    if (xboxControllerTank.getLeftBumper()) {
      posicaoRobo = "costas";
    }
    if (xboxControllerTank.getRightBumper()) {
      posicaoRobo = "frente";
    }
  }
  //#endregion

  //#region ArmController
  
  private void ArmController() {
    myArm.tankDrive(xboxControllerArm.getLeftY(), xboxControllerArm.getLeftY());
  }

  private void ExtendController() {
    motoresExtend.motorExtend.set(ControlMode.PercentOutput, xboxControllerArm.getRightY());
  }

  private void PneumaticController() {
    if (xboxControllerArm.getLeftBumper()) {
      solenoid.set(Value.kForward);
    }
    else if (xboxControllerArm.getRightBumper()) {
      solenoid.set(Value.kReverse);
    }
  }
  
  private void GetGamePieces() {
    if (xboxControllerArm.getXButton()) {
      if (posicaoRobo == "frente") {
        SetPositionArm(-48);
      }
      if (posicaoRobo == "costas") {
        SetPositionArm(48);
      }
    }
  }
  //#endregion

  //#region HoldArm e HoldExtend
  private void VerificarHolds() {
    if (xboxControllerArm.getYButton() == false) {
      HoldArm();
      HoldExtend();
    }
  }

  private void HoldArm() {
    VerificarArmMovimentando();
    if (movimentandoArm == false) {
      SetPositionArm(positionArmHold);
    }
  }

  private void VerificarArmMovimentando() {
    if (Math.abs(xboxControllerArm.getLeftY()) > 0.1) {  // Movimentando o arm
      movimentandoArm = true;
    }
    else if (Math.abs(xboxControllerArm.getLeftY()) < 0.1 && movimentandoArm == true) {
      movimentandoArm = false;
      positionArmHold = motoresArm.GetEncoderArm();
    }
  }

  private void HoldExtend() {
    VerificarExtendMovimentando();
    if (movimentandoExtend == false) {
      SetPositionExtend(positionExtendHold);
    }
  }

  private void VerificarExtendMovimentando() {
    if (Math.abs(xboxControllerArm.getRightY()) > 0.1) {  // Movimentando o arm
      movimentandoExtend = true;
    }
    else if (Math.abs(xboxControllerArm.getRightY()) < 0.1 && movimentandoExtend == true) {
      movimentandoExtend = false;
      positionExtendHold = motoresExtend.GetEncoderExtend();
    }
  }
  //#endregion

  //#region SetPosition

  private void SetPosition(double setPositionArm, double setPositionExtend) {
    if (xboxControllerArm.getYButton()) {
      SetPositionArm(setPositionArm);
      SetPositionExtend(setPositionExtend);
      positionArmHold = 0;
      positionExtendHold = 0;
    }
  }

  private void SetPositionArm(double setpointArm) {
    myArm.setMaxOutput(0.17);
    pidArm.setPID(0.3, 0.04, 0);
    // pidArm.setPID(0.3, 0.1, 0);
    positionArmHold = setpointArm;
    double outputArm = pidArm.calculate(motoresArm.GetEncoderArm(), setpointArm) * -1;
    myArm.tankDrive(outputArm, outputArm);
  }

  private void SetPositionExtend(double setpointExtend) {
    pidExtend.setPID(0.04, 0, 0.007);
    double outputExtend = pidExtend.calculate(motoresExtend.GetEncoderExtend(), setpointExtend) * -1;
    motoresExtend.motorExtend.set(ControlMode.PercentOutput, outputExtend);
  }

  //#endregion

  //#region Pigeon2 para autonomo
  
  private void ChargeStation() {  // Equilibrar na charge station
    if (xboxControllerTank.getLeftBumper()){
      mydrive.setMaxOutput(0.8);
      double outputSpeed = pidChargeStation.calculate(pigeon.getRoll(), -3);
      mydrive.arcadeDrive(-outputSpeed, 0);
    }
  }
  //#endregion

  //#region Limelight para autonomo
  

  //#endregion

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}