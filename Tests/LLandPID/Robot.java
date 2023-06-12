// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //#region Definindo variáveis de controles, motores, pneumática e câmera

  private XboxController xboxControllerTank = new XboxController(0);

  private DifferentialDrive mydrive;
  private Motor motores;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4;



  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  Pigeon2 pigeon = new Pigeon2(9);


  //#endregion

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    motores = new Motor(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3); // Iniciar os motores
    mydrive = new DifferentialDrive(motores.GetMotorLeft(), motores.GetMotorRight()); // Define o direcionador
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    // Criar período autônomo
  }

  double errorSum = 0;
  double lastTimeStamp = 0;
  double lastError = 0;

  @Override
  public void teleopInit() {
    mydrive.setSafetyEnabled(false);
    pigeon.setYaw(0);
    pigeon.configFactoryDefault();

    errorSum = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = 0;
  }



  @Override
  public void teleopPeriodic() {
    TankController(); // Controla a movimentação do robô
    LimelightController();
    PIDLine();
    ChargeStation();
    SmartDashboard.putNumber("area", ta.getDouble(0));

    SmartDashboard.putNumber("Yaw", pigeon.getYaw());
    SmartDashboard.putNumber("Pitch", pigeon.getPitch());
    SmartDashboard.putNumber("Roll", pigeon.getRoll()+5);

    // PVirar(180);
    PIDVirar(180);


    if (Math.abs(pigeon.getYaw()) >= 360 || xboxControllerTank.getRightBumper()) {
      pigeon.setYaw(0);
    }
  }

  private void ChargeStation() {
    if (xboxControllerTank.getLeftBumper()){
      final double kP = 0.048;
      final double setPoint = -5;  // Posição do X desejada
  
      double angle = pigeon.getRoll();
  
      double error = setPoint - angle;
      double speed = kP * error;
  
      mydrive.setMaxOutput(0.8);
      mydrive.arcadeDrive(-speed, 0);
    }
  }

  private void PIDVirar (double angulo) {
    if (xboxControllerTank.getLeftStickButton()) {
      final double kP = 0.065; // 0.03  0.055
      final double kI = 0.0023; //0.002
      final double kD = 0.01; // 0.01

      final double iLimit = 1;

      final double setPoint = angulo;  // Posição do X desejada
  
      double angle = pigeon.getYaw();
  
      double error = setPoint - angle;
      double dt = Timer.getFPGATimestamp() - lastTimeStamp;

      if (Math.abs(error) < iLimit) {
        errorSum += error + dt;
      }

      double errorRate = (error - lastError) / dt;

      double outputDirection = kP * error + kI * errorSum + kD * errorRate;
  
      mydrive.setMaxOutput(0.4);
      mydrive.arcadeDrive(0, -outputDirection);

      lastTimeStamp = Timer.getFPGATimestamp();
      lastError = error;
    }
  }

  private void PVirar(double angulo) {
    if (xboxControllerTank.getLeftStickButton()) {
      final double kP = 0.045; // 0.045 para 90°
      final double setPoint = angulo;  // Posição do X desejada
  
      double angle = pigeon.getYaw();
  
      double error = setPoint - angle;
      double outputDirection = kP * error;
  
      mydrive.setMaxOutput(0.4);
      mydrive.arcadeDrive(0, -outputDirection);
    }
  }

  private void PIDLine() {
    if (xboxControllerTank.getBButton()){
      final double kP = 0.3;
      final double setPoint = 0;  // Posição do X desejada
  
      double angle = pigeon.getYaw();
  
      double error = setPoint - angle;
      double outputDirection = kP * error;
  
      mydrive.setMaxOutput(0.8);
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), -outputDirection);
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
      mydrive.setMaxOutput(xboxControllerTank.getRightTriggerAxis()+ 0.5);
    }
    else {
      mydrive.setMaxOutput(0.5);
    }
  }


  private void LimelightController() {
    if (xboxControllerTank.getYButton()) {
      table.getEntry("pipeline").setNumber(1);
      mydrive.setMaxOutput(0.6);
      mydrive.arcadeDrive(-GetSpeed(), -GetDirection());
      // mydrive.tankDrive(GetDirection(), -GetDirection());
    }
    else if (xboxControllerTank.getXButton()) {
      table.getEntry("pipeline").setNumber(0);
      // mydrive.tankDrive(GetDirection(), -GetDirection());
      mydrive.setMaxOutput(0.6);

      mydrive.arcadeDrive(-GetSpeed(), -GetDirection());
    }
  }

  private double GetDirection() {
    final double kP = 0.035;
    final double setPoint = 0;  // Posição do X desejada

    double XPosition = tx.getDouble(0.0);

    double error = setPoint - XPosition;
    double outputSpeed = kP * error;

    return outputSpeed;
  }

  private double GetSpeed() {
    final double kP = 0.06;    //0.28
    final double setPoint = 11;  // Posição da area desejada    1.8

    double area = ta.getDouble(0.0);

    double error = setPoint - area;
    double outputSpeed = kP * error;

    return outputSpeed;
  }


/* 
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    if (xboxControllerTank.getYButton()) {
      if (x > 1){
        mydrive.tankDrive(-0.3, 0.3);
      }
      else if (x < 1){
        mydrive.tankDrive(0.3, -0.3);
      }
      else if(area < 9 && x < 1 && x > 1 ){
        mydrive.tankDrive(-0.3, -0.3);
      }
    } */

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