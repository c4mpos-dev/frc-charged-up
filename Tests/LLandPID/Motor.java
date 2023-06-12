package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Motor {
    public CANSparkMax leftMotor1;
    public CANSparkMax leftMotor2;
    public CANSparkMax rightMotor1;
    public CANSparkMax rightMotor2;

    // private Encoder leftEncoder;
    // private Encoder rightEncoder;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    public Motor(int idLeftMotor1, int idLeftMotor2, int idRightMotor1, int idRightMotor2){
        leftMotor1 = new CANSparkMax(idLeftMotor1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(idLeftMotor2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(idRightMotor1, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(idRightMotor2, MotorType.kBrushless);

        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

        InvertedMotorLeft();
    }

    private void InvertedMotorLeft(){
        leftMotors.setInverted(true);
    }

    public MotorControllerGroup GetMotorLeft(){
        return leftMotors;
    }

    public MotorControllerGroup GetMotorRight(){
        return rightMotors;
    }

    public void StopMotors() {
        leftMotors.set(0.05);
        rightMotors.set(0.05);
    }
}
