package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Motor {
    private WPI_VictorSPX leftMotor1;
    private WPI_VictorSPX leftMotor2;
    private WPI_VictorSPX rightMotor1;
    private WPI_VictorSPX rightMotor2;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    public Motor(int idLeftMotor1, int idLeftMotor2, int idRightMotor1, int idRightMotor2){
        leftMotor1 = new WPI_VictorSPX(idLeftMotor1);
        leftMotor2 = new WPI_VictorSPX(idLeftMotor2);
        rightMotor1 = new WPI_VictorSPX(idRightMotor1);
        rightMotor2 = new WPI_VictorSPX(idRightMotor2);

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
