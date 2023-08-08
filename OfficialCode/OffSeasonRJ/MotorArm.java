package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class MotorArm {
    public CANSparkMax leftMotor1;
    public CANSparkMax leftMotor2;
    public CANSparkMax rightMotor1;
    public CANSparkMax rightMotor2;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    public MotorArm(int idLeftMotor1, int idLeftMotor2, int idRightMotor1, int idRightMotor2){
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

    public void ResetEncoderArm() {
        leftMotor1.getEncoder().setPosition(0);
        rightMotor1.getEncoder().setPosition(0);
    }

    public Double GetEncoderArm() {
        return ((leftMotor1.getEncoder().getPosition() + (rightMotor1.getEncoder().getPosition() * -1)) / 2) * 12.5;
    }
}
