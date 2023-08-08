package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class MotorExtend {
    public VictorSPX motorExtend;

    private Encoder encoderExtend = new Encoder(1, 2, true);

    public MotorExtend(int idMotor){
        motorExtend = new VictorSPX(idMotor);
    }

    public void ResetEncoderExtend() {
        encoderExtend.reset();
    }

    public int GetEncoderExtend() {
        return encoderExtend.get() / 64;
    }
}