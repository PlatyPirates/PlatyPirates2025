package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Climber extends SubsystemBase {
    private final SparkMax m_rightClimberMotorSparkMax;
    private final SparkMax m_leftClimberMotorSparkMax;

    public Climber(){
        m_rightClimberMotorSparkMax = new SparkMax(DriveConstants.kRightClimberCanId, MotorType.kBrushless);
        m_leftClimberMotorSparkMax = new SparkMax(DriveConstants.kLeftClimberCanId, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        m_rightClimberMotorSparkMax.set(speed);
        m_leftClimberMotorSparkMax.set(speed);
    }
}
