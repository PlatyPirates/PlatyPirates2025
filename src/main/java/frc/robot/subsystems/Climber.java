package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Climber extends SubsystemBase {
    private final SparkMax m_climberMotorSparkMax;

    public Climber(){
        m_climberMotorSparkMax = new SparkMax(DriveConstants.kClimberMotorCanId, MotorType.kBrushless);
    }
    public void setSpeed(double speed){
        m_climberMotorSparkMax.set(speed);
    }
}
