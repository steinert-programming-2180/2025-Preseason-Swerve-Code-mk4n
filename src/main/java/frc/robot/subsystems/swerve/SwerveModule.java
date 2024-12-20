package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public interface SwerveModule
 {
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public Rotation2d getAngleEncoder();

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();
    
    public int getModuleNumber(); 

    public CANSparkFlex getDriveMotor();
    public CANSparkFlex getAngleMotor();

    public void setModuleNumber(int moduleNumber);

}