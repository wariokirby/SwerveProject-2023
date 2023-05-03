// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class WheelDrive extends PIDSubsystem {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private CANCoder rotationEncoder;
  private RelativeEncoder driveEncoder;
  private final double GEAR_RATIO = .12285;//assumes mk4 standard module with an 8.14:1 gear ratio
  private final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;//in inches
  /** Creates a new WheelDrive. */
  public WheelDrive(int angleMotorID , int speedMotorID , int rotationEncoderID) {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    
    angleMotor = new CANSparkMax(angleMotorID , MotorType.kBrushless);
    speedMotor = new CANSparkMax(speedMotorID , MotorType.kBrushless);
    rotationEncoder = new CANCoder(rotationEncoderID);
    driveEncoder = speedMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(GEAR_RATIO);
    driveEncoder.setVelocityConversionFactor(GEAR_RATIO);


  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    angleMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return rotationEncoder.getPosition();
  }

  public void drive(double speed , double angle){
    speedMotor.set(speed);
    setSetpoint(angle * 180);
  }

  public double getRate(){
    return driveEncoder.getVelocity() * WHEEL_CIRCUMFERENCE;
  }

  public double getDistance(){
    return driveEncoder.getPosition() * WHEEL_CIRCUMFERENCE;
  }

  public void resetDistance(){
    driveEncoder.setPosition(0);
  }
}
