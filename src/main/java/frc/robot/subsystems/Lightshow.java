package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;


public class LightShow extends SubsystemBase {
    /** Creates a new Orchestra. */
    CANdle light = new CANdle(Constants.Ports.CANdleID);


    public enum LightShowState {
        SILENT
    }

    LightShowState lightShowState = LightShowState.SILENT;

    public LightShow() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = .8; // dim the LEDs to 8/10 brightness
        light.configAllSettings(config);

        // for (int module = 0; module < 3; module++) {
        //     show.addInstrument(RobotContainer.drivetrain.getModule(module).getDriveMotor());
        //     show.addInstrument(RobotContainer.drivetrain.getModule(module).getSteerMotor());
        // }
    }

    public void setState(LightShowState state) {
        lightShowState = state;
    }

    public LightShowState getState() {
        return lightShowState;
    }

    // has Note
    public void lightOrange() {
        // set brightness
        light.configBrightnessScalar(5);
        // set color
        light.setLEDs(255, 85, 0);
    }

    public void lightBlue() {
        // set brightness
        light.configBrightnessScalar(5);
        // set color
        light.setLEDs(0, 0, 255);
    }

    // Intaking
    public void lightGreen() {
        // set brightness
        light.configBrightnessScalar(5);
        // set color
        light.setLEDs(0, 255, 0);
    }

    public void lightRed() {
        // set brightness
        light.configBrightnessScalar(5);
        // set colo
        light.setLEDs(255, 0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        }
    }
