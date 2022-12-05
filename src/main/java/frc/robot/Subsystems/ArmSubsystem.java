package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{

    /* Constants to set */
    //private int COUNTSPERREV = 1;
    private int ID;
    private int MAXPROCEDURELENGTH = -1;

    private CANSparkMax armMotor;
    private double armSpeed = 1d;
    //private RelativeEncoder armEncoder;

    private int currentActionDuration = -1;

    public ArmSubsystem(int id, double speed){
        
        //I do not know what to set these constants to
        this.ID = id;
        this.armSpeed = speed;
        this.armMotor = new CANSparkMax(this.ID, MotorType.kBrushed);

        //not sure if you need to invert all motors
        this.armMotor.setInverted(true);

        //unsure if it is a kHallSensor, a kNoSensor, or a kQuadrature
        //this.armEncoder = this.armMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, this.COUNTSPERREV);
    } 

    public ArmSubsystem(int id){
        this.ID = id;
        this.armMotor = new CANSparkMax(this.ID, MotorType.kBrushed);
    }

    public void setArmSpeed(double speed){
        this.armSpeed = speed;
    }

    @Override
    public void periodic(){
        if(this.currentActionDuration >= 0){
            if((int)System.currentTimeMillis() - this.currentActionDuration >= this.MAXPROCEDURELENGTH){
                this.armMotor.stopMotor();
                this.currentActionDuration = -1;
            }
        }
    }

    public void moveArmUp(){
        if(this.currentActionDuration > 0){
            this.armMotor.stopMotor();
        }
        this.armMotor.set(this.armSpeed);
        this.currentActionDuration = (int)System.currentTimeMillis();
    }

    public void moveArmUp(double speed){
        this.armSpeed = Math.abs(speed);
        this.moveArmUp();
    }

    public void moveArmDown(){
        if(this.currentActionDuration > 0){
            this.armMotor.stopMotor();
        }
        this.armMotor.set(-1 * this.armSpeed);
        this.currentActionDuration = (int)System.currentTimeMillis();
    }

    public void moveArmDown(double speed){
        this.armSpeed = Math.abs(speed);
        this.moveArmDown();
    }
}