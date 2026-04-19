package drivetrains.Swerve;
/**
 * Swerve units for one swerve pod
 * @author Xander Haemel - 31616 404 not found
 * @author Sohum Arora 22985 Paraducks
 */
public class SwerveUnitUpdated {
    private double motorPower;
    private double servoAngle;

    public SwerveUnitUpdated(double newMotorPower, double newServoAngle) {
        
    }

    /**
     * gets the stored motor power
     * @return is the power
     */
    public double getMotorPower() {return motorPower;}

    /**
     * gets the stored servo angle
     * @return servo angle
     */
    public double getServoAngle() {return servoAngle;}

    /**
     * sets the servo angle
     * @param angle is the angle in degrees
     */
    public void setServoAngle(double angle) {this.servoAngle = angle;}
    /**
     * sets the motor to a power
     * @param motorPower is the motor power
     */
    public void setMotorSpeed(double motorPower) {this.motorPower = motorPower;}
}
