package team3647.lib.drivers;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping
 * duplicate set commands. (254)
 */
public class LazySparkMax extends CANSparkMax {
    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;
    

    // Set if is a follower
    protected CANSparkMax mLeader = null;
    CANPIDController pidController;
    public LazySparkMax(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);
        pidController = getPIDController();
    }

    public CANSparkMax getLeader() {
        return mLeader;
    }

    @Override
    public CANError follow(final CANSparkMax leader) {
        mLeader = leader;
        return super.follow(leader);
    }

    /**
     * wrapper method to mimic TalonSRX set method
     */
    public void set(ControlType type, double setpoint) {
        if (setpoint != mLastSet || type != mLastControlType) {
            mLastSet = setpoint;
            mLastControlType = type;
            pidController.setReference(setpoint, type);
        }
    }
}
