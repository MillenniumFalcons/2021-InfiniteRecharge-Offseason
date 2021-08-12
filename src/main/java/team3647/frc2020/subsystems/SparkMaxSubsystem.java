package team3647.frc2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import team3647.lib.drivers.ClosedLoopFactory;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.SparkMaxUtil;
import team3647.lib.wpi.HALMethods;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.drivers.SparkMaxFactory.Configuration;
import team3647.lib.util.RollingAverage;

/**
 * Add you own bounds.
 */
public abstract class SparkMaxSubsystem implements PeriodicSubsystem {

    public static double kDt = .01;

    private CANSparkMax master;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private ControlType controlType = ControlType.kDutyCycle;
    private Configuration m_masterConfig;
    private final ClosedLoopConfig m_pidConfig;
    private SimpleMotorFeedforward feedforward;
    private final RollingAverage velocityAverage;
    private final RollingAverage positionAverage;

    private boolean positionFiltering = false;
    private boolean velocityFiltering = false;

    public static class PeriodicIO {
        // inputs
        public double position;
        public double velocity;
        public double prevVelocity;
        public double appliedOutput;
        // outputs
        /** In Volts */
        public double feedforward;
        public double demand;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    protected SparkMaxSubsystem(Configuration masterConfig, ClosedLoopConfig pidConfig) {
        boolean error = false;
        if (masterConfig == null) {
            HALMethods.sendDSError("Master config sparkmax Subsystem " + getName() + " was null");
            error = true;
        }

        if (pidConfig == null) {
            HALMethods.sendDSError("pid config sparkmax subsystem " + getName() + " was null");
            error = true;
        }

        if (error) {
            throw new IllegalArgumentException("either master config or pid config were null");
        }
        m_masterConfig = masterConfig;
        m_pidConfig = pidConfig;
        master = SparkMaxFactory.createSparkMax(m_masterConfig);
        encoder = new CANEncoder(master);
        pidController =
                ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
        feedforward = new SimpleMotorFeedforward(pidConfig.kS, pidConfig.kV, pidConfig.kA);
        velocityAverage = new RollingAverage(10);
        positionAverage = new RollingAverage(10);
    }

    protected void enableVelocityFiltering() {
        velocityFiltering = true;
    }

    protected void enablePositionFiltering() {
        positionFiltering = true;
    }

    @Override
    public void init() {
        try {
            setToBrake();
            ClosedLoopFactory.configSparkMaxPIDController(pidController, master, encoder,
                    m_pidConfig, 0);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            encoder = master.getEncoder();
            pidController =
                    ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = encoder.getPosition() * m_pidConfig.kEncoderTicksToUnits;
        periodicIO.velocity = encoder.getVelocity() * m_pidConfig.kEncoderVelocityToRPM;
        if (velocityFiltering) {
            velocityAverage.add(getVelocity());
        }

        if (positionFiltering) {
            positionAverage.add(getPosition());
        }
        periodicIO.appliedOutput = master.get();
    }

    @Override
    public void writePeriodicOutputs() {
        try {
            setSparkMax(controlType, periodicIO.demand, periodicIO.feedforward);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            controlType = ControlType.kDutyCycle;
            periodicIO = new PeriodicIO();
        }
    }

    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.demand = 0;
        periodicIO.feedforward = 0;
        controlType = ControlType.kDutyCycle;
    }

    protected void setEncoderPosition(double newPosition) {
        SparkMaxUtil.checkError(encoder.setPosition(newPosition / m_pidConfig.kEncoderTicksToUnits),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    protected void setPosition(double refrencePt) {
        periodicIO.demand = refrencePt / m_pidConfig.kEncoderTicksToUnits;
        controlType = ControlType.kSmartMotion;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity / m_pidConfig.kEncoderVelocityToRPM;
        periodicIO.feedforward =
                feedforward.calculate(velocity / 60, ((velocity - getVelocity()) / 60));
        System.out.println("demanded ff: " + periodicIO.feedforward);
        controlType = ControlType.kVelocity;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        periodicIO.feedforward = 0;
        controlType = ControlType.kDutyCycle;
    }

    public double getPosition() {
        return periodicIO.position;
    }

    public double getVelocity() {
        return periodicIO.velocity;
    }

    public double getFilteredVelocity() {
        return velocityAverage.getAverage();
    }

    public double getFilteredPosition() {
        return positionAverage.getAverage();
    }

    public double getOutput() {
        return periodicIO.appliedOutput;
    }

    /**
     * @param position position to check in real world units, average position over last 10 loops
     */
    protected boolean reachedPosition(double position) {
        return positionFiltering
                && (position < getFilteredPosition() + m_pidConfig.positionThreshold
                        && position > getFilteredPosition() - m_pidConfig.positionThreshold);
    }

    /**
     * @return is the subsystem at the target position, average position over last 10 loops
     */
    public boolean reachedTargetPosition() {
        if (controlType == ControlType.kPosition || controlType == ControlType.kSmartMotion) {
            return reachedPosition(periodicIO.demand);
        }
        return false;
    }

    /**
     * @param position position to check in real world units, subsystem could be moving
     */
    protected boolean atPosition(double position) {
        return position < getPosition() + m_pidConfig.positionThreshold
                && position > getPosition() - m_pidConfig.positionThreshold
                && Math.abs(getVelocity()) < m_pidConfig.velocityThreshold;
    }

    /**
     * @return is the subsystem at the target position, subsystem could be moving
     */
    public boolean atTargetPosition() {
        if (controlType == ControlType.kPosition || controlType == ControlType.kSmartMotion) {
            return atPosition(periodicIO.demand);
        }
        return false;
    }

    public boolean atVelocity(double velocity) {
        return velocity < getVelocity() + m_pidConfig.velocityThreshold
                && velocity > getVelocity() - m_pidConfig.velocityThreshold;
    }

    public boolean reachedVelocity(double velocity) {
        return velocityFiltering
                && (velocity < getFilteredVelocity() + m_pidConfig.velocityThreshold
                        && velocity > getFilteredVelocity() - m_pidConfig.velocityThreshold);
    }

    /**
     * @return is the current rpm match the setpoint although could be accelerating or decelerating
     *         away
     */
    public boolean atTargetVelocity() {
        if (controlType == ControlType.kVelocity) {
            return atVelocity(periodicIO.demand * m_pidConfig.kEncoderVelocityToRPM);
        }
        return false;
    }

    /**
     * @return was the motor rpm average from the last 10 loops between the rpm thresholds and
     *         setpoint
     */
    public boolean reachedTargetVelocity() {
        if (controlType == ControlType.kVelocity) {
            return reachedVelocity(periodicIO.demand * m_pidConfig.kEncoderVelocityToRPM);
        }
        return false;
    }

    public void setToCoast() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kCoast),
                    " Couldn't set to coast " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            HALMethods.sendDSError(e.toString());
        }
    }

    public void setToBrake() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kBrake),
                    " Couldn't set to brake " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            HALMethods.sendDSError(e.toString());
        }
    }

    private void setSparkMax(ControlType controlType, double demand, double feedForward) {
        // if the feed is outside -12 to 12, adjust to -12 to 12
        feedForward = Math.abs(feedForward) > 12 ? 12 * Math.signum(feedForward) : feedForward;
        if (controlType == ControlType.kDutyCycle) {
            master.set(demand);
        } else {
            pidController.setReference(demand, controlType, 0, feedForward);
        }
    }

    protected CANSparkMax addFollower(SparkMaxFactory.Configuration config,
            boolean isInvertedFromMaster) {
        CANSparkMax follower =
                SparkMaxFactory.createSparkMaxFollower(master, config, isInvertedFromMaster);
        follower.follow(master, isInvertedFromMaster);
        return follower;
    }

    protected CANSparkMax getMaster() {
        return master;
    }
}
