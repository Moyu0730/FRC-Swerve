package frc.robot.library;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PID {
    private double lastError = 0, m_iSum = 0;
    private double k_P, k_I, k_D, k_iLimit, m_Error, m_DeltaTime, m_prevTime, m_Time, m_DeltaError, m_prevError;

    private boolean m_Continuous;
    private double m_MinimumInput, m_MaximumInput;

    public double m_Output;

    public PID(double kP, double kI, double kD, double iLimit ){
        this.k_P = kP;
        this.k_I = kI;
        this.k_D = kD;
        this.k_iLimit = iLimit;
    }

    public double calculate( double ctrPos, double setPoint ){
        m_Time = Timer.getFPGATimestamp();
        m_DeltaTime = m_Time - m_prevTime;
        m_Error = setPoint - ctrPos;
        m_DeltaError = ( m_prevError - m_Error ) / m_DeltaTime;

        if (m_Continuous) {
            double errorBound = (m_MaximumInput - m_MinimumInput) / 2.0;
            m_Error = MathUtil.inputModulus(setPoint - ctrPos, -errorBound, errorBound);
        } else m_Error = setPoint - ctrPos;

        if( m_Error < Math.abs(k_iLimit) ) m_iSum += m_Error;
        else m_iSum = 0;

        m_prevTime = m_Time;
        m_prevError = m_Error;

        return m_Output = k_P * m_Error + k_I * m_iSum + k_D * m_DeltaError;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_Continuous = true;
        m_MinimumInput = minimumInput;
        m_MaximumInput = maximumInput;
    }
}
