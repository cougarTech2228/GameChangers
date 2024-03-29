package frc.robot.Toolkit;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.Timer;

public class CT_DigitalInput extends DigitalInput {

    private Runnable m_methodToRun;
    private Runnable m_lastMethodToRun;
    private boolean m_isInterruptLatched;
    private boolean m_negateLogic;
    private boolean m_doBeginningEdge;
    private int m_amountOfEdgesSeen;
    private double m_startTime;

    private double m_ignoringInterruptIteration;
    private boolean m_needToRelatchInterrupt;

    private boolean m_handleInterrupts;
    
    /**
     * The sole reason for seperate interrupt "state" flags (m_handleInterrupts and m_isInterruptLatched)
     * is so that there are seperate flags for simply handing the interrupt on conditions and doing relatively
     * complex logic within this class. Having the same variable for EVERY method to alter saw inconsistent and 
     * incorrect results, leading to that single flag being altered at inconsistent times when it probably should/shouldn't
     * be. Additionally, the m_isInterruptLatched is more a replacement for calling enable/disableInterrupts, 
     * while m_handleInterrupts is, as the name suggests, handling them not and necessarily enabling/disabling them alltogether. 
     * There might be a way to simplify the flags down to one, but for now handleInterrupts will be with the
     * onlyHandleInterruptsWhen method and everything else will use m_isInterruptLatched. 
     */

    /**
     * Negate logic is useful if you want to reverse the output of getStatus(). A use for setting this variable true
     * would be if you were using a light beam sensor and wanted to differentiate between searching for something blocking
     * the sensors, or searching for the absence of something. 
     */ 

    /**
     * Creates a default DigitalIO instance with the ability to negate the logic.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param negateLogic can negate the return value of getStatus() for this instance.
     */
    public CT_DigitalInput(int pin, boolean negateLogic) {
        this(pin, null, negateLogic);
    }

    /**
     * Creates a default DigitalIO instance.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     */
    public CT_DigitalInput(int pin) {
        this(pin, false);
    }

    /**
     * Creates a DigitalIO instance with a method.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     */
    public CT_DigitalInput(int pin, Runnable methodToRun) { 
        this(pin, methodToRun, false);
    }

    /**
     * Creates a DigitalIO instance with a method and the ability to negate logic.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     * @param negateLogic can negate the return value of getStatus() for this instance
     */
    public CT_DigitalInput(int pin, Runnable methodToRun, boolean negateLogic) { 
        super(pin);
        m_negateLogic = negateLogic;
        m_methodToRun = methodToRun;
        m_lastMethodToRun = m_methodToRun;
        m_isInterruptLatched = false;
        m_amountOfEdgesSeen = 0;
        m_startTime = 0;
        m_ignoringInterruptIteration = 0;
        m_needToRelatchInterrupt = false;
        m_handleInterrupts = false;
    }

    /**
     * POLLING/NON-INTERRUPT METHOD
     * 
     * Sets the method that will be ran in the runWhenTripped method.
     * 
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     */
    public void setMethodToRun(Runnable methodToRun) {
        m_methodToRun = methodToRun;
        m_lastMethodToRun = m_methodToRun;
    }

    /**
     * POLLING/NON-INTERRUPT METHOD
     * 
     * Resets the method to run for the runWhenTripped method to use as it won't run 
     * again if a method is not inputted or isn't reseted.
     */
    public void resetMethodToRun() {
        m_methodToRun = m_lastMethodToRun;
    }

    /**
     * POLLING/NON-INTERRUPT METHOD
     * 
     * Checks to see if the digital IO is tripped depending on if it is acive high or low.
     * If the sensor was tripped, the method either passed in to a constructor or DigitalIO method setMethodToRun will run.
     * This method will not work if an interrupt has been enabled. 
     * The given method will only run ONCE and will have to be reset by the DigitalIO method setMethodToRun.
     * This method is best used when called in the periodic method of a subsystem.
     * 
     * @return If the digital IO was tripped.
     */
    public boolean runWhenTripped() {
        if(get() && !m_isInterruptLatched) {

            if(m_methodToRun != null) {
                Runnable method = m_methodToRun;
                m_methodToRun = null;
                method.run();
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }
    }

    /**
     * Sets an interrupt for the digital input. Can be used in conjunction with the 
     * onlyHandleInterruptsWhen() method to only run the method when certain conditions are met.
     * To use a command, for example use: "() -> new PrintCommand("Interrupt Fired").schedule" for the runnable.
     * 
     * @param runnable the runnable that will run when the interrupt is fired.
     * @param interruptOnRisingEdge fire interrupt on the rising edge.
     * @param interruptOnFallingEdge fire interrupt on the falling edge.
     */
    public void setInterrupt(Runnable runnable, boolean interruptOnRisingEdge, boolean interruptOnFallingEdge) {

        requestInterrupts(new InterruptHandlerFunction<Object>() {

            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                if(m_isInterruptLatched && m_handleInterrupts) {
                    runnable.run();
                } else { /* Do Nothing */ }

                // Uncomment this if you want to see what edge is being produced by the sensor.
                // if((interruptAssertedMask & 0x0100) == 0x0100) {
                //     System.out.println("Falling edge");
                // } else {
                //     System.out.println("Rising edge");
                // }

            }
        });

        setUpSourceEdge(interruptOnRisingEdge, interruptOnFallingEdge);
        enableInterrupts();
        setInterruptLatched(true);
        m_handleInterrupts = true;
    }

    /**
     * Sets an interrupt for the digital input. Can be used in conjunction with the 
     * onlyHandleInterruptsWhen() method to only run the method when certain conditions are met.
     * To use a command, for example use: "() -> new PrintCommand("Interrupt Fired").schedule" for the runnable.
     * 
     * @param runnable the runnable that will run when the interrupt is fired.
     * @param interruptOnRisingEdge fire interrupt on the rising edge.
     * @param interruptOnFallingEdge fire interrupt on the falling edge.
     * @param time time in seconds for the interrupt to be ignored after the interupt is activated.
     */
    public void setTimedInterrupt(Runnable runnable, boolean interruptOnRisingEdge, boolean interruptOnFallingEdge, double time) {

        requestInterrupts(new InterruptHandlerFunction<Object>() {

            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                if((Timer.getFPGATimestamp() - m_startTime) >= time) {
                    if(m_isInterruptLatched && m_handleInterrupts) {
                        runnable.run();
                        m_startTime = Timer.getFPGATimestamp();
                    } else { /* Do Nothing */ }
                }

                // Uncomment this if you want to see what edge is being produced by the sensor.
                // if((interruptAssertedMask & 0x0100) == 0x0100) {
                //     System.out.println("Falling edge");
                // } else {
                //     System.out.println("Rising edge");
                // }

            }
        });

        setUpSourceEdge(interruptOnRisingEdge, interruptOnFallingEdge);
        enableInterrupts();
        m_isInterruptLatched = true;
        m_handleInterrupts = true;
    }

    /**
     * Sets up an alternating interrupt that will enable and disable itself based on the two edges.
     * For example, if you decide to begin on a rising edge, another rising edge cannot trigger 
     * the interrupt until a falling edge happens for a certain amount of edges.
     * To use a command, for example use: "() -> new PrintCommand("Interrupt Fired").schedule" for the runnable.
     * 
     * @param runnable the runnable to be run when the interrupt is fired.
     * @param beginOnRisingEdge whether or not the interrupt will start on the rising edge or the falling edge.
     *                          The example above is an example of this value being true.
     * @param amountOfEdges amount of edges of the opposite edge that needs to be seen before enabling the beginning edge.
     */
    public void setAlternatingInterrupt(Runnable runnable, boolean beginOnRisingEdge, int amountOfEdges) {

        m_doBeginningEdge = true;

        requestInterrupts(new InterruptHandlerFunction<Object>() {
            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                if(m_isInterruptLatched && m_handleInterrupts) {
                    boolean wasRisingEdge = true;
                    if((interruptAssertedMask & 0x0100) == 0x0100) {
                        wasRisingEdge = false;
                        //System.out.println("Falling edge");
                    }
                    // else {
                    //     System.out.println("Rising edge");
                    // }
                    // Uncomment this if you want to see what edge is being produced by the sensor.

                    if(beginOnRisingEdge) {
                        if(wasRisingEdge && m_doBeginningEdge) {
                            runnable.run();
                            m_doBeginningEdge = false;
                            m_amountOfEdgesSeen = 0;
                        } else if(!wasRisingEdge) {
                            m_amountOfEdgesSeen++;
                            if(m_amountOfEdgesSeen == amountOfEdges)
                                m_doBeginningEdge = true;
                        }
                    } else {
                        if(!wasRisingEdge && m_doBeginningEdge) {
                            runnable.run();
                            m_doBeginningEdge = false;
                            m_amountOfEdgesSeen = 0;
                        } else if(wasRisingEdge) {
                            m_amountOfEdgesSeen++;
                            if(m_amountOfEdgesSeen == amountOfEdges)
                                m_doBeginningEdge = true;
                        }
                    }
                }
            }
        });

        setUpSourceEdge(true, true);
        enableInterrupts();
        m_isInterruptLatched = true;
        m_handleInterrupts = true;
    }

    /**
     * This method is for if you want to ignore interrupts after a certain action as taken place.
     * Use this method in a periodic to gain full effect as it relies on iteration to count time.
     * Use this method in conjunction with ignoreInterruptsNow().
     * 
     * @param seconds time in seconds interrupts will be ignored after ignoreInterruptsNow() is called.
     */
    public void ignoreInterruptsFor(double seconds) {
        if(m_needToRelatchInterrupt) {
            m_ignoringInterruptIteration++;

            if(m_ignoringInterruptIteration >= (seconds * 50)) {
                //System.out.println("Interrupts can now be seen");
                setInterruptLatched(true);
                m_needToRelatchInterrupt = false;
                m_ignoringInterruptIteration = 0;
            }
        }
    }

    /**
     * This method works with the ignoreInterruptsFor() method, which should be called in a periodic.
     * Call this method when you want interrupts to be ignored for the amount of time passed into ignoreInterruptsFor()
     */
    public void ignoreInterruptsNow() {
        //System.out.println("Ignoring Interrupts");
        setInterruptLatched(false);
        m_needToRelatchInterrupt = true;
    }

    /**
     * Enables the interrupts when all of the conditions are true.
     * Use this method in a periodic for full effect.
     * @param conditions the conditions to be met for the interrupt to be activated. 
     */
    public void onlyHandleInterruptsWhen(boolean... conditions) {
        m_handleInterrupts = isAllTrue(conditions);
    }

    /**
     * Setter method for the latch interrupt variable. Will decide if said interrupt will be run or not.
     * 
     * @param latchInterrupt boolean variable to be set to m_isInterruptLatched
     */
    private void setInterruptLatched(boolean latchInterrupt) {
        m_isInterruptLatched = latchInterrupt;
    }

    /**
     * Returns if the interrupt is latched
     * @return if the interrupt is latched
     */
    public boolean isInterruptLatched() {
        return m_isInterruptLatched;
    }

    /**
     * Gets the status of the digital input. 
     * Returned value is negated depending on the value of negateLogic passed into the constructor.
     * If no negateLogic boolean was passed into the constructor, then the logic will not be negated.
     * 
     * @return the status of the digital input.
     */
    public boolean get() {
        if(m_negateLogic)
            return !super.get();
        else
            return super.get();
    }

    /**
     * Determines if every value in a boolean array is true.
     * @return whether or not every value is true.
     */
    private boolean isAllTrue(boolean[] array) {
        for (boolean b: array) {
            if (!b) {
                return false;
            }
        }
        return true;
    }

}