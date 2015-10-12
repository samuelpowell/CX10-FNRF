void control()
{
	
	
	
	
	// Set next state transition
    state_next = BIND;
    
    // Start main FC loop
    while(1) {
        
        state = state_next;                 // Set to next state
        uint32_t CycleStart = micros();     // Record start of cycle
        
        switch(state)
        {
            case INIT:
                // It should never come to this
                break;
            
            case BIND:
                // Bind RF, when finished, move to calibration state
                set_blink_style(BLINKER_BIND);
                bind_rf();
                state_next = CALIBRATING;            
                break;
            
            case CALIBRATING:
                // Run calibration cycle, when finished, move to disarmed state
                set_blink_style(BLINKER_BIND);
                if(calibGyroDone > 0) ReadMPU(GyroXYZ, ACCXYZ, angle, &I2C_Errors, &calibGyroDone);
                else state_next = DISARMED;
                break;
                           
            case DISARMED:
                // Await a valid arming request, move to armed state when received
                set_blink_style(BLINKER_DISARM);
                failsafe = rx_rf(RXcommands) ? 0 : failsafe+1;
                if(RXcommands[4] > 150 && failsafe < 10 && RXcommands[0] <= 150) state_next = ARMED;    
                break;
                        
            case ARMED:
            case ARMED_LOWBAT:    
                // Device is armed, process flight control and write motors, move to
                // disarm if there is an RF failure or if it commanded.
                set_blink_style(BLINKER_ON);
                ReadMPU(GyroXYZ, ACCXYZ, angle, &I2C_Errors, &calibGyroDone);
                failsafe = rx_rf(RXcommands) ? 0 : failsafe+1;
                
                // Disarm on RF failsafe or user command
                if(failsafe > 10 || RXcommands[4] <= 150)
                {
                    RXcommands[0] = 0;
                    state_next = DISARMED;
                }
                
                // Get setpooints
                for(int i=0; i<3; i++)
                {
                    RPY_useRates[i] = 100-(uint32_t)((abs(RXcommands[i+1])*2)*RPY_Rate[i])/1000;
                    if(mode == 0)
                    { // HH mode
                        setpoint[i] = ((RXcommands[i+1])*RC_Rate/100);
                    }
                }
                
                // PID controller (time is not implemented because of a fix loop time)
                for(int i=0; i<3; i++)
                {
                    // Error
                    int32_t error = setpoint[i]-((GyroXYZ[i])*RPY_useRates[i]/100);
                    
                    // Proportional
                    int32_t PT = (error*G_P[i])/100;
                    
                    // Integral
                    int32_t IT = constrain(error+Isum[i],(Imax[i]*-1),Imax[i]);
                    if(abs(error) > 700)  IT = 0;
                    if(RXcommands[0] < MIN_COMMAND) IT = 0;
                    Isum[i] = IT;
                    IT = (IT*G_I[i])/3000;
                    
                    // Derivative
                    int16_t tmpDT = error-lastError[i];
                    int32_t DT= ((tmpDT+LastDt[i])*G_D[i])/300;
                    LastDt[i] = tmpDT;
                    lastError[i] = error;
                    
                    // Combine
                    if(i == 2)
                    {
                        PIDdata[i] = constrain(PT+IT+DT,-500,+500);
                    }
                    else
                    {
                        PIDdata[i] = PT+IT+DT;
                    }
                }
                
                // Set motor duty cycle
                set_motorpwm(PIDdata, RXcommands, (state_next == ARMED) && (RXcommands[0] > MIN_COMMAND));
                
                // Sample the battery voltage
                ADC_StartOfConversion(ADC1);
                
                if(millis()-last_Time > 100) // 10Hz
                {
                    failsafe++; // RX should send with ~50Hz so it should not be higher then 10 as long as there is a good signal
                    last_Time = millis();
                   
                    if(LiPoEmptyWaring == 350)
                    {
                        set_blink_style(BLINKER_LOWBAT);
                        
                        #if defined(CX10_BLUE) // turn off to save the lipo
                        if(LiPoVolt < 250) GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
                        #endif
                        
                    }
                    else if(LiPoVolt < 300) LiPoEmptyWaring++;
                    else if(LiPoEmptyWaring > 10) LiPoEmptyWaring -= 10;
                }        
                break;
                
        }
                
        while(micros()-CycleStart<minCycleTime);
           
    }
}
	