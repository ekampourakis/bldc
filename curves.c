// Throttle curves
// Input range: [0.0, 1.0]
// Output range: [0.0, 1.0]
float ThrottleCurve_Cruise(float Throttle) {
    // Curves are designed for inputs in range of [0.0, 100.0] for convenience
    // Map input to required range
	float x = Throttle * 100.0;
    // Calculate output
	float Return = -4.67295958912712E-19*powf(x,10)-1.06556085662012E-17*powf(x,9)
					-7.45588240125191E-16*powf(x,8)+8.09408774437283E-13*powf(x,7)
					-1.41330185621697E-11*powf(x,6)+1.59122820981732E-08*powf(x,5)
					+1.54418795880239E-06*powf(x,4)-0.000688945018719017*powf(x,3)
					+0.0381048838450034*powf(x,2)+0.604507796654288*x-3.64748061104605E-13;
	// Contain output in required range
    Return /= 100.0;
	return Return;
}
float ThrottleCurve_Race(float Throttle) {
	float x = Throttle * 100.0;
	float Return = 5.00955311844635E-19*powf(x,10)-4.70669651905403E-18*powf(x,9)
					-1.26336856326747E-14*powf(x,8)+1.48989828341861E-13*powf(x,7)
					-1.30982244889488E-11*powf(x,6)+7.97273790527461E-09*powf(x,5)
					+4.02329970575204E-06*powf(x,4)-0.000669049571195449*powf(x,3)
					+0.0172320642167611*powf(x,2)+1.96120608157882*x+8.24286451703942E-13;
	Return /= 100.0;
	return Return;
}

// Convert ERPM value to fraction of max ERPMs
float RPMToFraction(float ERPM, bool ConvertToPercent) {
	float Result = ERPM;
	// Limiting ERPM input to 0.0 will treat negative ERPM as starting from standstill
	mc_configuration* config = mc_interface_get_configuration();
	utils_truncate_number(&Result, 0.0, config->l_max_erpm);
	// Convert to fraction of max ERPM
	Result /= config->l_max_erpm;
	// Contain in useful range
	utils_truncate_number(&Result, 0.0, 1.0);
	return (ConvertToPercent ? Result * 100.0 : Result);
}

// Power curves
// Input:   ERPM                    Range: [0.0, MCCONF_L_RPM_MAX]
// Output:  Maximum Amps fraction   Range: [0.0, 1.0]
float PowerCurve_Cruise(float Input) {
    // Curves are designed for inputs in range of [0.0, 100.0] for convenience
    // Convert ERPM input value to fraction value
	float x = RPMToFraction(Input, true);
    // Calculate output
	float Output = 2.91796555055034E-12*powf(x,7)-4.95282884127652E-10*powf(x,6)
                    +3.23671237797064E-09*powf(x,5)-2.09586670717177E-06*powf(x,4)
                    +0.00116462984668322*powf(x,3)-0.115681265669006*powf(x,2)
                    +4.32001791790831*x+49.3340835571487;
    // Contain output in required range
	Output /= 100.0;
	utils_truncate_number(&Output, 0.0, 1.0);
	return Output;
}
float PowerCurve_Race(float Input) {
	float x = RPMToFraction(Input, true);
	float Output = 149.4555 / powf(2.71828, 0.000503872 * powf(x - 56.22544, 2));
	Output /= 100.0;
	utils_truncate_number(&Output, 0.0, 1.0);
	return Output;
}