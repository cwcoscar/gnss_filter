float64 rcvTOW          # Measurement time of week in receiver local time [s]
                        # approximately aligned to the GPS time system. The
                        # receiver local time of week number and leap second
                        # information can be used to translate the time to other 
                        # time systems. More information about the difference in
                        # time systems can be found in RINEX 3 documentation. 
                        # For a receiver operating in GLONASS only mode, UTC
                        # time can be determined by subtracting the leapS field 
                        # from GPS time regardless of whether the GPS leap
                        # seconds are valid. 
int16 week             # GPS week number in receiver local time. [weeks]
int16 leapS              # GPS leap seconds (GPS-UTC). [s]
                        # This field represents the receiver's best knowledge of
                        # the leap seconds offset. A flag is given in the
                        # recStat bitfield to indicate if the leap seconds 
                        # are known.
int16 numMeas           # # of measurements to follow

### Measures
float64 prMes             # Pseudorange measurement [m]. GLONASS inter frequency
                          # channel delays are compensated with an internal
                          # calibration table.
float64 cpMes             # Carrier phase measurement [L1 cycles]. The carrier
                          # phase initial ambiguity is initialized using an
                          # approximate value to make the magnitude of
                          # the phase close to the pseudorange
                          # measurement. Clock resets are applied to both
                          # phase and code measurements in accordance
                          # with the RINEX specification.
float32 doMes             # Doppler measurement [Hz] (positive sign for
                          # approaching satellites)
int16 gnssId              # GNSS identifier (see CfgGNSS for constants)

int16 svId                # Satellite identifier (see Satellite Numbering)

int16 freqId              # Only used for GLONASS: This is the frequency
                          # slot + 7 (range from 0 to 13)
int16 locktime           # Carrier phase locktime counter [ms] 
                          # (maximum 64500 ms)
int16 cno                  # Carrier-to-noise density ratio (signal strength) 
                          # [dB-Hz]
int16 prStdev             # Estimated pseudorange measurement standard
                          # deviation [m / 0.01*2^n]
int16 cpStdev             # Estimated carrier phase measurement standard
                          # deviation (note a raw value of 0x0F indicates the
                          # value is invalid) [cycles / 0.004]
int16 doStdev             # Estimated Doppler measurement standard deviation 
                          # [Hz / 0.002*2^n]

int16 trkStat             # Tracking status bitfield
int16 TRK_STAT_PR_VALID = 1       # Pseudorange valid
int16 TRK_STAT_CP_VALID = 2       # Carrier phase valid
int16 TRK_STAT_HALF_CYC = 4       # Half cycle valid
int16 TRK_STAT_SUB_HALF_CYC = 8   # Half cycle subtracted from phase

float64 latitude
float64 longitude
float64 altitude
# Gnss localization (deg/deg/meter)

