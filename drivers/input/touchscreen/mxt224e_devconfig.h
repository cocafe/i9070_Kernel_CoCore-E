
/**********************************************************

  DEVICE   : mxT224E  0.5.6
  CUSTOMER : SAMSUNG
  PROJECT  : N1
  X SIZE   : X18
  Y SIZE   : Y11
  CHRGTIME : 2.59us
  X Pitch  :
  Y Pitch  :
***********************************************************/

#define __MXT224E_CONFIG__



/* SPT_USERDATA_T38 INSTANCE 0 */
#define T38_USERDATA0             0
#define T38_USERDATA1             4     /* CAL_THR */
#define T38_USERDATA2             15     /* num_of_antitouch */
#define T38_USERDATA3             0
#define T38_USERDATA4             0	/* MXT_ADR_T8_ATCHFRCCALRATIO for normal */
#define T38_USERDATA5             0
#define T38_USERDATA6             0
#define T38_USERDATA7			0


#define T7_IDLEACQINT             64
#define T7_ACTVACQINT             255
#define T7_ACTV2IDLETO            25

/* _GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
#define T8_CHRGTIME		45	   /* 6 - 60  * 83 ns */	// Charge-transfer dwell time
#define T8_CHRGTIME_TA		31	   /* 6 - 60  * 83 ns */
#define T8_ATCHDRIFT              0
#define T8_TCHDRIFT               5					// Touch drift time
#define T8_DRIFTST                1
#define T8_TCHAUTOCAL             0					// Touch Automatic Calibration
#define T8_SYNC                   0
#define T8_ATCHCALST			3				// Anti-touch calibration time
#define T8_ATCHCALSTHR			20				// Anti-touch Calibration suspend time
#define T8_ATCHFRCCALTHR		127		/* V2.0 added */
#define T8_ATCHFRCCALRATIO		127		/* V2.0 added */



/* TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0 */
//0x80 : Scan ena
//0x8  : Disable vector change, 0x2: Enable reporting, 0x1 : Enable the multi-touch
#define T9_CTRL                   0x8B		// Disable amplitude change : 0x1 << 3
#define T9_XORIGIN                0
#define T9_YORIGIN                0
#define T9_XSIZE                  18
#define T9_YSIZE                  11
#define T9_AKSCFG                 1
#define T9_BLEN                   0x10		// Gain of the analog circuits in front of the ADC [7:4]
#define T9_TCHTHR		  32 		// touch Threshold value
#define T9_TCHDI                  2
#define T9_ORIENT                 1		// 0x4 : Invert Y, 0x2 : Invert X, 0x1 : Switch
#define T9_MRGTIMEOUT             10
#define T9_MOVHYSTI               1		// Move hysteresis, initial
#define T9_MOVHYSTN               1		// Move hysteresis, next
#define T9_MOVFILTER              0x2F		// Filter Limit[6:4] , Adapt threshold [3:0]
#define T9_NUMTOUCH               10 
#define T9_MRGHYST                70		// Merge hysteresis
#define T9_MRGTHR                 70		// Merge threshold
#define T9_AMPHYST                10		// Amplitude hysteresis
#define T9_XRANGE                 (800-1)
#define T9_YRANGE                 (480-1)
#define T9_XLOCLIP                20
#define T9_XHICLIP                20
#define T9_YLOCLIP                40
#define T9_YHICLIP                31
#define T9_XEDGECTRL              143
#define T9_XEDGEDIST              40
#define T9_YEDGECTRL              143
#define T9_YEDGEDIST              80
#define T9_JUMPLIMIT              18
#define T9_TCHHYST                5	 /* V2.0 or MXT224E added */
#define T9_XPITCH                 50	 /* MXT224E added */
#define T9_YPITCH                 50	 /* MXT224E added */
#define T9_NEXTTCHDI              2

/* TOUCH_KEYARRAY_T15 */
#define T15_CTRL                  0x83 /* single key configuration*/  /* 0x03 = multi-key */
#define T15_XORIGIN               16
#define T15_XORIGIN_4KEY	  14
#define T15_YORIGIN		  11
#define T15_XSIZE		  2
#define T15_XSIZE_4KEY		  4
#define T15_YSIZE                 1
#define T15_AKSCFG                3
#define T15_BLEN                  0
#define T15_TCHTHR                60
#define T15_TCHDI                 3
#define T15_RESERVED_0            0
#define T15_RESERVED_1            0


/* SPT_COMMSCONFIG_T18 */
#define T18_CTRL                  0
#define T18_COMMAND               0



/* SPT_GPIOPWM_T19 INSTANCE 0 */
#define T19_CTRL                  0
#define T19_REPORTMASK            0
#define T19_DIR                   0
#define T19_INTPULLUP             0
#define T19_OUT                   0
#define T19_WAKE                  0
#define T19_PWM                   0
#define T19_PERIOD                0
#define T19_DUTY_0                0
#define T19_DUTY_1                0
#define T19_DUTY_2                0
#define T19_DUTY_3                0
#define T19_TRIGGER_0             0
#define T19_TRIGGER_1             0
#define T19_TRIGGER_2             0
#define T19_TRIGGER_3             0


/* TOUCH_PROXIMITY_T23 */
#define T23_CTRL                  0
#define T23_XORIGIN               0
#define T23_YORIGIN               0
#define T23_XSIZE                 0
#define T23_YSIZE                 0
#define T23_RESERVED              0
#define T23_BLEN                  0
#define T23_FXDDTHR               0
#define T23_FXDDI                 0
#define T23_AVERAGE               0
#define T23_MVNULLRATE            0
#define T23_MVDTHR                0


/* T24_[PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTANCE 0] */
#define T24_CTRL                  0
#define T24_NUMGEST               0
#define T24_GESTEN                0
#define T24_PROCESS               0
#define T24_TAPTO                 0
#define T24_FLICKTO               0
#define T24_DRAGTO                0
#define T24_SPRESSTO              0
#define T24_LPRESSTO              0
#define T24_REPPRESSTO            0
#define T24_FLICKTHR              0
#define T24_DRAGTHR               0
#define T24_TAPTHR                0
#define T24_THROWTHR              0


/* [SPT_SELFTEST_T25 INSTANCE 0] */
#define T25_CTRL                  0
#define T25_CMD                   0
#define T25_SIGLIM_0_UPSIGLIM     13500
#define T25_SIGLIM_0_LOSIGLIM     5500
#define T25_SIGLIM_1_UPSIGLIM     13500
#define T25_SIGLIM_1_LOSIGLIM     5500
#define T25_SIGLIM_2_UPSIGLIM     0
#define T25_SIGLIM_2_LOSIGLIM     0


/* PROCI_GRIPSUPPRESSION_T40 */

#define T40_CTRL                  0
#define T40_XLOGRIP               0
#define T40_XHIGRIP               0
#define T40_YLOGRIP               0
#define T40_YHIGRIP               0

/* PROCI_TOUCHSUPPRESSION_T42 */
#if 0
#define T42_CTRL                  3
#define T42_APPRTHR               30   /* 0 (TCHTHR/4), 1 to 255 */
#define T42_MAXAPPRAREA           64   /* 0 (40ch), 1 to 255 */
#define T42_MAXTCHAREA            80   /* 0 (35ch), 1 to 255 */
#define T42_SUPSTRENGTH           32   /* 0 (128), 1 to 255 */
#define T42_SUPEXTTO              0   /* 0 (never expires), 1 to 255 (timeout in cycles) */
#define T42_MAXNUMTCHS            0   /* 0 to 9 (maximum number of touches minus 1) */
#define T42_SHAPESTRENGTH         0   /* 0 (10), 1 to 31 */
#else
#define T42_CTRL                  0x03
#define T42_APPRTHR               60   /* 0 (TCHTHR/4), 1 to 255 */
#define T42_MAXAPPRAREA           50	/* 33    0 (40ch), 1 to 255 */
#define T42_MAXTCHAREA            50	/* 33    0 (35ch), 1 to 255 */
#define T42_SUPSTRENGTH           128   /* 0 (128), 1 to 255 */
#define T42_SUPEXTTO              0   /* 0 (never expires), 1 to 255 (timeout in cycles) */
#define T42_MAXNUMTCHS            0   /* 0 to 9 (maximum number of touches minus 1) */
#define T42_SHAPESTRENGTH         0   /* 0 (10), 1 to 31 */
#endif



/* SPT_CTECONFIG_T46  */
#define T46_CTRL                  0x04  /*Reserved */
#define T46_MODE                  2  /*0: 16X14Y, 1: 17X13Y, 2: 18X12Y, 3: 19X11Y, 4: 20X10Y, 5: 21X15Y, 6: 22X8Y, */
#define T46_IDLESYNCSPERX		26
#define T46_ACTVSYNCSPERX		26
#define T46_IDLESYNCSPERX_TA		35
#define T46_ACTVSYNCSPERX_TA		35
#define T46_ADCSPERSYNC           0
#define T46_PULSESPERADC          0  /*0:1  1:2   2:3   3:4 pulses */
#define T46_XSLEW                 1  /*0:500nsec,  1:350nsec */
#define T46_SYNCDELAY			  0

/* PROCI_STYLUS_T47 */
#define T47_CTRL                  0
#define T47_CONTMIN               0
#define T47_CONTMAX               0
#define T47_STABILITY             0
#define T47_MAXTCHAREA            0
#define T47_AMPLTHR               0
#define T47_STYSHAPE              0
#define T47_HOVERSUP              0
#define T47_CONFTHR               0
#define T47_SYNCSPERX             0


/* PROCG_NOISESUPPRESSION_T48  */
/* for TA */
#define T48_CTRL_TA                  3
#define T48_CFG_TA                   0x84
#define T48_CALCFG_TA                0x50
#define T48_BASEFREQ_TA              0
#define	T48_RESERVED0_TA             0
#define	T48_RESERVED1_TA             0
#define	T48_RESERVED2_TA             0
#define	T48_RESERVED3_TA             0
#define T48_MFFREQ_2_TA              0
#define T48_MFFREQ_3_TA              0
#define	T48_RESERVED4_TA             0
#define	T48_RESERVED5_TA             0
#define	T48_RESERVED6_TA             0
#define T48_GCACTVINVLDADCS_TA       6
#define T48_GCIDLEINVLDADCS_TA         6
#define	T48_RESERVED7_TA             0
#define	T48_RESERVED8_TA             0
#define T48_GCMAXADCSPERX_TA         100
#define T48_GCLIMITMIN_TA			6
#define T48_GCLIMITMAX_TA			64
#define T48_GCCOUNTMINTGT_TA		10
#define T48_MFINVLDDIFFTHR_TA		32
#define T48_MFINCADCSPXTHR_TA		5
#define T48_MFERRORTHR_TA			38
#define	T48_SELFREQMAX_TA			8
#define	T48_RESERVED9_TA             0
#define	T48_RESERVED10_TA            0
#define	T48_RESERVED11_TA            0
#define	T48_RESERVED12_TA           0
#define	T48_RESERVED13_TA            0
#define	T48_RESERVED14_TA            0
#define T48_BLEN_TA                    0
#define T48_TCHTHR_TA                   50
#define T48_TCHDI_TA                   2
#define T48_MOVHYSTI_TA                5
#define T48_MOVHYSTN_TA                2
#define	T48_MOVFILTER_TA             0x2F
#define T48_NUMTOUCH_TA                5
#define T48_MRGHYST_TA                 70
#define T48_MRGTHR_TA                  70
#define T48_XLOCLIP_TA                 15
#define T48_XHICLIP_TA                 20
#define T48_YLOCLIP_TA                 46
#define T48_YHICLIP_TA                 40
#define T48_XEDGECTRL_TA               146
#define T48_XEDGEDIST_TA               40
#define T48_YEDGECTRL_TA               149
#define T48_YEDGEDIST_TA               68
#define T48_JUMPLIMIT_TA               13
#define T48_TCHHYST_TA                35
#define T48_NEXTTCHDI_TA               2

/* for BATTERY */
#define	T48_CTRL			3
#define	T48_CFG				0x84
#define	T48_CALCFG                0x40
#define	T48_BASEFREQ              0
#define	T48_RESERVED0             0
#define	T48_RESERVED1             0
#define	T48_RESERVED2             0
#define	T48_RESERVED3             0
#define	T48_MFFREQ_2              0
#define	T48_MFFREQ_3              0
#define	T48_RESERVED4             0
#define	T48_RESERVED5             0
#define	T48_RESERVED6             0
#define	T48_GCACTVINVLDADCS       6
#define	T48_GCIDLEINVLDADCS         6
#define	T48_RESERVED7             0
#define	T48_RESERVED8             0
#define	T48_GCMAXADCSPERX         60
#define	T48_GCLIMITMIN			6
#define	T48_GCLIMITMAX			60
#define	T48_GCCOUNTMINTGT		10
#define	T48_MFINVLDDIFFTHR		32
#define	T48_MFINCADCSPXTHR		5
#define	T48_MFERRORTHR			38
#define	T48_SELFREQMAX			5
#define	T48_RESERVED9             0
#define	T48_RESERVED10            0
#define	T48_RESERVED11            0
#define	T48_RESERVED12            0
#define	T48_RESERVED13            0
#define	T48_RESERVED14            0
#define	T48_BLEN                    0x10
#define	T48_TCHTHR                  70
#define	T48_TCHDI                   2
#define	T48_MOVHYSTI                10
#define	T48_MOVHYSTN                3
#define	T48_MOVFILTER             0x2F
#define	T48_NUMTOUCH                10
#define	T48_MRGHYST                 70
#define	T48_MRGTHR                  70
#define T48_XLOCLIP                20
#define T48_XHICLIP                20
#define T48_YLOCLIP                40
#define T48_YHICLIP                31
#define T48_XEDGECTRL              143
#define T48_XEDGEDIST              40
#define T48_YEDGECTRL              143
#define T48_YEDGEDIST              80

#define	T48_JUMPLIMIT               9
#define	T48_TCHHYST                 5
#define	T48_NEXTTCHDI               2

#define	T48_CHGON_BIT		0x20
/********************* END  *********************/
