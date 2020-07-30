
/*----------------------------------------------------------------------------
  type definitions
  ----------------------------------------------------------------------------*/
#define NUM_OF_TOUCH_OBJECTS	    2

typedef struct
{
	uint8_t reset;       	/*!< Force chip reset             */
	uint8_t backupnv;    	/*!< Force backup to eeprom/flash */
	uint8_t calibrate;   	/*!< Force recalibration          */
	uint8_t reportall;   	/*!< Force all objects to report  */
	uint8_t reserved;     	/*!< Turn on output of debug data */
	uint8_t diagnostic;  	/*!< Controls the diagnostic object */
	uint8_t reserved_1;  	/*!< Controls the diagnostic object */
} gen_commandprocessor_t6_config_t;

typedef struct
{
	uint8_t idleacqint;    /*!< Idle power mode sleep length in ms           */
	uint8_t actvacqint;    /*!< Active power mode sleep length in ms         */
	uint8_t actv2idleto;   /*!< Active to idle power mode delay length in units of 0.2s*/
	uint8_t cfg;
	uint8_t cfg2;
} gen_powerconfig_t7_config_t;

typedef struct
{ 
	uint8_t chrgtime;          /*!< Charge-transfer dwell time             */  
	uint8_t atchdrift;          /*!< reserved                               */
	uint8_t tchdrift;          /*!< Touch drift compensation period        */
	uint8_t driftst;           /*!< Drift suspend time                     */
	uint8_t tchautocal;        /*!< Touch automatic calibration delay in units of 0.2s*/
	uint8_t sync;              /*!< Measurement synchronisation control    */
	uint8_t atchcalst;         /*!< recalibration suspend time after last detection */
	uint8_t atchcalsthr;       /*!< Anti-touch calibration suspend threshold */
	uint8_t atchfrccalthr;     /*!< Anti-touch force calibration threshold */
	int8_t  atchfrccalratio;   /*!< Anti-touch force calibration ratio */  
	uint8_t measallow;         /*!< reserved            */
	uint8_t measidledef;
	uint8_t measactivedef;
	uint8_t refmode;
	uint8_t cfg;
} gen_acquisitionconfig_t8_config_t;

typedef struct
{
	/* Key Array Configuration */
	uint8_t ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */
	uint8_t xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
	uint8_t yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
	uint8_t xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
	uint8_t ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */
	uint8_t akscfg;             /*!< Adjacent key suppression config     */
	uint8_t blen;               /*!< ACMASK Burst length for all object channels*/
	uint8_t tchthr;             /*!< ACMASK LCMASK Threshold for all object channels   */
	uint8_t tchdi;              /*!< Detect integration config           */
	uint8_t tchhyst;        /*!< Spare x2 */
	uint8_t reserved;
} touch_keyarray_t15_config_t;

typedef struct
{
	uint8_t  ctrl;
	uint8_t  cmd;
} spt_commsconfig_t18_config_t;

typedef struct
{
	uint8_t  ctrl;
	uint8_t  reportmask;
	uint8_t  dir;
	uint8_t  intpullup;
	uint8_t  out;
	uint8_t  wake;
} spt_gpiopwm_t19_config_t;

typedef struct
{
	uint16_t upsiglim;              /* LCMASK */
	uint16_t losiglim;              /* LCMASK */

} siglim_t;

/*! = Config Structure = */

typedef struct
{
	uint8_t  ctrl;                 /* LCENABLE */
	uint8_t  cmd;
#if(NUM_OF_TOUCH_OBJECTS)
	siglim_t siglim[NUM_OF_TOUCH_OBJECTS];   /* LCMASK */
#endif
	uint8_t  pindwellus;
#if(NUM_OF_TOUCH_OBJECTS)  
	uint16_t sigrangelim[NUM_OF_TOUCH_OBJECTS];   /* LCMASK */
#endif
} spt_selftest_t25_config_t;

typedef struct
{
	uint8_t mode;
	uint8_t page;
	uint8_t data[128];

} debug_diagnositc_t37_t;

typedef struct
{
	uint8_t mode;
	uint8_t page;
	int8_t data[128];

} debug_diagnositc_t37_delta_t;

typedef struct
{
	uint8_t mode;
	uint8_t page;
	uint16_t data[64];

} debug_diagnositc_t37_reference_t;

typedef struct
{
	uint8_t data[64];

} spt_userdata_t38_config_t;

typedef struct
{
	uint8_t ctrl;          /*!< Reserved/ GRIPMODE/ Reserved/ ENABLE */
	uint8_t xlogrip;       /*!< Grip suppression X low boundary   */
	uint8_t xhigrip;       /*!< Grip suppression X high boundary  */
	uint8_t ylogrip;       /*!< Grip suppression Y low boundary   */
	uint8_t yhigrip;       /*!< Grip suppression Y high boundary  */
} proci_gripsuppression_t40_config_t;

typedef struct
{
	uint8_t ctrl;            /*!< ctrl field reserved for future expansion */
	uint8_t apprthr;         /*!< Approach threshold */
	uint8_t maxapprarea;     /*!< Maximum approach area threshold */
	uint8_t maxtcharea;      /*!< Maximum touch area threshold */
	uint8_t supstrength;     /*!< Suppression aggressiveness */
	uint8_t supextto;        /*!< Suppression extension timeout */ 
	uint8_t maxnumtchs;      /*!< Maximum touches */
	uint8_t shapestrength;   /*!< Shaped-based aggressiveness */
	uint8_t supdist;
	uint8_t disthyst;
	uint8_t maxscrnarea;
	uint8_t cfg;
	uint8_t maxfngrsarea;
} proci_touchsuppression_t42_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t reserved;
	uint16_t xlength;
	uint16_t ylength;
	uint8_t rwkrate;
	uint8_t reserved_1;
	uint8_t heightoffset;
	uint8_t reserved_2;
	uint8_t widthoffset;
	uint8_t cfg;
	uint8_t heightsf;
	uint8_t widthsf;
	uint8_t edgesnpcfg;
} spt_digitizer_t43_config_t;

typedef struct
{
	uint8_t count;
}spt_messagecount_t44_config_t;

typedef struct
{
	uint8_t ctrl;          
	uint8_t mode;                                    
	uint8_t idlesyncsperx; 
	uint8_t actvsyncsperx; 
	uint8_t adcspersync;        
	uint8_t pulsesperadc;            
	uint8_t xslew;                             
	uint16_t syncdelay; 
	uint8_t xvoltage;	
	uint8_t adcctrl;
	uint8_t reserved;
}spt_cteconfig_t46_config_t;

typedef struct
{
	uint8_t  ctrl;          /*!< Reserved ENABLE            */   
	uint8_t  contmin;       /*!< Minimum contact diameter   */
	uint8_t  contmax;       /*!< Maximum contact diameter   */
	uint8_t  stability;     /*!< Stability                  */
	uint8_t  maxtcharea;    /*!< Maximum touch are          */
	uint8_t  amplthr;       /*!< Maximum touch amplitude    */
	uint8_t  styshape;      /*!< Stylus shape adjustment    */
	uint8_t  hoversup;      /*!< Hovering finger suppression*/
	uint8_t  confthr;       /*!< Confidence threshold       */
	uint8_t  syncsperx;     /*!< ADC sets per X             */
	int8_t   xposadj;
	int8_t   yposadj;	
	uint8_t  cfg;
	uint8_t  reserved[7];
	uint8_t  supstyto;		/*!< Suppression timeout        */
	uint8_t  maxnumsty;		/*!< Maximum stylus touches     */
	uint8_t  xedgectrl;
	uint8_t  xedgedist;
	uint8_t  yedgectrl;
	uint8_t  yedgedist;
	uint8_t  supto;
	uint8_t  supclassmode;
}proci_stylus_t47_config_t;

typedef struct
{
	uint8_t ctrl;				
	uint8_t targetthr;	
	uint8_t thradjlim;
	uint8_t resetsteptime;
	uint8_t forcechgdist;
	uint8_t forcechgtime;
	int8_t lowestthr;
} proci_adaptivethreshold_t55_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t command;
	uint8_t optint; 
	uint8_t inttime; 
	uint8_t intdelay0;
	uint8_t intdelay1;	
	uint8_t intdelay2;	
	uint8_t intdelay3;	
	uint8_t intdelay4;	
	uint8_t intdelay5;	
	uint8_t intdelay6;	
	uint8_t intdelay7;	
	uint8_t intdelay8;		
	uint8_t intdelay9;	
	uint8_t intdelay10;
	uint8_t intdelay11;
	uint8_t intdelay12;
	uint8_t intdelay13;
	uint8_t intdelay14;
	uint8_t intdelay15;	
	uint8_t intdelay16;
	uint8_t intdelay17;
	uint8_t intdelay18;	
	uint8_t intdelay19;
	uint8_t intdelay20;
	uint8_t intdelay21;	
	uint8_t intdelay22;
	uint8_t intdelay23;
	uint8_t intdelay24;
	uint8_t intdelay25;
	uint8_t intdelay26;
	uint8_t intdelay27;
	uint8_t intdelay28;
	uint8_t intdelay29;
	uint8_t intdelay30; 
	uint8_t intdelay31;	
} proci_shieldless_t56_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t cmd;
	uint8_t mode;
	uint16_t period;
} spt_timer_t61_config_t;

typedef struct {
	uint8_t ctrl;          
	uint8_t gradthr;	
	uint16_t ylonoisemul; 
	uint16_t ylonoisediv; 
	uint16_t yhinoisemul; 
	uint16_t yhinoisediv; 
	uint8_t lpfiltcoef;          
	uint16_t forcescale; 
	uint8_t forcethr;	
	uint8_t forcethrhyst;	
	uint8_t forcedi;	
	uint8_t forcehyst;
	uint8_t atchratio;
	uint16_t totvarlim;
	uint8_t reserved[3];
}proci_lensbending_t65_config_t;

typedef struct
{
	uint8_t ctrl;				
	uint8_t fcalfailthr;	
	uint8_t fcaldriftcnt;
} spt_goldenreferences_t66_config_t;

typedef struct
{
	uint8_t ctrl;
	uint16_t event;	
	uint8_t objtype;
	uint8_t reserved;
	uint8_t objinst;
	uint8_t dstoffset;
	uint16_t srcoffset;
	uint8_t length;
}spt_dynamiccfgcontroller_t70_config_t;

typedef struct
{
	uint8_t data[200];
}spt_dynamiccfgcontainer_t71_config_t;

typedef struct {
	uint8_t ctrl;
	uint8_t calcfg1;
	uint8_t cfg1;
	uint8_t cfg2;
	uint8_t debugcfg;
	uint8_t hopcnt;
	uint8_t hopcntper;
	uint8_t hopevalto;
	uint8_t hopst;	
	uint8_t nlgaindualx; 
	uint8_t minnlthr; 
	uint8_t incnlthr; 
	uint8_t fallnlthr; 
	uint8_t nlthrmargin; 
	uint8_t minthradj;
	uint8_t nlthrlimit;
	uint8_t bgscan;
	uint8_t nlgainsingx;
	uint8_t blknlthr;
	uint8_t cfg3;
	uint8_t stabctrl;
	uint8_t stabfreq[5];
	uint8_t stabtchapx[5];
	uint8_t stabnotchapx[5];
	uint8_t stabpc;
	uint8_t reserved;
	uint8_t stabhighnlthr;
	uint8_t stabcnt;
	uint8_t noisctrl;
	uint8_t noisfreq[5];
	uint8_t noistchapx[5];
	uint8_t noisnotchapx[5];
	uint8_t noispc;
	uint8_t noislownlthr;
	uint8_t noishighnlthr;
	uint8_t noiscnt;
	uint8_t vnoictrl;
	uint8_t vnoifreq[5];
	uint8_t vnoitchapx[5];
	uint8_t vnoinotchapx[5];
	uint8_t vnoipc;
	uint8_t vnoilownlthr;
	uint8_t reserved_1;
	uint8_t vnoicnt;
	uint8_t reserved_2;
	uint8_t notchmindiff;
	uint8_t tchmindiff;
	uint8_t notchminhop;
	uint8_t tchminhop;
} procg_noisesuppression_t72_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t minarea;	
	uint8_t confthr;
	uint8_t mindist;
	uint8_t glovemodeto;
	uint8_t supto;
	uint8_t syncsperx;
	uint8_t hithrmrgn;
	uint8_t reserved[4];
}proci_glovedetection_t78_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t compgain;	
	uint8_t targetdelta;
	uint8_t compthr;
	uint8_t atchthr;
	uint8_t moistcfg;
	uint8_t moistdto;
	uint8_t reserved;
	uint8_t reserved_1;
}proci_retransmissioncompensation_t80_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t zonethr;	
	uint8_t direl;
	uint8_t dto;
}proci_gesturepocessor_t84_config_t;

typedef struct {
	/* Screen Configuration */
	uint8_t ctrl;            /*!< ACENABLE LCENABLE Main configuration field  */
	uint8_t cfg1;
	uint8_t scraux;
	uint8_t tchaux;
	uint8_t tcheventcfg;
	uint8_t akscfg;
	uint8_t numtch;
	uint8_t xycfg;
	uint8_t xorigin;         /*!< LCMASK ACMASK Object x start position on matrix  */
	uint8_t xsize;           /*!< LCMASK ACMASK Object x size (i.e. width)         */
	uint8_t xpitch;
	int8_t xloclip;       /*!< LCMASK */
	int8_t xhiclip;       /*!< LCMASK */
	uint16_t xrange;       /*!< LCMASK */
	uint8_t xedgecfg;     /*!< LCMASK */
	uint8_t xedgedist;     /*!< LCMASK */
	uint8_t dxxedgecfg;     /*!< LCMASK */
	uint8_t dxxedgedist;     /*!< LCMASK */
	uint8_t yorigin;         /*!< LCMASK ACMASK Object y start position on matrix  */
	uint8_t ysize;           /*!< LCMASK ACMASK Object y size (i.e. height)        */
	uint8_t ypitch;
	int8_t yloclip;       /*!< LCMASK */
	int8_t yhiclip;       /*!< LCMASK */
	uint16_t yrange;       /*!< LCMASK */
	uint8_t yedgecfg;     /*!< LCMASK */
	uint8_t yedgedist;     /*!< LCMASK */
	uint8_t gain;
	uint8_t dxgain;
	uint8_t tchthr;          /*!< ACMASK Threshold for all object channels   */
	uint8_t tchhyst;       /* Touch threshold hysteresis */
	uint8_t intthr;
	uint8_t noisesf;
	uint8_t cutoffthr;
	uint8_t mrgthr;     /*!< The threshold for the point when two peaks are
						 *   considered one touch */
	uint8_t mrgthradjstr;
	uint8_t mrghyst;    /*!< The hysteresis applied on top of the merge threshold
						 *   to stop oscillation */
	uint8_t dxthrsf;
	uint8_t tchdidown;
	uint8_t tchdiup;
	uint8_t nexttchdi;
	uint8_t reserved;
	uint8_t jumplimit;
	uint8_t movfilter;
	uint8_t movsmooth;
	uint8_t movpred;
	uint16_t movhysti;
	uint16_t movhystn;
	uint8_t amplhyst;
	uint8_t scrareahyst;
	uint8_t intthrhyst;
} touch_multitouchscreen_t100_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t xloclip;	
	uint8_t xhiclip;
	uint8_t xedgecfg;
	uint8_t xedgedist;
	uint8_t xgain;
	uint8_t xhvrthr;
	uint8_t xhvrhyst;
	uint8_t yloclip;
	uint8_t yhiclip;
	uint8_t yedgecfg;
	uint8_t yedgedist;
	uint8_t ygain;
	uint8_t yhvthr;	
	uint8_t yhvrhyst;
	uint8_t hvrdi;
	uint8_t confthr;
	uint8_t movfilter;
	uint8_t movsmooth;
	uint8_t movpred;
	uint16_t movhysti;
	uint16_t movhystn;
	uint8_t hvraux;
	uint8_t reserved;
}spt_touchscreenhover_t101_config_t;

typedef struct
{
	uint8_t ctrl;
	uint8_t xgain;	
	uint8_t xtchthr;
	uint8_t xtchhyst;
	uint8_t xintthr;
	uint8_t xinthyst;
	uint8_t ygain;	
	uint8_t ytchthr;
	uint8_t ytchhyst;
	uint8_t yintthr;
	uint8_t yinthyst;
}spt_auxtouchconfig_t104_config_t;

typedef struct
{
	uint8_t	ctrl;
	uint8_t	calcfg1;
	uint8_t	cfg1;
	uint8_t	cfg2;
	uint8_t	cfg3;
	uint8_t	nlgain;
	uint8_t nlgclim;
	uint8_t	iircoeff;	
	uint8_t	minnltddiff;
	uint8_t	minnltdhop;
	uint8_t	reserved;
	uint8_t	reserved_1;
	uint8_t	hopst;
	uint8_t	stabctrl;
	uint8_t	stabfreq[5];
	uint8_t	stabtchapx[5];
	uint8_t	stabnotchapx[5];
	uint8_t	reserved_2;
	uint8_t	reserved_3;
	uint8_t	stabhinltdthr;
	uint8_t	reserved_4;
	uint8_t	noisctrl;		
	uint8_t	noisfreq[5]; 	
	uint8_t	noistchapx[5]; 
	uint8_t	noisnotchapx[5];
	uint8_t	reserved_5;
	uint8_t	noislonltdthr;  
	uint8_t	noishinltdthr;
	uint8_t	noiscnt; 		
	uint8_t	vnoictrl;	
	uint8_t	vnoifreq[5];
	uint8_t	vnoitchapx[5];
	uint8_t	vnoinotchapx[5]; 
	uint8_t	reserved_6;
	uint8_t	vnoilonltdthr;
	uint8_t	reserved_7;	
	uint8_t	vnoicnt;
	uint8_t	blknltdthr;
	uint8_t	reserved_8;  
}proci_noisesupselfcap_t108_config_t;

typedef struct
{
	uint8_t	ctrl;
	uint8_t	dbgctrl;
	uint8_t	cmdonreset;
	uint8_t	cmd;
	uint16_t lfcompsfx;
	uint16_t lfcompsfy;
	uint8_t	reserved;
}spt_selfcapglobalconfig_t109_config_t;

#define T110_PARAM_SIZE	20
typedef struct
{
	uint16_t	params[T110_PARAM_SIZE];
}spt_selfcaptuningparams_t110_config_t;

typedef struct
{
	uint8_t	ctrl;
	uint8_t	dbgctrl;
	uint8_t	inttime;
	uint8_t	delaytime;
	uint8_t	idlesyncsperl;
	uint8_t	actvsyncsperl;
	uint8_t	drift;
	uint8_t	driftst;
	uint8_t	driftsthrsf;
	uint8_t	calecstr;
	uint8_t	filter;
	uint8_t	filtcfg;
	uint8_t	dyniirthru;
	uint8_t	dyniirthrl;
	uint8_t	dyniirclmp;
	uint8_t	rerserved;
}spt_selfcapconfig_t111_config_t;

typedef struct
{
	uint8_t	ctrl;
	uint8_t	gainx;
	uint8_t	reserved;
}spt_proxmeasureconfig_t113_config_t;

#include "atmel_641t_cfg_multi_ef71.h"

/******************************************************************************
 *       Object table init
 * *****************************************************************************/
//General Object
gen_powerconfig_t7_config_t 			power_config = {0};  
gen_acquisitionconfig_t8_config_t 		acquisition_config = {0};

//Touch Object
touch_keyarray_t15_config_t 			keyarray_config = {0};  
touch_multitouchscreen_t100_config_t 	touchscreen_config = {0};

//Support Objects
spt_commsconfig_t18_config_t 			    comms_config = {0};
spt_gpiopwm_t19_config_t              gpiopwm_config = {0};
spt_selftest_t25_config_t 				    selftest_config = {0};  
spt_cteconfig_t46_config_t              cte_t46_config = {0};
spt_timer_t61_config_t					spt_timer_t61_config = {0};
spt_goldenreferences_t66_config_t		goldenreferences_t66_config = {0};
spt_dynamiccfgcontroller_t70_config_t dynamiccfgcontroller_t70_config= {0};
spt_dynamiccfgcontainer_t71_config_t dynamiccfgcontainer_t71_config={{0},};
spt_auxtouchconfig_t104_config_t			auxtouchconfig_t104_config = {0};
spt_selfcapglobalconfig_t109_config_t	selfcapglobalconfig_t109_config = {0};
spt_selfcaptuningparams_t110_config_t	selfcaptuningparams_t110_config ={{0},};
spt_selfcapconfig_t111_config_t			selfcapconfig_t111_config = {0};
spt_proxmeasureconfig_t113_config_t		proxmeasureconfig_t113_config = {0};

//Signal Processing Objects
proci_gripsuppression_t40_config_t      gripsuppression_t40_config = {0};       
proci_touchsuppression_t42_config_t     touchsuppression_t42_config = {0};
proci_stylus_t47_config_t               stylus_t47_config = {0};
proci_adaptivethreshold_t55_config_t 	proci_adaptivethreshold_t55_config = {0}; 
proci_shieldless_t56_config_t			proci_shieldless_t56_config = {0}; 
proci_lensbending_t65_config_t			lensbending_t65_config = {0};
procg_noisesuppression_t72_config_t     noisesuppression_t72_config = {0};
proci_glovedetection_t78_config_t       glovedetection_t78_config = {0};
proci_retransmissioncompensation_t80_config_t	retransmissioncompensation_t80_config = {0};
proci_noisesupselfcap_t108_config_t		noisesupselfcap_t108_config  = {0};

// none object firmware 3.0
proci_gesturepocessor_t84_config_t gesturepocessor_t84_config = {0};
spt_touchscreenhover_t101_config_t touchscreenhover_t101_config = {0};

//*****************************************************************************
//		Config Table for Touch Monitor Interface by SWKim
//*****************************************************************************
typedef enum {
	UINT8 = 0,
	UINT16 = 1,
	INT8 = 2,
	INT16 = 3,
}enum_size;

typedef struct {
	int16_t* value;
	enum_size size;
}config_table_element;

config_table_element t7_power_config_table[] = { 
	{(int16_t*)&power_config.idleacqint, UINT8},
	{(int16_t*)&power_config.actvacqint, UINT8},
	{(int16_t*)&power_config.actv2idleto, UINT8},
	{(int16_t*)&power_config.cfg, UINT8},
	{(int16_t*)&power_config.cfg2, UINT8},
};

config_table_element t8_acquisition_config_table[] = { 
	{(int16_t*)&acquisition_config.chrgtime, UINT8},
	{(int16_t*)&acquisition_config.atchdrift, UINT8},
	{(int16_t*)&acquisition_config.tchdrift, UINT8},
	{(int16_t*)&acquisition_config.driftst, UINT8},
	{(int16_t*)&acquisition_config.tchautocal, UINT8},
	{(int16_t*)&acquisition_config.sync, UINT8},
	{(int16_t*)&acquisition_config.atchcalst, UINT8},
	{(int16_t*)&acquisition_config.atchcalsthr, UINT8},
	{(int16_t*)&acquisition_config.atchfrccalthr, UINT8},
	{(int16_t*)&acquisition_config.atchfrccalratio, INT8},
	{(int16_t*)&acquisition_config.measallow, UINT8},
	{(int16_t*)&acquisition_config.measidledef, UINT8},
	{(int16_t*)&acquisition_config.measactivedef, UINT8},
	{(int16_t*)&acquisition_config.refmode, UINT8},
	{(int16_t*)&acquisition_config.cfg, UINT8},
};

config_table_element t15_keyarray_config_table[] = { 
	{(int16_t*)&keyarray_config.ctrl, UINT8},
	{(int16_t*)&keyarray_config.xorigin, UINT8},
	{(int16_t*)&keyarray_config.yorigin, UINT8},
	{(int16_t*)&keyarray_config.xsize, UINT8},
	{(int16_t*)&keyarray_config.ysize, UINT8},
	{(int16_t*)&keyarray_config.akscfg, UINT8},
	{(int16_t*)&keyarray_config.blen, UINT8},
	{(int16_t*)&keyarray_config.tchthr, UINT8},
	{(int16_t*)&keyarray_config.tchdi, UINT8},
	{(int16_t*)&keyarray_config.tchhyst, UINT8},
	{(int16_t*)&keyarray_config.reserved, UINT8},
};

config_table_element t18_commsconfig_config_table[] = { 
	{(int16_t*)&comms_config.ctrl, UINT8},
	{(int16_t*)&comms_config.cmd, UINT8},
};

config_table_element t19_gpiopwm_config_table[] = { 
	{(int16_t*)&gpiopwm_config.ctrl, UINT8},
	{(int16_t*)&gpiopwm_config.reportmask, UINT8},
	{(int16_t*)&gpiopwm_config.dir, UINT8},	
	{(int16_t*)&gpiopwm_config.intpullup, UINT8},
	{(int16_t*)&gpiopwm_config.out, UINT8},
	{(int16_t*)&gpiopwm_config.wake, UINT8},
};

config_table_element t25_selftest_config_table[] = {
	{(int16_t*)&selftest_config.ctrl, UINT8},
	{(int16_t*)&selftest_config.cmd, UINT8},
#if NUM_OF_TOUCH_OBJECTS
	{(int16_t*)&selftest_config.siglim[0].upsiglim, UINT16},
	{0},
	{(int16_t*)&selftest_config.siglim[0].losiglim, UINT16},
	{0},
	{(int16_t*)&selftest_config.siglim[1].upsiglim, UINT16},
	{0},
	{(int16_t*)&selftest_config.siglim[1].losiglim, UINT16},
	{0},
#endif
	{(int16_t*)&selftest_config.pindwellus, UINT8},
#if NUM_OF_TOUCH_OBJECTS
	{(int16_t*)&selftest_config.sigrangelim[0], UINT16},
	{0},
	{(int16_t*)&selftest_config.sigrangelim[1], UINT16},
	{0},
#endif
};

config_table_element t40_gripsuppression_config_table[] = {
	{(int16_t*)&gripsuppression_t40_config.ctrl, UINT8},
	{(int16_t*)&gripsuppression_t40_config.xlogrip, UINT8},
	{(int16_t*)&gripsuppression_t40_config.xhigrip, UINT8},
	{(int16_t*)&gripsuppression_t40_config.ylogrip, UINT8},
	{(int16_t*)&gripsuppression_t40_config.yhigrip, UINT8},
};
config_table_element t42_touchsuppression_config_table[] = {
	{(int16_t*)&touchsuppression_t42_config.ctrl, UINT8},
	{(int16_t*)&touchsuppression_t42_config.apprthr, UINT8},
	{(int16_t*)&touchsuppression_t42_config.maxapprarea, UINT8},
	{(int16_t*)&touchsuppression_t42_config.maxtcharea, UINT8},
	{(int16_t*)&touchsuppression_t42_config.supstrength, UINT8},
	{(int16_t*)&touchsuppression_t42_config.supextto, UINT8},
	{(int16_t*)&touchsuppression_t42_config.maxnumtchs, UINT8},
	{(int16_t*)&touchsuppression_t42_config.shapestrength, UINT8},
	{(int16_t*)&touchsuppression_t42_config.supdist, UINT8},
	{(int16_t*)&touchsuppression_t42_config.disthyst, UINT8},
	{(int16_t*)&touchsuppression_t42_config.maxscrnarea, UINT8},
	{(int16_t*)&touchsuppression_t42_config.cfg, UINT8},
	{(int16_t*)&touchsuppression_t42_config.maxfngrsarea, UINT8},

};
config_table_element t46_cteconfig_config_table[] = {
	{(int16_t*)&cte_t46_config.ctrl, UINT8},
	{(int16_t*)&cte_t46_config.mode, UINT8},
	{(int16_t*)&cte_t46_config.idlesyncsperx, UINT8},
	{(int16_t*)&cte_t46_config.actvsyncsperx, UINT8},
	{(int16_t*)&cte_t46_config.adcspersync, UINT8},
	{(int16_t*)&cte_t46_config.pulsesperadc, UINT8}, 
	{(int16_t*)&cte_t46_config.xslew, UINT8},  
	{(int16_t*)&cte_t46_config.syncdelay, UINT16},
	{0},
	{(int16_t*)&cte_t46_config.xvoltage, UINT8},
	{(int16_t*)&cte_t46_config.adcctrl, UINT8},
	{(int16_t*)&cte_t46_config.reserved, UINT8},
};

config_table_element t47_stylus_config_table[] = {
	{(int16_t*)&stylus_t47_config.ctrl, UINT8},
	{(int16_t*)&stylus_t47_config.contmin, UINT8},
	{(int16_t*)&stylus_t47_config.contmax, UINT8},
	{(int16_t*)&stylus_t47_config.stability, UINT8},
	{(int16_t*)&stylus_t47_config.maxtcharea, UINT8},
	{(int16_t*)&stylus_t47_config.amplthr, UINT8},
	{(int16_t*)&stylus_t47_config.styshape, UINT8},
	{(int16_t*)&stylus_t47_config.hoversup, UINT8},
	{(int16_t*)&stylus_t47_config.confthr, UINT8},
	{(int16_t*)&stylus_t47_config.syncsperx, UINT8},
	{(int16_t*)&stylus_t47_config.xposadj, INT8},
	{(int16_t*)&stylus_t47_config.yposadj, INT8},
	{(int16_t*)&stylus_t47_config.cfg, UINT8},
	{(int16_t*)&stylus_t47_config.reserved[0], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[1], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[2], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[3], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[4], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[5], UINT8},
	{(int16_t*)&stylus_t47_config.reserved[6], UINT8},
	{(int16_t*)&stylus_t47_config.supstyto, UINT8},
	{(int16_t*)&stylus_t47_config.maxnumsty, UINT8},
	{(int16_t*)&stylus_t47_config.xedgectrl, UINT8},
	{(int16_t*)&stylus_t47_config.xedgedist, UINT8},
	{(int16_t*)&stylus_t47_config.yedgectrl, UINT8},
	{(int16_t*)&stylus_t47_config.yedgedist, UINT8},
	{(int16_t*)&stylus_t47_config.supto, UINT8},
	{(int16_t*)&stylus_t47_config.supclassmode, UINT8},
};

config_table_element t56_shieldless_config_table[] = {
	{(int16_t*)&proci_shieldless_t56_config.ctrl, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.command, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.optint, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.inttime, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay0, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay1, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay2, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay3, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay4, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay5, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay6, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay7, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay8, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay9, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay10, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay11, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay12, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay13, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay14, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay15, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay16, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay17, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay18, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay19, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay20, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay21, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay22, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay23, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay24, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay25, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay26, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay27, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay28, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay29, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay30, UINT8},
	{(int16_t*)&proci_shieldless_t56_config.intdelay31, UINT8},
};

config_table_element t61_timer_config[] = {
	{(int16_t*)&spt_timer_t61_config.ctrl, UINT8},
	{(int16_t*)&spt_timer_t61_config.cmd, UINT8},
	{(int16_t*)&spt_timer_t61_config.mode, UINT8},
	{(int16_t*)&spt_timer_t61_config.period, UINT16},
	{0},
};

config_table_element t65_lensbending_config_table[] = {
	{(int16_t*)&lensbending_t65_config.ctrl, UINT8},
	{(int16_t*)&lensbending_t65_config.gradthr, UINT8},
	{(int16_t*)&lensbending_t65_config.ylonoisemul, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.ylonoisediv, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.yhinoisemul, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.yhinoisediv, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.lpfiltcoef, UINT8},
	{(int16_t*)&lensbending_t65_config.forcescale, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.forcethr, UINT8},
	{(int16_t*)&lensbending_t65_config.forcethrhyst, UINT8},
	{(int16_t*)&lensbending_t65_config.forcedi, UINT8},
	{(int16_t*)&lensbending_t65_config.forcehyst, UINT8},
	{(int16_t*)&lensbending_t65_config.atchratio, UINT8},
	{(int16_t*)&lensbending_t65_config.totvarlim, UINT16},
	{0},
	{(int16_t*)&lensbending_t65_config.reserved[0], UINT8},
	{(int16_t*)&lensbending_t65_config.reserved[1], UINT8},
	{(int16_t*)&lensbending_t65_config.reserved[2], UINT8},
};

config_table_element t70_dynamiccfgcontroller_config_table[] = {
	{(int16_t*)&dynamiccfgcontroller_t70_config.ctrl, UINT8},
	{(int16_t*)&dynamiccfgcontroller_t70_config.event, UINT16},
	{0},
	{(int16_t*)&dynamiccfgcontroller_t70_config.objtype, UINT8},
	{(int16_t*)&dynamiccfgcontroller_t70_config.reserved, UINT8},
	{(int16_t*)&dynamiccfgcontroller_t70_config.objinst, UINT8},
	{(int16_t*)&dynamiccfgcontroller_t70_config.dstoffset, UINT8},
	{(int16_t*)&dynamiccfgcontroller_t70_config.srcoffset, UINT16},
	{0},
	{(int16_t*)&dynamiccfgcontroller_t70_config.length, UINT8},
};

config_table_element t72_noisesuppression_config_table[] = {
	{(int16_t*)&noisesuppression_t72_config.ctrl, UINT8},
	{(int16_t*)&noisesuppression_t72_config.calcfg1, UINT8},
	{(int16_t*)&noisesuppression_t72_config.cfg1, UINT8},
	{(int16_t*)&noisesuppression_t72_config.cfg2, UINT8},
	{(int16_t*)&noisesuppression_t72_config.debugcfg, UINT8},
	{(int16_t*)&noisesuppression_t72_config.hopcnt, UINT8},
	{(int16_t*)&noisesuppression_t72_config.hopcntper, UINT8},
	{(int16_t*)&noisesuppression_t72_config.hopevalto, UINT8},
	{(int16_t*)&noisesuppression_t72_config.hopst, UINT8},
	{(int16_t*)&noisesuppression_t72_config.nlgaindualx, UINT8},
	{(int16_t*)&noisesuppression_t72_config.minnlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.incnlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.fallnlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.nlthrmargin, UINT8},
	{(int16_t*)&noisesuppression_t72_config.minthradj, UINT8},
	{(int16_t*)&noisesuppression_t72_config.nlthrlimit, UINT8},
	{(int16_t*)&noisesuppression_t72_config.bgscan, UINT8},
	{(int16_t*)&noisesuppression_t72_config.nlgainsingx, UINT8},
	{(int16_t*)&noisesuppression_t72_config.blknlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.cfg3, UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabctrl, UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabfreq[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabfreq[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabfreq[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabfreq[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabfreq[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabtchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabtchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabtchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabtchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabtchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabnotchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabnotchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabnotchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabnotchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabnotchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabpc, UINT8},
	{(int16_t*)&noisesuppression_t72_config.reserved, UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabhighnlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.stabcnt, UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisctrl, UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisfreq[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisfreq[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisfreq[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisfreq[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisfreq[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noistchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noistchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noistchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noistchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noistchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisnotchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisnotchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisnotchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisnotchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noisnotchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.noispc, UINT8},
	{(int16_t*)&noisesuppression_t72_config.noislownlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.noishighnlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.noiscnt, UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoictrl, UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoifreq[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoifreq[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoifreq[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoifreq[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoifreq[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoitchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoitchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoitchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoitchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoitchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoinotchapx[0], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoinotchapx[1], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoinotchapx[2], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoinotchapx[3], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoinotchapx[4], UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoipc, UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoilownlthr, UINT8},
	{(int16_t*)&noisesuppression_t72_config.reserved_1, UINT8},
	{(int16_t*)&noisesuppression_t72_config.vnoicnt, UINT8},
	{(int16_t*)&noisesuppression_t72_config.reserved_2, UINT8},
	{(int16_t*)&noisesuppression_t72_config.notchmindiff, UINT8},
	{(int16_t*)&noisesuppression_t72_config.tchmindiff, UINT8},
	{(int16_t*)&noisesuppression_t72_config.notchminhop, UINT8},
	{(int16_t*)&noisesuppression_t72_config.tchminhop, UINT8},
};

config_table_element t78_glovedetection_config_table[] = {
	{(int16_t*)&glovedetection_t78_config.ctrl, UINT8},
	{(int16_t*)&glovedetection_t78_config.minarea, UINT8},
	{(int16_t*)&glovedetection_t78_config.confthr, UINT8},
	{(int16_t*)&glovedetection_t78_config.mindist, UINT8},
	{(int16_t*)&glovedetection_t78_config.glovemodeto, UINT8},
	{(int16_t*)&glovedetection_t78_config.supto, UINT8},
	{(int16_t*)&glovedetection_t78_config.syncsperx, UINT8},
	{(int16_t*)&glovedetection_t78_config.hithrmrgn, UINT8},
	{(int16_t*)&glovedetection_t78_config.reserved[0], UINT8},
	{(int16_t*)&glovedetection_t78_config.reserved[1], UINT8},
	{(int16_t*)&glovedetection_t78_config.reserved[2], UINT8},
	{(int16_t*)&glovedetection_t78_config.reserved[3], UINT8},
};


config_table_element t80_retransmissioncompensation_config_table[] = {
	{(int16_t*)&retransmissioncompensation_t80_config.ctrl, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.compgain, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.targetdelta, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.compthr, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.atchthr, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.moistcfg, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.moistdto, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.reserved, UINT8},
	{(int16_t*)&retransmissioncompensation_t80_config.reserved_1, UINT8},
};

config_table_element t84_gesturepocessor_config_table[] = {
  {(int16_t*)&gesturepocessor_t84_config.ctrl, UINT8},
  {(int16_t*)&gesturepocessor_t84_config.zonethr, UINT8}, 
  {(int16_t*)&gesturepocessor_t84_config.direl, UINT8},
  {(int16_t*)&gesturepocessor_t84_config.dto, UINT8},
};

config_table_element t100_touchscreen_config_table[] = {
	{(int16_t*)&touchscreen_config.ctrl, UINT8},
	{(int16_t*)&touchscreen_config.cfg1, UINT8},
	{(int16_t*)&touchscreen_config.scraux, UINT8},
	{(int16_t*)&touchscreen_config.tchaux, UINT8},
	{(int16_t*)&touchscreen_config.tcheventcfg, UINT8},
	{(int16_t*)&touchscreen_config.akscfg, UINT8},
	{(int16_t*)&touchscreen_config.numtch, UINT8},
	{(int16_t*)&touchscreen_config.xycfg, UINT8},
	{(int16_t*)&touchscreen_config.xorigin, UINT8},
	{(int16_t*)&touchscreen_config.xsize, UINT8},
	{(int16_t*)&touchscreen_config.xpitch, UINT8},
	{(int16_t*)&touchscreen_config.xloclip, INT8},
	{(int16_t*)&touchscreen_config.xhiclip, INT8},
	{(int16_t*)&touchscreen_config.xrange, UINT16},
	{0},
	{(int16_t*)&touchscreen_config.xedgecfg, UINT8},
	{(int16_t*)&touchscreen_config.xedgedist, UINT8},
	{(int16_t*)&touchscreen_config.dxxedgecfg, UINT8},
	{(int16_t*)&touchscreen_config.dxxedgedist, UINT8},
	{(int16_t*)&touchscreen_config.yorigin, UINT8},
	{(int16_t*)&touchscreen_config.ysize, UINT8},
	{(int16_t*)&touchscreen_config.ypitch, UINT8},
	{(int16_t*)&touchscreen_config.yloclip, INT8},
	{(int16_t*)&touchscreen_config.yhiclip, INT8},
	{(int16_t*)&touchscreen_config.yrange, UINT16},
	{0},
	{(int16_t*)&touchscreen_config.yedgecfg, UINT8},
	{(int16_t*)&touchscreen_config.yedgedist, UINT8},
	{(int16_t*)&touchscreen_config.gain, UINT8},
	{(int16_t*)&touchscreen_config.dxgain, UINT8},
	{(int16_t*)&touchscreen_config.tchthr, UINT8},
	{(int16_t*)&touchscreen_config.tchhyst, UINT8},
	{(int16_t*)&touchscreen_config.intthr, UINT8},
	{(int16_t*)&touchscreen_config.noisesf, UINT8},
	{(int16_t*)&touchscreen_config.cutoffthr, UINT8},
	{(int16_t*)&touchscreen_config.mrgthr, UINT8},
	{(int16_t*)&touchscreen_config.mrgthradjstr, UINT8},
	{(int16_t*)&touchscreen_config.mrghyst, UINT8},
	{(int16_t*)&touchscreen_config.dxthrsf, UINT8},
	{(int16_t*)&touchscreen_config.tchdidown, UINT8},
	{(int16_t*)&touchscreen_config.tchdiup, UINT8},
	{(int16_t*)&touchscreen_config.nexttchdi, UINT8},
	{(int16_t*)&touchscreen_config.reserved, UINT8},
	{(int16_t*)&touchscreen_config.jumplimit, UINT8},
	{(int16_t*)&touchscreen_config.movfilter, UINT8},
	{(int16_t*)&touchscreen_config.movsmooth, UINT8},
	{(int16_t*)&touchscreen_config.movpred, UINT8},
	{(int16_t*)&touchscreen_config.movhysti, UINT16},
	{0},
	{(int16_t*)&touchscreen_config.movhystn, UINT16},
	{0},
	{(int16_t*)&touchscreen_config.amplhyst, UINT8},
	{(int16_t*)&touchscreen_config.scrareahyst, UINT8},
	{(int16_t*)&touchscreen_config.intthrhyst, UINT8},
};

config_table_element t101_touchscreenhover_config_table[] = {
	{(int16_t*)&touchscreenhover_t101_config.ctrl, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xloclip, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xhiclip, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xedgecfg, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xedgedist, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xgain, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xhvrthr, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.xhvrhyst, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.yloclip, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.yhiclip, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.yedgecfg, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.yedgedist, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.ygain, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.yhvthr, UINT8},  
	{(int16_t*)&touchscreenhover_t101_config.yhvrhyst, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.hvrdi, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.confthr, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.movfilter,  UINT8},
	{(int16_t*)&touchscreenhover_t101_config.movsmooth,  UINT8},
	{(int16_t*)&touchscreenhover_t101_config.movpred, UINT8},
	{(int16_t*)&touchscreenhover_t101_config.movhysti, UINT16},
	{0},
	{(int16_t*)&touchscreenhover_t101_config.movhystn, UINT16},
	{0},
	{(int16_t*)&touchscreenhover_t101_config.hvraux,  UINT8},
	{(int16_t*)&touchscreenhover_t101_config.reserved,  UINT8},
};

config_table_element t104_auxtouchconfig_config_table[] = {
	{(int16_t*)&auxtouchconfig_t104_config.ctrl, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.xgain, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.xtchthr, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.xtchhyst, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.xintthr, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.xinthyst, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.ygain, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.ytchthr, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.ytchhyst, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.yintthr, UINT8},
	{(int16_t*)&auxtouchconfig_t104_config.yinthyst, UINT8},
};

config_table_element t108_noisesupselfcap_config_table[] = {
	{(int16_t*)&noisesupselfcap_t108_config.ctrl, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.calcfg1, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.cfg1, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.cfg2, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.cfg3, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.nlgain, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.nlgclim, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.iircoeff, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.minnltddiff, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.minnltdhop, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_1, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.hopst, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabctrl, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabfreq[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabfreq[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabfreq[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabfreq[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabfreq[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabtchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabtchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabtchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabtchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabtchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabnotchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabnotchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabnotchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabnotchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabnotchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_2, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_3, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.stabhinltdthr, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_4, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisctrl, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisfreq[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisfreq[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisfreq[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisfreq[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisfreq[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noistchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noistchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noistchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noistchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noistchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisnotchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisnotchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisnotchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisnotchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noisnotchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_5, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noislonltdthr, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noishinltdthr, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.noiscnt, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoictrl, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoifreq[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoifreq[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoifreq[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoifreq[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoifreq[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoitchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoitchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoitchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoitchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoitchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoinotchapx[0], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoinotchapx[1], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoinotchapx[2], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoinotchapx[3], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoinotchapx[4], UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_6, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoilonltdthr, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_7, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.vnoicnt, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.blknltdthr, UINT8},
	{(int16_t*)&noisesupselfcap_t108_config.reserved_8, UINT8},
};

config_table_element t109_selfcapglobal_config_table[] = {
	{(int16_t*)&selfcapglobalconfig_t109_config.ctrl, UINT8},
	{(int16_t*)&selfcapglobalconfig_t109_config.dbgctrl, UINT8},
	{(int16_t*)&selfcapglobalconfig_t109_config.cmdonreset, UINT8},
	{(int16_t*)&selfcapglobalconfig_t109_config.cmd, UINT8},
	{(int16_t*)&selfcapglobalconfig_t109_config.lfcompsfx, UINT16},
	{0},
	{(int16_t*)&selfcapglobalconfig_t109_config.lfcompsfy, UINT16},
	{0},
	{(int16_t*)&selfcapglobalconfig_t109_config.reserved, UINT8},
};

config_table_element t113_proxmeasure_config_table[] = {
	{(int16_t*)&proxmeasureconfig_t113_config.ctrl, UINT8},
	{(int16_t*)&proxmeasureconfig_t113_config.gainx, UINT8},
	{(int16_t*)&proxmeasureconfig_t113_config.reserved, UINT8},
};

config_table_element* config_table[150] = {
	[7] = (config_table_element*) &t7_power_config_table,
	[8] = (config_table_element*) &t8_acquisition_config_table,
	[15] = (config_table_element*) &t15_keyarray_config_table,
	[18] = (config_table_element*) &t18_commsconfig_config_table,
	[19] = (config_table_element*) &t19_gpiopwm_config_table,
	[25] = (config_table_element*) &t25_selftest_config_table,
	[40] = (config_table_element*) &t40_gripsuppression_config_table,
	[42] = (config_table_element*) &t42_touchsuppression_config_table,
	[46] = (config_table_element*) &t46_cteconfig_config_table,
	[47] = (config_table_element*) &t47_stylus_config_table,
	[56] = (config_table_element*) &t56_shieldless_config_table,
	[61] = (config_table_element*) &t61_timer_config,
	[65] = (config_table_element*) &t65_lensbending_config_table,
	[70] = (config_table_element*) &t70_dynamiccfgcontroller_config_table,	
	[72] = (config_table_element*) &t72_noisesuppression_config_table,
	[78] = (config_table_element*) &t78_glovedetection_config_table,
	[80] = (config_table_element*) &t80_retransmissioncompensation_config_table,
  	[84] = (config_table_element*) &t84_gesturepocessor_config_table,
	[100] = (config_table_element*) &t100_touchscreen_config_table,
  	[101] = (config_table_element*) &t101_touchscreenhover_config_table,  
	[104] = (config_table_element*) &t104_auxtouchconfig_config_table,
	[108] = (config_table_element*) &t108_noisesupselfcap_config_table,
	[109] = (config_table_element*) &t109_selfcapglobal_config_table,
	[113] = (config_table_element*) &t113_proxmeasure_config_table,
};

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	/* only used object */
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_DEBUG_DIAGNOSTIC_T37:
	case MXT_SPT_USERDATA_T38:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_MESSAGECOUNT_T44:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_DYNAMICCFGCONTROLLER_T70:
	case MXT_SPT_DYNAMICCFGCONTAINER_T71:
	case MXT_PROCG_NOISESUPPRESSION_T72:
	case MXT_PROCI_GLOVEDETECTION_T78:
	case MXT_PROCI_RETRANSMISSION_T80:
	case MXT_PROCI_GESTURE_T84:
	case MXT_TOUCH_MULTI_T100:
	case MXT_SPT_TOUCHSCREENHOVER_T101:
	case MXT_SPT_AUXTOUCHCONFIG_T104:
	case MXT_PROCG_NOISESUPSELFCAP_T108:
	case MXT_SPT_SELFCAPGLOBAL_T109:
	case MXT_SPT_SELFCAPTUNINGPARAMS_T110:
	case MXT_SPT_SELFCAPCONFIG_T111:
	case MXT_PROCI_HOVERGRIPSUPPRESSION_T112:
	case MXT_SPT_PROXMEASURECONFIG_T113:
		return true;
	default:
		return false;
	}
}

static int mxt_write_config(uint8_t object_type, uint8_t instance, void *cfg)
{
	struct mxt_object *object = NULL;
	u16 reg = 0;
	int ret;

	object = mxt_get_object(data_common, object_type);
	if ((!object) || (!object->start_address) || (!OBP_SIZE(object))) {
		return -EINVAL;
	}

	reg = object->start_address + OBP_SIZE(object) * instance;

	if (!reg){
		ret = -EINVAL;
	}
	else{
		ret = mxt_write_mem(data_common->client, reg, OBP_SIZE(object), cfg);
	}

	return ret; 
}

int write_power_T7_config(gen_powerconfig_t7_config_t cfg)
{
	return mxt_write_config(MXT_GEN_POWER_T7, 0, (void *) &cfg);
}

int write_acquisition_T8_config(gen_acquisitionconfig_t8_config_t cfg)
{
	return mxt_write_config(MXT_GEN_ACQUIRE_T8, 0, (void *) &cfg);	
}

int write_keyarray_T15_config(uint8_t instance, touch_keyarray_t15_config_t cfg)
{
	return mxt_write_config(MXT_TOUCH_KEYARRAY_T15, instance, (void *) &cfg);
}

int write_comms_T18_config(uint8_t instance, spt_commsconfig_t18_config_t cfg)
{
	return mxt_write_config(MXT_SPT_COMMSCONFIG_T18, instance, (void *) &cfg);
}

int write_gpiopwm_T19_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg)
{
	return mxt_write_config(MXT_SPT_GPIOPWM_T19, instance, (void *) &cfg);
}

int write_selftest_T25_config(uint8_t instance, spt_selftest_t25_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_selftest_t25_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.cmd;

	ret = mxt_write_config(MXT_SPT_SELFTEST_T25, instance, tmp);

	kfree(tmp);
	return ret;
}

int write_grip_suppression_T40_config(proci_gripsuppression_t40_config_t cfg)
{
	return(mxt_write_config(MXT_PROCI_GRIP_T40, 0, (void *) &cfg));
}

int write_touch_suppression_T42_config(proci_touchsuppression_t42_config_t cfg)
{
	return(mxt_write_config(MXT_PROCI_TOUCHSUPPRESSION_T42, 0, (void *) &cfg));
}

int write_CTE_T46_config(spt_cteconfig_t46_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_cteconfig_t46_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	memcpy(tmp, &cfg, 7);
	*(tmp + 7) = (uint8_t) (cfg.syncdelay& 0x00FF);
	*(tmp + 8) = (uint8_t) (cfg.syncdelay >> 8);
	*(tmp + 9) = cfg.xvoltage;
	*(tmp + 10) = cfg.adcctrl;
	*(tmp + 11) = cfg.reserved;

	ret = mxt_write_config(MXT_SPT_CTECONFIG_T46, 0, tmp);

	kfree(tmp);
	return ret;
}

int  write_stylus_T47_config(proci_stylus_t47_config_t cfg){
	return(mxt_write_config(MXT_PROCI_STYLUS_T47, 0, (void *) &cfg));
}

int  write_shieldless_T56_config(proci_shieldless_t56_config_t cfg){
	return(mxt_write_config(MXT_PROCI_SHIELDLESS_T56, 0, (void *) &cfg));
}

int write_timer_T61_config(spt_timer_t61_config_t cfg){
	return(mxt_write_config(MXT_SPT_TIMER_T61, 0, (void *) &cfg));
}

int write_lensbending_T65_config(uint8_t instance, proci_lensbending_t65_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(proci_lensbending_t65_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = cfg.gradthr;
	*(tmp + 2) = (uint8_t) (cfg.ylonoisemul & 0x00FF);
	*(tmp + 3) = (uint8_t) (cfg.ylonoisemul >> 8);
	*(tmp + 4) = (uint8_t) (cfg.ylonoisediv & 0x00FF);
	*(tmp + 5) = (uint8_t) (cfg.ylonoisediv >> 8);
	*(tmp + 6) = (uint8_t) (cfg.yhinoisemul & 0x00FF);
	*(tmp + 7) = (uint8_t) (cfg.yhinoisemul >> 8);
	*(tmp + 8) = (uint8_t) (cfg.yhinoisediv & 0x00FF);
	*(tmp + 9) = (uint8_t) (cfg.yhinoisediv >> 8);
	*(tmp + 10) = cfg.lpfiltcoef;
	*(tmp + 11) = (uint8_t) (cfg.forcescale & 0x00FF);
	*(tmp + 12) = (uint8_t) (cfg.forcescale >> 8);
	*(tmp + 13) = cfg.forcethr;
	*(tmp + 14) = cfg.forcethrhyst;
	*(tmp + 15) = cfg.forcedi;
	*(tmp + 16) = cfg.forcehyst;
	*(tmp + 17) = cfg.atchratio;
	*(tmp + 18) = (uint8_t) (cfg.totvarlim & 0x00FF);
	*(tmp + 19) = (uint8_t) (cfg.totvarlim >> 8);
	*(tmp + 20) = cfg.reserved[0];
	*(tmp + 21) = cfg.reserved[1];
	*(tmp + 22) = cfg.reserved[2];

	ret = mxt_write_config(MXT_PROCI_LENSBENDING_T65, instance, tmp);

	kfree(tmp);
	return ret;
}

int  write_dynamiccfgcontroller_t70_config(uint8_t instance, spt_dynamiccfgcontroller_t70_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_dynamiccfgcontroller_t70_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	*(tmp + 0) = cfg.ctrl;
	*(tmp + 1) = (uint8_t) (cfg.event & 0x00FF);
	*(tmp + 2) = (uint8_t) (cfg.event >> 8);
	*(tmp + 3) = cfg.objtype;
	*(tmp + 4) = cfg.reserved;
	*(tmp + 5) = cfg.objinst;
	*(tmp + 6) = cfg.dstoffset;
	*(tmp + 7) = (uint8_t) (cfg.srcoffset & 0xFF);
	*(tmp + 8) = (uint8_t) (cfg.srcoffset >> 8);
	*(tmp + 9) = cfg.length;
  
	ret = mxt_write_config(MXT_SPT_DYNAMICCFGCONTROLLER_T70, instance, tmp);

	kfree(tmp);
	return ret;
}

int  write_dynamicconfigurationcontainer_t71_config(spt_dynamiccfgcontainer_t71_config_t cfg)
{
	return(mxt_write_config(MXT_SPT_DYNAMICCFGCONTAINER_T71, 0, (void *) &cfg));
}

int  write_noisesuppression_T72_config(procg_noisesuppression_t72_config_t cfg)
{
	return(mxt_write_config(MXT_PROCG_NOISESUPPRESSION_T72, 0, (void *) &cfg));
}

int  write_glovedetection_T78_config(proci_glovedetection_t78_config_t cfg)
{
	return(mxt_write_config(MXT_PROCI_GLOVEDETECTION_T78, 0, (void *) &cfg));
}

int  write_retransmissioncompensation_T80_config(proci_retransmissioncompensation_t80_config_t cfg)
{
	return(mxt_write_config(MXT_PROCI_RETRANSMISSION_T80, 0, (void *) &cfg));
}

int  write_gesturepocessor_T84_config(proci_gesturepocessor_t84_config_t cfg)
{
	return(mxt_write_config(MXT_PROCI_GESTURE_T84, 0, (void *) &cfg));
}

int write_multitouchscreen_T100_config(uint8_t instance, touch_multitouchscreen_t100_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(touch_multitouchscreen_t100_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	memcpy(tmp, &cfg, 13);
	*(tmp + 13) = (uint8_t) (cfg.xrange &  0xFF);
	*(tmp + 14) = (uint8_t) (cfg.xrange >> 8);
	*(tmp + 15) = cfg.xedgecfg;
	*(tmp + 16) = cfg.xedgedist;
	*(tmp + 17) = cfg.dxxedgecfg;
	*(tmp + 18) = cfg.dxxedgedist;
	*(tmp + 19) = cfg.yorigin;
	*(tmp + 20) = cfg.ysize;
	*(tmp + 21) = cfg.ypitch;
	*(tmp + 22) = cfg.yloclip;
	*(tmp + 23) = cfg.yhiclip;	
	*(tmp + 24) = (uint8_t) (cfg.yrange &  0xFF);
	*(tmp + 25) = (uint8_t) (cfg.yrange >> 8);

	memcpy((tmp+26), &(cfg.yedgecfg), 21);
	*(tmp + 47) = (uint8_t) (cfg.movhysti&  0xFF);
	*(tmp + 48) = (uint8_t) (cfg.movhysti >> 8);
	*(tmp + 49) = (uint8_t) (cfg.movhystn &  0xFF);
	*(tmp + 50) = (uint8_t) (cfg.movhystn >> 8);
	*(tmp + 51) = cfg.amplhyst;
	*(tmp + 52) = cfg.scrareahyst;
	*(tmp + 53) = cfg.intthrhyst;	

	ret = mxt_write_config(MXT_TOUCH_MULTI_T100, instance, tmp);

	kfree(tmp);
	return ret;

}

int write_touchscreenhover_T101_config(uint8_t instance, spt_touchscreenhover_t101_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_touchscreenhover_t101_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	memcpy(tmp, &cfg, 20);
	*(tmp + 20) = (uint8_t) (cfg.movhysti &  0xFF);
	*(tmp + 21) = (uint8_t) (cfg.movhysti >> 8);
	*(tmp + 22) = (uint8_t) (cfg.movhystn &  0xFF);
	*(tmp + 23) = (uint8_t) (cfg.movhystn >> 8);
	*(tmp + 24) = cfg.hvraux;
	*(tmp + 25) = cfg.reserved;

	ret = mxt_write_config(MXT_SPT_TOUCHSCREENHOVER_T101, instance, tmp);

	kfree(tmp);
	return ret;

}

int  write_auxtouch_T104_config(spt_auxtouchconfig_t104_config_t cfg){
	return(mxt_write_config(MXT_SPT_AUXTOUCHCONFIG_T104, 0, (void *) &cfg));
}

int  write_noisesupselfcap_t108_config(proci_noisesupselfcap_t108_config_t cfg){
	return(mxt_write_config(MXT_PROCG_NOISESUPSELFCAP_T108, 0, (void *) &cfg));
}

int  write_selfcapglobalconfig_t109_config(spt_selfcapglobalconfig_t109_config_t cfg){
	uint8_t *tmp = NULL;
	int ret = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_selfcapglobalconfig_t109_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	memcpy(tmp, &cfg, 4);
	*(tmp + 4) = (uint8_t) (cfg.lfcompsfx&  0xFF);
	*(tmp + 5) = (uint8_t) (cfg.lfcompsfx >> 8);
	*(tmp + 6) = (uint8_t) (cfg.lfcompsfy &  0xFF);
	*(tmp + 7) = (uint8_t) (cfg.lfcompsfy >> 8);
	*(tmp + 8) = cfg.reserved;

	ret = mxt_write_config(MXT_SPT_SELFCAPGLOBAL_T109, 0, tmp);

	kfree(tmp);
	return ret;
}

int  write_selfcaptuningparams_t110_config(uint8_t instance, spt_selfcaptuningparams_t110_config_t cfg)
{
	uint8_t *tmp = NULL;
	int ret = 0;
	int i = 0;

	tmp = (uint8_t *) kzalloc(sizeof(spt_selfcaptuningparams_t110_config_t), GFP_KERNEL | GFP_ATOMIC);
	if (!tmp)
		return(-EINVAL);

	for(i=0; i<T110_PARAM_SIZE; i++){
		*(tmp + (2*i)) 	= (uint8_t) (cfg.params[i] & 0x00FF);
		*(tmp + ((2*i)+1)) 	= (uint8_t) (cfg.params[i] >> 8);
	}
  
	ret = mxt_write_config(MXT_SPT_SELFCAPTUNINGPARAMS_T110, instance, tmp);

	kfree(tmp);
	return ret;
}

int  write_selfcapconfig_t111_config(uint8_t instance, spt_selfcapconfig_t111_config_t cfg){
	return(mxt_write_config(MXT_SPT_SELFCAPCONFIG_T111, instance, (void *) &cfg));
}

int  write_proxmeasureconfig_t113_config(spt_proxmeasureconfig_t113_config_t cfg){
	return(mxt_write_config(MXT_SPT_PROXMEASURECONFIG_T113, 0, (void *) &cfg));
}

void mxt_T7_Power_Config_Init(void)
{
	power_config = obj_power_config_t7[mTouch_mode];

	if (write_power_T7_config(power_config))
		dbg_cr(" T7 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_Acquisition_Config_T8_Init(void)
{
	acquisition_config = obj_acquisition_config_t8[mTouch_mode];

	if (write_acquisition_T8_config(acquisition_config))	
		dbg_cr(" T8 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_KeyArray_T15_Init(void)
{
	keyarray_config = obj_key_array_t15[mTouch_mode];

	if (write_keyarray_T15_config(0, keyarray_config))
		dbg_cr(" T15 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_CommsConfig_T18_Init(void)
{
	comms_config = obj_comm_config_t18[mTouch_mode];

	if (write_comms_T18_config(0, comms_config))
		dbg_cr("T18 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_Gpio_Pwm_T19_Init(void)
{
	gpiopwm_config = obj_gpiopwm_config_t19[mTouch_mode];

	if (write_gpiopwm_T19_config(0, gpiopwm_config))
		dbg_cr("T19 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_T25_Selftest_Init(void)
{
	selftest_config = obj_self_test_t25[mTouch_mode];

	if (write_selftest_T25_config(0,selftest_config))
		dbg_cr("T25 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_Grip_Suppression_T40_Config_Init(void)
{
	gripsuppression_t40_config = obj_grip_suppression_t40[mTouch_mode];

	if (write_grip_suppression_T40_config(gripsuppression_t40_config))
		dbg_cr("T40 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_Touch_Suppression_T42_Config_Init(void)
{
	touchsuppression_t42_config = obj_touch_suppression_t42[mTouch_mode];

	if (write_touch_suppression_T42_config(touchsuppression_t42_config))
		dbg_cr("T42 Configuration Fail!!! , Line %d \n", __LINE__);		
}

void mxt_CTE_T46_Config_Init(void)
{
	cte_t46_config = obj_cte_config_t46[mTouch_mode];

	if (write_CTE_T46_config(cte_t46_config))
		dbg_cr("T46 Configuration Fail!!! , Line %d \n", __LINE__);
}

void mxt_Stylus_T47_Config_Init(void)
{
	stylus_t47_config = obj_stylus_t47[mTouch_mode];

	if (write_stylus_T47_config(stylus_t47_config))
		dbg_cr("T47 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Shieldless_T56_Config_Init(void)
{
	proci_shieldless_t56_config = obj_slim_sensor_t56[mTouch_mode];

	if (write_shieldless_T56_config(proci_shieldless_t56_config))
		dbg_cr("T56 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Timer_T61_Config_Init(void)
{
	spt_timer_t61_config = obj_timer_t61[mTouch_mode];

	if (write_timer_T61_config(spt_timer_t61_config))
		dbg_cr("T61 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Lensbending_T65_Config_Init(void)
{
	lensbending_t65_config = obj_lens_bending_t65[mTouch_mode];

	if (write_lensbending_T65_config(0,lensbending_t65_config))
		dbg_cr("T65 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Dynamicconfigurationcontroller_T70_Config_Init(int instance)
{
	dynamiccfgcontroller_t70_config = obj_dynamic_config_controller_t70[mTouch_mode][instance];

	if (write_dynamiccfgcontroller_t70_config(instance, dynamiccfgcontroller_t70_config))
		dbg_cr("T70 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Dynamicconfigurationcontainer_T71_Config_Init(void)
{
	dynamiccfgcontainer_t71_config = obj_dynamic_config_container_t71[mTouch_mode];
	
	if (write_dynamicconfigurationcontainer_t71_config(dynamiccfgcontainer_t71_config))
		dbg_cr("T71 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Noisesuppression_T72_Config_Init(void)
{
	noisesuppression_t72_config = obj_mxt_charger_t72[mTouch_mode];

	if (write_noisesuppression_T72_config(noisesuppression_t72_config))
		dbg_cr("T72 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Glovedetection_T78_Config_Init(void)
{
	glovedetection_t78_config = obj_glove_detect_t78[mTouch_mode];

	if (write_glovedetection_T78_config(glovedetection_t78_config))
		dbg_cr("T78 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Retransmissioncompensation_T80_Config_Init(void)
{
	retransmissioncompensation_t80_config = obj_retransmissioncompensation_t80[mTouch_mode];

	if (write_retransmissioncompensation_T80_config(retransmissioncompensation_t80_config))
		dbg_cr("T80 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Gestureprocess_T84_Config_Init(void)
{
	gesturepocessor_t84_config = obj_gesturepocessor_t84[mTouch_mode];

	if (write_gesturepocessor_T84_config(gesturepocessor_t84_config))
		dbg_cr("T84 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Multitouchscreen_T100_Config_Init(void)
{	
	touchscreen_config = obj_multi_touch_t100[mTouch_mode];

	if (write_multitouchscreen_T100_config(0, touchscreen_config))
		dbg_cr("T100 Configuration Fail!!!, %s, Line %d \n", __func__, __LINE__);
}

void mxt_Touchscreenhover_T101_Config_Init(void) {
	touchscreenhover_t101_config = obj_touchscreenhover_t101[mTouch_mode];

	if (write_touchscreenhover_T101_config(0, touchscreenhover_t101_config))
		dbg_cr("T101 Configuration Fail!!!, %s, Line %d \n", __func__, __LINE__);
}

void mxt_Auxtouch_T104_Config_Init(void) {
	auxtouchconfig_t104_config = obj_auxtouchconfig_t104[mTouch_mode];

	if (write_auxtouch_T104_config(auxtouchconfig_t104_config))
		dbg_cr("T104 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Noisesupselfcap_T108_Config_Init(void) {
	noisesupselfcap_t108_config = obj_noisesupselfcap_t108[mTouch_mode];

	if (write_noisesupselfcap_t108_config(noisesupselfcap_t108_config))
		dbg_cr("T108 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Selfcapglobalconfig_T109_config_Init(void) {
	selfcapglobalconfig_t109_config = obj_selfcapglobalconfig_t109[mTouch_mode];

	if (write_selfcapglobalconfig_t109_config(selfcapglobalconfig_t109_config))
		dbg_cr("T109 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Selfcaptuningparams_T110_Config_Init(int instance)
{
	selfcaptuningparams_t110_config = obj_selfcaptuningparams_t110[mTouch_mode][instance];

	if (write_selfcaptuningparams_t110_config(instance, selfcaptuningparams_t110_config))
		dbg_cr("T110 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Selfcapconfig_T111_Config_Init(int instance)
{
	selfcapconfig_t111_config = obj_selfcapconfig_t111[mTouch_mode][instance];

	if (write_selfcapconfig_t111_config(instance, selfcapconfig_t111_config))
		dbg_cr("T111 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_Selfcapglobalconfig_T113_config_Init(void) {
	proxmeasureconfig_t113_config = obj_proxmeasureconfig_t113[mTouch_mode];

	if (write_proxmeasureconfig_t113_config(proxmeasureconfig_t113_config))
		dbg_cr("T113 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void mxt_cfg_all_clear(void)
{
	struct mxt_object *object = NULL;
	u16 size = 0;
	u8* mem = NULL;
	int i = 0, ret = 0;

	for(i = MXT_OBJECT_START; i < (MXT_MAX_OBJECT+1); i++){

		object = mxt_get_object(data_common, i);
		if ((!object) || (!object->start_address) || (!OBP_SIZE(object))) {
			dbg_cr("object(%d) is NULL.\n", i); continue;
		}

		size = (OBP_SIZE(object) * OBP_INSTANCES(object));	
		mem = kzalloc(size, GFP_KERNEL);
		if (!mem){
			dbg_cr("mem is NULL.\n"); continue;
		}	

		ret = mxt_write_mem(data_common->client, object->start_address, size, mem);
		dbg_cr("object(%d) clear ret : %d!!\n", i, ret);
		 
		kfree(mem);
		
	}	
}


static void apply_touch_config(void)
{
//	if (driver_setup != DRIVER_SETUP_OK){
//	  dbg_config("%s driver setup is failed\n",__FUNCTION__);
//		return;
//	}
#if MXT_CFG_WRITE_BIN
	return;
#endif

	if (write_power_T7_config(power_config))	
		dbg_cr(" T7 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_acquisition_T8_config(acquisition_config))	
		dbg_cr("T8 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_grip_suppression_T40_config(gripsuppression_t40_config))		
		dbg_cr("T40 Configuration Fail!!! , Line %d \n\r", __LINE__);	
	
	if (write_touch_suppression_T42_config(touchsuppression_t42_config))	
		dbg_cr("T42 Configuration Fail!!! , Line %d \n\r", __LINE__);	
	
	if (write_CTE_T46_config(cte_t46_config))		
		dbg_cr("T46 Configuration Fail!!! , Line %d \n\r", __LINE__);	

	if (write_stylus_T47_config(stylus_t47_config))	
		dbg_cr("T47 Configuration Fail!!! , Line %d \n\r", __LINE__);	

	if (write_shieldless_T56_config(proci_shieldless_t56_config)) 
		dbg_cr("T56 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_lensbending_T65_config(0, lensbending_t65_config)) 
		dbg_cr("T65 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_noisesuppression_T72_config(noisesuppression_t72_config)) 
		dbg_cr("T72 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_glovedetection_T78_config(glovedetection_t78_config))
		dbg_cr("T78 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_retransmissioncompensation_T80_config(retransmissioncompensation_t80_config))
		dbg_cr("T80 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_gesturepocessor_T84_config(gesturepocessor_t84_config))
		dbg_cr("T84 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_multitouchscreen_T100_config(0, touchscreen_config)) 
		dbg_cr("T100 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_touchscreenhover_T101_config(0, touchscreenhover_t101_config)) 
		dbg_cr("T101 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_auxtouch_T104_config(auxtouchconfig_t104_config)) 
		dbg_cr("T104 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_noisesupselfcap_t108_config(noisesupselfcap_t108_config)) 
		dbg_cr("T108 Configuration Fail!!! , Line %d \n\r", __LINE__);

	if (write_selfcapglobalconfig_t109_config(selfcapglobalconfig_t109_config)) 
		dbg_cr("T109 Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void init_touch_config(void)
{
	int i = 0;
	
#if MXT_CFG_WRITE_BIN
	return;
#endif

#if 1
	mxt_cfg_all_clear();

/* Only used config */
	mxt_T7_Power_Config_Init();
	mxt_Acquisition_Config_T8_Init();
	mxt_Grip_Suppression_T40_Config_Init();
	mxt_Touch_Suppression_T42_Config_Init();
	mxt_CTE_T46_Config_Init();
	mxt_Stylus_T47_Config_Init();	
	mxt_Shieldless_T56_Config_Init();
	mxt_Lensbending_T65_Config_Init();
 	mxt_Dynamicconfigurationcontroller_T70_Config_Init(0);
 	mxt_Dynamicconfigurationcontroller_T70_Config_Init(1);
 	mxt_Dynamicconfigurationcontroller_T70_Config_Init(2);
	mxt_Dynamicconfigurationcontainer_T71_Config_Init();
	mxt_Noisesuppression_T72_Config_Init();
	mxt_Glovedetection_T78_Config_Init();
	mxt_Retransmissioncompensation_T80_Config_Init();
	mxt_Multitouchscreen_T100_Config_Init();
	mxt_Touchscreenhover_T101_Config_Init();
	mxt_Auxtouch_T104_Config_Init();
	mxt_Noisesupselfcap_T108_Config_Init();
	mxt_Selfcapglobalconfig_T109_config_Init();

	for(i=0; i<T110_MAX_INSTANCE_SIZE;i++)
		mxt_Selfcaptuningparams_T110_Config_Init(i);
	for(i=0; i<T111_MAX_INSTANCE_SIZE;i++)
		mxt_Selfcapconfig_T111_Config_Init(i);
		
	mxt_Selfcapglobalconfig_T113_config_Init();
#endif
}

