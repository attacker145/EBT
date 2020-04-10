/*
 * api.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Roman
 */

#ifndef API_H_
#define API_H_
//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern int BlockRead_(unsigned char ucRegAddr, unsigned char *pucBlkData, unsigned char ucBlkDataSz);
extern int GetRegisterValue_(unsigned char ucRegAddr, unsigned char *pucRegValue);
extern int SetRegisterValue_(unsigned char ucRegAddr, unsigned char ucRegValue);
extern int BMA222Close_();
extern int BMA222Read_(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ);
extern int BMA222ReadNew_(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ);
extern void MessageUART1(const char *str);
extern void InitTermUART1();
extern int GetCmdUART1(char *pcBuffer, unsigned int uiBufLen);
extern int BMA222Open_();
extern void AccSample_();
extern void SetAccAvg_();
extern int GetRegisterValue_temp_(unsigned char ucRegAddr, unsigned short *pusRegValue);
extern int TMP006DrvOpen_();
extern double ComputeTemperature_(double dVobject, double dTAmbient);
extern int TMP006DrvGetTemp_(float *pfCurrTemp);
//extern int BMA222Open_();
//extern unsigned short itoa(short cNum, char *cString);
extern void init_adc (void);

extern void TimerRefIntHandler(void);
extern void TimerBaseIntHandler(void);
extern void timerA0_start(unsigned long ms);
extern void timerA0_stop(void);
extern void credentials (void);

extern void AK9753_ID (void);

//extern void WDT_IF_Init(fAPPWDTDevCallbk fpAppWDTCB, unsigned int uiReloadVal);
extern void WDT_IF_DeInit(void);
extern void WatchdogIntHandler(void);
extern void PerformPRCMSleepGPTWakeup();
extern void EnterHIBernate();
extern long Network_IF_DeInitDriver(void);
void GPIO_IF_Toggle(unsigned char SH_GPIO_XX);
extern void AK975XsetMode(unsigned char mode);
extern int AK9753AReadData(unsigned char *IR1L_, unsigned char *IR1H_,
        unsigned char *IR2L_, unsigned char *IR2H_,
        unsigned char *IR3L_, unsigned char *IR3H_,
        unsigned char *IR4L_, unsigned char *IR4H_);
//extern void hexdec_long( uint32_t count );
//extern void setCutoffFrequency(uint8_t frequency);
extern void SoftReset(void);
extern void IntrrptSourceSet(void);
extern void ECNTL1_rst (void);
extern float getTemperature_AK9753(unsigned char *sgn);
//extern int HTTPPostMethod_SQL_delete(HTTPCli_Handle httpClient);
extern void GPIOs2IntHandler();
extern void Button_IF_EnableInterrupt(unsigned char ucSwitch);
//extern void Button_IF_Init(P_INT_HANDLER S2InterruptHdl,P_INT_HANDLER S3InterruptHdl );
extern int ReportUART1(const char *pcFormat, ...);
extern void scan_IR(void);
extern void address (unsigned char *Databuf);

//extern void data_init (void);
//extern float adc_read (uint64_t* run);

extern void sonar_cycle_pwr (void);
extern float SignMag (unsigned int IR);
extern double sonar_sensrA(unsigned long int *mean, unsigned int *stl_data1, unsigned char *T_snr);
extern double sonar_sensrB(unsigned long int *mean, unsigned int *stl_data2, unsigned char *T_snr);
extern double noise_suppr (double stdev, unsigned long int* mean, unsigned int* data10, unsigned int* stl_data);
extern void prnt_sonar (double st_dev1, double st_dev2, unsigned long int mean_SNSR1, unsigned long int mean_SNSR2);




#endif /* API_H_ */
