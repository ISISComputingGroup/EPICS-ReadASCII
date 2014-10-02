#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <sys/stat.h>

#include "ReadASCII.h"

#include <macLib.h>
#include <epicsGuard.h>

#include <epicsExport.h>

#define INIT_ROW_NUM 20
#define EPSILON 0.001

static const char *driverName="ReadASCII";
void readFilePoll(void *drvPvt);
void rampThread(void *drvPvt);

/// Constructor for the ReadASCII class.
/// Calls constructor for the asynPortDriver base class.
ReadASCII::ReadASCII(const char *portName) 
   : asynPortDriver(portName, 
                    4, /* maxAddr */ 
                    NUM_READASCII_PARAMS,
					asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynOctetMask,  /* Interrupt mask */
                    0, /* asynFlags.  This driver can block but it is not multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0)	/* Default stack size*/
{
	asynStatus status;
    const char *functionName = "ReadASCII";

	eventId_ = epicsEventCreate(epicsEventEmpty);

	createParam(P_DirString, asynParamOctet, &P_Dir);
	createParam(P_DirBaseString, asynParamOctet, &P_DirBase);
	createParam(P_IndexString, asynParamInt32, &P_Index);

	createParam(P_SPArrString, asynParamFloat64Array, &P_SPArr);
	createParam(P_PArrString, asynParamFloat64Array, &P_PArr);
	createParam(P_IArrString, asynParamFloat64Array, &P_IArr);
	createParam(P_DArrString, asynParamFloat64Array, &P_DArr);
	createParam(P_MaxHeatArrString, asynParamFloat64Array, &P_MaxHeatArr);

	createParam(P_RampingString, asynParamInt32, &P_Ramping);
	createParam(P_RampOnString, asynParamInt32, &P_RampOn);
	createParam(P_LookUpOnString, asynParamInt32, &P_LookUpOn);

	createParam(P_TargetString, asynParamFloat64, &P_Target);
	createParam(P_RampRateString, asynParamFloat64, &P_RampRate);
	createParam(P_CurTempString, asynParamFloat64, &P_CurTemp);

	createParam(P_SPOutString, asynParamFloat64, &P_SPOut);
	createParam(P_PString, asynParamFloat64, &P_P);
	createParam(P_IString, asynParamFloat64, &P_I);
	createParam(P_DString, asynParamFloat64, &P_D);
	createParam(P_MaxHeatString, asynParamFloat64, &P_MaxHeat);
	
	//Allocate column data
	pSP_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	pP_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	pI_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	pD_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	pMaxHeat_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));

	//Init
	setStringParam(P_Dir, "");

	setDoubleParam(P_RampRate, 1.0);
	setIntegerParam(P_Ramping, 0);
	setIntegerParam(P_RampOn, 0);
	setIntegerParam(P_LookUpOn, 0);

	setDoubleParam(P_SPOut, 0.0);

	lastModified = 0;
	fileBad = true;

	/* Create the thread that watches the file in the background 	*/
	status = (asynStatus)(epicsThreadCreate("ReadASCIIFile",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)::readFilePoll,
		this) == NULL);
	/* Create the thread that ramps the output 	*/
	status = (asynStatus)(epicsThreadCreate("ReadASCIIRamp",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)::rampThread,
		this) == NULL);
	if (status) {
		std::cerr << status << "epicsThreadCreate failure\n";
		return;
	}

}

//check if directory has changed
asynStatus ReadASCII::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
	int function = pasynUser->reason;
	int status = asynSuccess;
	const char *paramName;
	const char* functionName = "writeOctet";

	/* Set the parameter in the parameter library. */
	status = (asynStatus)setStringParam(function, value);

	/* Fetch the parameter string name for possible use in debugging */
	getParamName(function, &paramName);

	if (function == P_Dir) {

		// Directory has changed so update length
		if (value != ""){
			char dirBase[DIR_LENGTH];

			fileBad = false;
			getStringParam(P_DirBase, DIR_LENGTH, dirBase);

			strcat(dirBase, "/");
			status |= readFile(strcat(dirBase, value));
		}
	}

	/* Do callbacks so higher layers see any changes */
	status |= (asynStatus)callParamCallbacks();

	if (status)
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		"%s:%s: status=%d, function=%d, name=%s, value=%d",
		driverName, functionName, status, function, paramName, value);

	*nActual = maxChars;
	return (asynStatus)status;
}

//Check for updates to the index
asynStatus ReadASCII::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName;
	const char* functionName = "writeInt32";
	int LUTOn;

	/* Set the parameter in the parameter library. */
	status = (asynStatus)setIntegerParam(function, value);

	if (function == P_Index) {
		//check lookup on
		getIntegerParam(P_LookUpOn, &LUTOn);

		if (LUTOn)
		{
			//update all column floats
			setDoubleParam(P_SPOut, pSP_[value]);
			setDoubleParam(P_P, pP_[value]);
			setDoubleParam(P_I, pI_[value]);
			setDoubleParam(P_D, pD_[value]);
			setDoubleParam(P_MaxHeat, pMaxHeat_[value]);
		}
	}else if (function == P_LookUpOn) {
		//check file is loaded ok
		if (false == fileBad)
		{
			//uses the current temperature to find PID values
			if (value)
			{
				double curTemp;

				getDoubleParam(P_CurTemp, &curTemp);
				updatePID(getSPInd(curTemp));
			}
		}
	}

	/* Do callbacks so higher layers see any changes */
	status = (asynStatus)callParamCallbacks();

	if (status)
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		"%s:%s: status=%d, function=%d, name=%s, value=%d",
		driverName, functionName, status, function, paramName, value);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
		"%s:%s: function=%d, name=%s, value=%d\n",
		driverName, functionName, function, paramName, value);
	return status;

}

asynStatus ReadASCII::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	int rampOn, LUTOn;
	const char *paramName;
	const char* functionName = "writeFloat64";

	/* Set the parameter in the parameter library. */
	status = (asynStatus)setDoubleParam(function, value);

	/* Fetch the parameter string name for possible use in debugging */
	getParamName(function, &paramName);

	if (function == P_Target)
	{
		//check ramp on
		getIntegerParam(P_RampOn, &rampOn);

		//check lookup on
		getIntegerParam(P_LookUpOn, &LUTOn);

		if (rampOn) {
			double startTemp= 0.0;

			//get current temperature and set as SP
			getDoubleParam(P_CurTemp, &startTemp);
			setDoubleParam(P_SPOut, startTemp);

			//update PIDs
			if (LUTOn)
			{
				updatePID(getSPInd(startTemp));
			}

			//start ramping
			setIntegerParam(P_Ramping, 1);

			//send event to start ramp
			epicsEventSignal(eventId_);

		} else {
			//directly output SP
			setDoubleParam(P_SPOut, value);

			//update PIDs
			if (LUTOn)
			{
				updatePID(getSPInd(value));
			}
		}
	}

	/* Do callbacks so higher layers see any changes */
	status = (asynStatus)callParamCallbacks();

	if (status)
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		"%s:%s: status=%d, function=%d",
		driverName, functionName, status, function);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
		"%s:%s: function=%d\n",
		driverName, functionName, function);
	return status;
}


//Return array when waveform is scanned
asynStatus ReadASCII::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
	size_t nElements, size_t *nIn)
{
	int ncopy;
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *functionName = "readFloat64Array";

	if (nElements < ncopy) ncopy = nElements;
	else ncopy = rowNum;
	if (function == P_SPArr) {
		memcpy(value, pSP_, ncopy*sizeof(epicsFloat64));
	}
	else if (function == P_PArr) {
		memcpy(value, pP_, ncopy*sizeof(epicsFloat64));
	}
	else if (function == P_IArr) {
		memcpy(value, pI_, ncopy*sizeof(epicsFloat64));
	}
	else if (function == P_DArr) {
		memcpy(value, pD_, ncopy*sizeof(epicsFloat64));
	}
	else if (function == P_MaxHeatArr) {
		memcpy(value, pMaxHeat_, ncopy*sizeof(epicsFloat64));
	}
	*nIn = ncopy;

	if (status)
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		"%s:%s: status=%d, function=%d",
		driverName, functionName, status, function);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
		"%s:%s: function=%d\n",
		driverName, functionName, function);
	return status;
}

void ReadASCII::rampThread(void)
{
	double wait, rate, target, curSP;
	int ramping, rampOn, lookUpOn;

	lock();

	while (1)
	{
		//get rate
		getDoubleParam(P_RampRate, &rate);

		//get target
		getDoubleParam(P_Target, &target);

		//check running
		getIntegerParam(P_Ramping, &ramping);

		//check ramp mode
		getIntegerParam(P_RampOn, &rampOn);

		unlock();

		if ((!ramping) || (!rampOn)) {
			//wait for run to change
			epicsEventWait(eventId_);

			lock();

			continue;
		}

		lock();

		//get current SP
		getDoubleParam(P_SPOut, &curSP);

		callParamCallbacks();
		unlock(); 
		wait = 5.0; //default wait

		//check near final SP
		double diff = abs(target - curSP);
		if (diff < (5 * rate))
		{
			//wait less time
			wait = diff / rate;
		}
		
		//wait
		epicsEventWaitWithTimeout(eventId_, wait);

		//check rampOn and running (could have changed whilst waiting)
		lock();

		getIntegerParam(P_RampOn, &rampOn);

		if (!rampOn)
		{
			//no longer ramping
			setIntegerParam(P_Ramping, 0);

			continue;
		}

		//rate may have changed whilst waiting
		double newRate;

		getDoubleParam(P_RampRate, &newRate);
		
		if (5.0 != wait) //last part of ramp
		{
			if (newRate < rate){
				continue; //more time is needed
			}
		}
		else{
			rate = newRate; //update rate
		}

		if (curSP > target)
			rate = -rate;

		//update SP with wait*rate
		double newSP = curSP + wait*rate;
		setDoubleParam(P_SPOut, newSP);

		if (abs(newSP - target) < EPSILON)
		{
			//no longer ramping
			setIntegerParam(P_Ramping, 0);
		}

		callParamCallbacks();

		//check PID table in use
		getIntegerParam(P_LookUpOn, &lookUpOn);

		if (lookUpOn)
		{
			checkLookUp(newSP, curSP);
		}
	}

}

int ReadASCII::getSPInd (double SP)
{
	//NOTE: this assumes the lookup table is in order

	for (int i = 0; i<rowNum; i++)
	{
		double SPLookUp = pSP_[i];
		if (SP < SPLookUp)
		{
			if (i==0)
			{
				std::cerr << "SP Out of Look Up Range" << std::endl;
				return 0;
			}
			else
				return i-1;
		}
	}

	return rowNum-1;
}

void ReadASCII::updatePID(int index)
{
	setDoubleParam(P_P, pP_[index]);
	setDoubleParam(P_I, pI_[index]);
	setDoubleParam(P_D, pD_[index]);
	setDoubleParam(P_MaxHeat, pMaxHeat_[index]);
}

void ReadASCII::checkLookUp (double newSP, double oldSP)
{
	int newInd, oldInd;

	newInd = getSPInd(newSP);
	oldInd = getSPInd(oldSP);

	//check crossed an SP threshold
	if (newInd != oldInd)
	{
		updatePID(newInd);
	}
}

void ReadASCII::readFilePoll(void)
{
	char localDir[DIR_LENGTH], dirBase[DIR_LENGTH];
	asynStatus status;

	while (1) {
		
		//wait for a file change
		epicsThreadSleep(2.0);
		
		if (true == fileBad)
			continue;

		lock();

		//get directory in this thread
		getStringParam(P_DirBase, DIR_LENGTH, dirBase);
		getStringParam(P_Dir, DIR_LENGTH, localDir);

		//release lock
		unlock();

		strcat(dirBase, "/");
		strcat(dirBase, localDir);

		if (false == isModified(dirBase))
		{
			continue;
		}

		lock();

		status = readFile(dirBase);

		unlock();
	}
}

asynStatus ReadASCII::readFile(const char *dir)
{
	float SP, P, I, D, maxHeater;
	int ind = 0;
	FILE *fp;

	if (NULL != (fp = fopen(dir, "r"))) {

		//ignore first line
		fscanf(fp, "%*[^\n]\n");

		while (fscanf(fp, "%f %f %f %f %f", &SP, &P, &I, &D, &maxHeater) != EOF){
			pSP_[ind] = SP;
			pP_[ind] = P;
			pI_[ind] = I;
			pD_[ind] = D;
			pMaxHeat_[ind] = maxHeater;

			ind++;
		}

		rowNum = ind;

		callParamCallbacks();

		fclose(fp);
	}
	else {
		//send a file not found error?
		std::cerr << "File Open Failed: " << dir << std::endl;
		fileBad = true;
		return asynError;
	}	

	return asynSuccess;
	
}

bool ReadASCII::isModified(const char *checkDir)
{
	double diff = 0;
	struct stat buf;
	time_t newModified;

	if (stat(checkDir, &buf) >= 0)
	{
		newModified = buf.st_mtime;

		diff = difftime(newModified, lastModified);

		lastModified = newModified;

		if (0 == diff) return false;
		else return true;
	}
	else{
		//send a file not found error?
		std::cerr << "File Modified Check Failed: " << checkDir << std::endl;
		return true;
	}

}

void readFilePoll(void *drvPvt)
{
	ReadASCII *pPvt = (ReadASCII *)drvPvt;

	pPvt->readFilePoll();
}

void rampThread(void *drvPvt)
{
	ReadASCII *pPvt = (ReadASCII *)drvPvt;

	pPvt->rampThread();
}


extern "C" {

/// EPICS iocsh callable function to call constructor of ReadASCII().
/// \param[in] portName @copydoc initArg0
int ReadASCIIConfigure(const char *portName)
{
	try
	{
		new ReadASCII(portName);
		return(asynSuccess);
	}
	catch(const std::exception& ex)
	{
		std::cerr << "ReadASCIIDriver failed: " << ex.what() << std::endl;
		return(asynError);
	}
}

// EPICS iocsh shell commands 

static const iocshArg initArg0 = { "portName", iocshArgString};			///< The name of the asyn driver port we will create

static const iocshArg * const initArgs[] = { &initArg0 };

static const iocshFuncDef initFuncDef = {"ReadASCIIConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    ReadASCIIConfigure(args[0].sval);
}

static void ReadASCIIRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ReadASCIIRegister);

}

