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
ReadASCII::ReadASCII(const char *portName, const char *searchDir)
   : asynPortDriver(portName, 
                    0, /* maxAddr */ 
                    NUM_READASCII_PARAMS,
					asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
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
	createParam(P_SPRBVString, asynParamFloat64, &P_SPRBV);
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
	setStringParam(P_Dir, "Default.txt");
	setStringParam(P_DirBase, searchDir);

	setDoubleParam(P_RampRate, 1.0);
	setIntegerParam(P_Ramping, 0);
	setIntegerParam(P_RampOn, 0);
	setIntegerParam(P_LookUpOn, 0);

	// Do no initialise P_SPOut here because it will be sent to the eurotherm

	lastModified = 0;
	fileBad = true; //Set so that program doesn't attempt to read file before base dir is set

	/* Create the thread that watches the file in the background 	*/
	status = (asynStatus)(epicsThreadCreate("ReadASCIIFile",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC)::readFilePoll,
		this) == NULL);
	/* Create the thread that ramps the output 	*/
	if (status == asynSuccess)
	{	
        status = (asynStatus)(epicsThreadCreate("ReadASCIIRamp",
		    epicsThreadPriorityMedium,
		    epicsThreadGetStackSize(epicsThreadStackMedium),
		    (EPICSTHREADFUNC)::rampThread,
		    this) == NULL);
	}
	if (status) {
		std::cerr << status << "epicsThreadCreate failure\n";
		return;
	}

}

asynStatus ReadASCII::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
	//Checks if directory has changed, reads file again if it has
	int function = pasynUser->reason;
	int status = asynSuccess;
	const char *paramName;
	const char* functionName = "writeOctet";

	/* Set the parameter in the parameter library. */
	status = (asynStatus)setStringParam(function, value);

	/* Fetch the parameter string name for possible use in debugging */
	getParamName(function, &paramName);

	if (function == P_Dir | function == P_DirBase) {
		// Directory has changed so update
		status |= readFileBasedOnParameters();
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

/// Read Values as Octet
asynStatus ReadASCII::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
	//Checks if directory has changed, reads file again if it has
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *paramName;
	const char* functionName = "readOctet";

	/* Fetch the parameter string name for possible use in debugging */
	getParamName(function, &paramName);

	if (function == P_DirBase) {
		char dirBase[DIR_LENGTH];
		getStringParam(P_DirBase, DIR_LENGTH, dirBase);

		strncpy(value, dirBase, maxChars);
		*nActual = strlen(dirBase);
		std::cerr << status << "new dir base " << dirBase << std::endl;
	} else {
		status = asynError;
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"%s:%s: status=%d, function=%d, name=%s function does not have read.",
				driverName, functionName, status, function, paramName);
	}

	return status;

}

/// Read the file ramp file based on the parameters in dir and base
asynStatus ReadASCII::readFileBasedOnParameters() {

	char localDir[DIR_LENGTH], dirBase[DIR_LENGTH * 2 + 1];
	getStringParam(P_DirBase, DIR_LENGTH, dirBase);
	getStringParam(P_Dir, DIR_LENGTH, localDir);
	strcat(dirBase, "/");
	strcat(dirBase, localDir);
	asynStatus status = readFile(dirBase);
	setParamStatus(P_Dir, (asynStatus) status);
	return status;
}

asynStatus ReadASCII::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	//Checks for updates to the index and on/off of PID lookup
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
		//reload file if bad
		if (true == fileBad)
		{
			status = readFileBasedOnParameters();
		}

		//file may now be good - retry
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
	//Deals with the user changing the SP target
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


asynStatus ReadASCII::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
	size_t nElements, size_t *nIn)
{
	//Return arrays when waveform is scanned
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
    const double SECONDS_IN_MINUTE = 60.0;
	//Ramps SP values when the ramp is on
	double wait, rate, target, curSP, newSP, SPRBV;
	int ramping, rampOn, lookUpOn;

	lock();

	while (1)
	{
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

		//get SP:RBV
		getDoubleParam(P_SPRBV, &SPRBV);

		//get current SP
		getDoubleParam(P_SPOut, &curSP);

		//get target
		getDoubleParam(P_Target, &target);

		if ((abs(SPRBV - target) < EPSILON) && (abs(curSP - target) < EPSILON))
		{
			setIntegerParam(P_Ramping, 0);
			continue;
		}

		callParamCallbacks();
		unlock(); 
		wait = 5.0; //default wait

		//wait
		epicsEventWaitWithTimeout(eventId_, wait);

		//check near final SP (Doesn't work as requires long time to write to Eurotherm) 
		//if (diff < (wait * rate))
		//{
		//	//wait less time
		//	wait = diff / rate;
		//}
		
		lock();

		//check rampOn (could have changed whilst waiting)
		getIntegerParam(P_RampOn, &rampOn);
		
		if (!rampOn)
		{
			//no longer ramping
			setIntegerParam(P_Ramping, 0);
			continue;
		}

		//rate may have changed whilst waiting
		getDoubleParam(P_RampRate, &rate);
        // convert from K/min to K/s
        rate /= SECONDS_IN_MINUTE;
			
		//SP may have changed
		getDoubleParam(P_SPOut, &curSP);

		//target may have changed
		double oldTarget = target;
		getDoubleParam(P_Target, &target);

		if (oldTarget != target)
		{
			//start back at current temp
			getDoubleParam(P_CurTemp, &curSP);
			setDoubleParam(P_SPOut, curSP);
			continue;
		}

		double diff = abs(target - curSP);

		if (diff < (wait * rate))
		{
			newSP = target;
		}else{
			if (curSP > target)
				rate = -rate;

			//update SP with wait*rate
			newSP = curSP + wait*rate;
		}

		setDoubleParam(P_SPOut, newSP);

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
	//Updates the PID and max heater values to those in the file on the given line
	setDoubleParam(P_P, pP_[index]);
	setDoubleParam(P_I, pI_[index]);
	setDoubleParam(P_D, pD_[index]);
	setDoubleParam(P_MaxHeat, pMaxHeat_[index]);
}

void ReadASCII::checkLookUp (double newSP, double oldSP)
{
	//Checks if the SP has crossed a threshold in the file by comparing their indices
	int newInd, oldInd;

	newInd = getSPInd(newSP);
	oldInd = getSPInd(oldSP);

	if (newInd != oldInd)
	{
		updatePID(newInd);
	}
}

void ReadASCII::readFilePoll(void)
{
	//Thread to poll the file used in the PID lookup and update the array of values when the file is modified
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
	//Reads PID values from a file and places them in an array
	float SP, P, I, D, maxHeater;
	int ind = 0;
	FILE *fp;

	if (NULL != (fp = fopen(dir, "r"))) {

		//ignore first line
		fscanf(fp, "%*[^\n]\n");

		int result = fscanf(fp, "%f %f %f %f %f", &SP, &P, &I, &D, &maxHeater);

		//check for incorrect format
		if (result < 5) {
			std::cerr << "File format incorrect" << std::endl;
			fileBad = true;
			return asynError;
		}

		do{
			pSP_[ind] = SP;
			pP_[ind] = P;
			pI_[ind] = I;
			pD_[ind] = D;
			pMaxHeat_[ind] = maxHeater;

			ind++;

		} while (fscanf(fp, "%f %f %f %f %f", &SP, &P, &I, &D, &maxHeater) != EOF);

		rowNum = ind;

		callParamCallbacks();

		fclose(fp);
	}
	else {
		//send a file not found error
		std::cerr << "File Open Failed: " << dir << std::endl;
		fileBad = true;
		return asynError;
	}	

	fileBad = false;
	return asynSuccess;
	
}

bool ReadASCII::isModified(const char *checkDir)
{
	//Checks if a given directory has been modified since last check. Returns true if modified.
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
	//Set up a thread to read the PID file
	ReadASCII *pPvt = (ReadASCII *)drvPvt;

	pPvt->readFilePoll();
}

void rampThread(void *drvPvt)
{
	//Set up a thread to ramp the SP value
	ReadASCII *pPvt = (ReadASCII *)drvPvt;

	pPvt->rampThread();
}


extern "C" {

/// EPICS iocsh callable function to call constructor of ReadASCII().
/// \param[in] portName @copydoc initArg0
/// \param[in] rampDir @copydoc initArg0
int ReadASCIIConfigure(const char *portName, const char *rampDir)
{
	try
	{
		new ReadASCII(portName, rampDir);
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
static const iocshArg initArg1 = { "rampDir",iocshArgString};           ///< Initial ramp dir

static const iocshArg * const initArgs[] = { &initArg0, &initArg1 };

static const iocshFuncDef initFuncDef = {"ReadASCIIConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    ReadASCIIConfigure(args[0].sval, args[1].sval);
}

static void ReadASCIIRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ReadASCIIRegister);

}

