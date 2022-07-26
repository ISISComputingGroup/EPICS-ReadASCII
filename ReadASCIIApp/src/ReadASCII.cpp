#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <errno.h>
#include <cmath>
#include <exception>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <errlog.h>
#include <fstream>
#include <istream>
#include <sstream>

#include <sys/stat.h>

#include <macLib.h>
#include <epicsGuard.h>
#include "asynPortDriver.h"

#include <epicsExport.h>

#include "ReadASCII.h"

#define INIT_ROW_NUM 60
#define EPSILON 0.001

static const char *driverName="ReadASCII";
void readFilePoll(void *drvPvt);
void rampThread(void *drvPvt);

/// Constructor for the ReadASCII class.
/// Calls constructor for the asynPortDriver base class.
ReadASCII::ReadASCII(const char *portName, const char *searchDir, const int stepsPerMinute, const bool setQuietOnSetPoint)
   : asynPortDriver(portName, 
                    0, /* maxAddr */ 
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

    addParameter(P_DirString, asynParamOctet, &P_Dir);
    addParameter(P_DirBaseString, asynParamOctet, &P_DirBase);
    addParameter(P_IndexString, asynParamInt32, &P_Index);

    addParameter(P_RampingString, asynParamInt32, &P_Ramping);
    addParameter(P_RampOnString, asynParamInt32, &P_RampOn);
    addParameter(P_LookUpOnString, asynParamInt32, &P_LookUpOn);
	addParameter(P_LookUpTableChangedString, asynParamInt32, &P_LookUpTableChanged);

    addParameter(P_TargetString, asynParamFloat64, &P_Target);
    addParameter(P_SPRBVString, asynParamFloat64, &P_SPRBV);
    addParameter(P_RampRateString, asynParamFloat64, &P_RampRate);
    addParameter(P_StepsPerMinString, asynParamFloat64, &P_StepsPerMin);
    addParameter(P_CurTempString, asynParamFloat64, &P_CurTemp);

    addParameter(P_SPOutString, asynParamFloat64);

    //Init
    setStringParam(P_Dir, DEFAULT_RAMP_FILE);
	setIntegerParam(P_LookUpTableChanged, 0);
    setStringParam(P_DirBase, searchDir);
    fileBad = true;
    rowNum = 0;
    lastModified = 0;
    quietOnSetPoint = setQuietOnSetPoint;
	

    setDoubleParam(P_RampRate, 1.0);
    setDoubleParam(P_StepsPerMin, stepsPerMinute);
    setIntegerParam(P_Ramping, 0);
    setIntegerParam(P_RampOn, 0);
    setIntegerParam(P_LookUpOn, 0);
	

    //Try to read file once to load parameters
    readFileBasedOnParameters();

    //From now on parameters need to be created dynamically
    dynamicParameters = true;

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
        std::cerr << "ReadASCII: epicsThreadCreate failure " << status << std::endl;
        return;
    }

}

// private constructor for testing
ReadASCII::ReadASCII()
   : asynPortDriver(std::to_string(rand()).c_str(), // this is so each test gets a unique port
                    0, /* maxAddr */ 
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
                    0, /* Autoconnect */
                    0, /* Default priority */
                    0)	/* Default stack size*/
{
}

asynStatus ReadASCII::drvUserCreate(asynUser* pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize)
{
    if (!dynamicParameters)
    {
        return asynPortDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
    }
    // Add missing parameter
    try
    {
        findParam(drvInfo);
    }
    catch (...)
    {
        std::cerr << "ReadASCII:: Parameter missing: " << drvInfo << ", adding dynamically." << std::endl;
        std::string name = std::string(drvInfo);
        if (name.compare(0, strlen(ARRAY_PARAMETER_PREFIX), ARRAY_PARAMETER_PREFIX) == 0) {
            // Callback will be set when the table values updates (populateLookupTable code)
            addParameter(name, asynParamFloat64Array);
        }
        else
        {
            addParameter(name, asynParamFloat64);
        }
    }
    return asynPortDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
}

asynStatus ReadASCII::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    //Checks if directory has changed, reads file again if it has
    int function = pasynUser->reason;
    int status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeOctet";
	char localDir[DIR_LENGTH];

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_Dir || function == P_DirBase) {
        // Directory has changed so update
        status |= readFileBasedOnParameters();
		
		// If directory has changed (and not to default) then update
		getStringParam(P_Dir, DIR_LENGTH, localDir);
		if(strncmp(DEFAULT_RAMP_FILE, localDir, DIR_LENGTH) == 0){
			setIntegerParam(P_LookUpTableChanged, 0);
		} else {
			setIntegerParam(P_LookUpTableChanged, 1);
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
    int LUTOn = 0;

    getParamName(function, &paramName);
    /* Set the parameter in the parameter library. */
    status = (asynStatus)setIntegerParam(function, value);

    if (function == P_Index) {
        //check lookup on
        getIntegerParam(P_LookUpOn, &LUTOn);

        if (LUTOn)
        {
            if (value >= 0 && value < rowNum)
            {
                if (!quietOnSetPoint) {
                    std::cerr << "ReadASCII: Updating values: " << std::endl;
                }
                for (const LookupTableColumn& col : lookupTable)
                {
                    updateParameter(col.values[value], col.header);
                    if (!quietOnSetPoint) {
                        std::cerr << col.header << "=" << col.values[value] << " ";
                    }
                }
                if (!quietOnSetPoint) {
                    std::cerr << std::endl;
                }
            }
            else
            {
                errlogSevPrintf(errlogMajor, "Index %d out of range for lookup table", value);
            }
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
            setIntegerParam(P_LookUpOn, value);
            //uses the current temperature to find PID values
            if (value)
            {
                double curTemp;
                getDoubleParam(P_CurTemp, &curTemp);
                updateTableValues(getSPInd(curTemp));
            }
        }
        else
        {
            setIntegerParam(P_LookUpOn, 0); // turn off lookup if file bad
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
    int rampOn = 0, LUTOn = 0;
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
            updateParameter(startTemp, P_SPOutString);
            if (!quietOnSetPoint) {
                std::cerr << "ReadASCII: Setting SP to " << startTemp << " and ramping" << std::endl;
            }
            //update PIDs
            if (LUTOn)
            {
                updateTableValues(getSPInd(startTemp));
            }

            //start ramping
            setIntegerParam(P_Ramping, 1);

            //send event to start ramp
            epicsEventSignal(eventId_);

        } else {
            //directly output SP
            updateParameter(value, P_SPOutString);
            setIntegerParam(P_Ramping, 0);
            if (!quietOnSetPoint) {
                std::cerr << "ReadASCII: Setting SP to " << value << " (no ramp)" << std::endl;
            }
            //update PIDs
            if (LUTOn)
            {
                updateTableValues(getSPInd(value));
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

    ncopy = ( (nElements < rowNum) ? nElements : rowNum );
    std::string header = findParam(function)->name;
    try
    {
        const LookupTableColumn* column = findColumnByHeader(header.substr(std::string(ARRAY_PARAMETER_PREFIX).size()));
        memcpy(value, column->values.data(), ncopy * sizeof(epicsFloat64));
        *nIn = ncopy;
    }
    catch (...)
    {
        std::cerr << "ReadASCII: unable to retrieve array data" << std::endl;
        *nIn = 0;
        status = asynError;
    }

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
    double wait, rate, target, curSP, newSP, SPRBV, stepsPerMin;
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
        getDoubleParam(findParam(P_SPOutString)->index, &curSP);

        //get target
        getDoubleParam(P_Target, &target);

        //get steps per minute
        getDoubleParam(P_StepsPerMin, &stepsPerMin);

        if ((abs(SPRBV - target) < EPSILON) && (abs(curSP - target) < EPSILON))
        {
            setIntegerParam(P_Ramping, 0);
            callParamCallbacks();
            continue;
        }

        if (P_StepsPerMin < EPSILON) {
            wait = 5.0; //default wait
        } else {
            wait = SECONDS_IN_MINUTE/stepsPerMin;
        }

        unlock(); 

        //wait
        epicsEventWaitWithTimeout(eventId_, wait);
        
        lock();

        //check rampOn (could have changed whilst waiting)
        getIntegerParam(P_RampOn, &rampOn);
        
        if (!rampOn)
        {
            //no longer ramping
            setIntegerParam(P_Ramping, 0);
            callParamCallbacks();
            continue;
        }

        //rate may have changed whilst waiting
        getDoubleParam(P_RampRate, &rate);
        // convert from K/min to K/s
        rate /= SECONDS_IN_MINUTE;
            
        //SP may have changed
        getDoubleParam(findParam(P_SPOutString)->index, &curSP);

        //target may have changed
        double oldTarget = target;
        getDoubleParam(P_Target, &target);

        if (oldTarget != target)
        {
            //start back at current temp
            getDoubleParam(P_CurTemp, &curSP);
            updateParameter(curSP, P_SPOutString);
            //setDoubleParam(findParamByName(P_SPOutString)->index, curSP);
            std::cerr << "ReadASCII: RAMP: new SP " << curSP << std::endl;
            callParamCallbacks();
            continue;
        }

        double diff = abs(target - curSP);

        if (diff < (wait * rate)) {
            newSP = target;
        } else {
            if (curSP > target)
                rate = -rate;

            //update SP with wait*rate
            newSP = curSP + wait*rate;
        }

        updateParameter(newSP, P_SPOutString);
        //setDoubleParam(findParamByName(P_SPOutString)->index, newSP);
        std::cerr << "ReadASCII: RAMP: new SP " << newSP << std::endl;

        //check PID table in use
        getIntegerParam(P_LookUpOn, &lookUpOn);
        if (lookUpOn)
        {
            checkLookUp(newSP, curSP);
        }
        
        callParamCallbacks();
    }

}

// if file is bad, rowNum == 0 and so returns -1
int ReadASCII::getSPInd (double SP)
{
    try
    {
        for (int i = 0; i < rowNum; i++)
        {
            double SPLookUp = lookupTable[0].values[i];
            if (SP < SPLookUp)
            {
                if (i == 0)
                {
                    std::cerr << "ReadASCII: SP below Look Up Lower Range, " << SP << " < " << lookupTable[0].values[0] << std::endl;
                    return 0;
                }
                else
                    return i - 1;
            }
        }
        if (rowNum > 0)
        {
            std::cerr << "ReadASCII: SP above Look Up Higher Range, " << SP << " > " << lookupTable[0].values[rowNum - 1] << std::endl;
        }
    }
    catch (...) 
    {
        std::cerr << "ReadASCII: Lookup table critical error, SP column is missing" << std::endl;
    }
    return rowNum - 1;
}

void ReadASCII::updateTableValues(int index)
{
    if (index < 0 || index >= rowNum)
    {
        return;
    }
    // Set to minus one first and then the actual value.
    // This marks the value as "changed" so that the monitor actually gets fired.
    // start loop at 1 as lookupTable[0] is the SP
    std::cerr << "ReadASCII: Updating values: ";
    for (int i = 1; i < lookupTable.size(); i++)
    {
        updateParameter(-1, lookupTable[i].header);
        updateParameter(lookupTable[i].values[index], lookupTable[i].header);
        std::cerr << lookupTable[i].header << "=" << lookupTable[i].values[index] << " ";
    }
    std::cerr << std::endl;
}

void ReadASCII::checkLookUp (double newSP, double oldSP)
{
    //Checks if the SP has crossed a threshold in the file by comparing their indices
    int newInd, oldInd;

    newInd = getSPInd(newSP);
    oldInd = getSPInd(oldSP);

    if (newInd != oldInd)
    {
        updateTableValues(newInd);
    }
}

void ReadASCII::readFilePoll(void)
{
    //Thread to poll the file used in the PID lookup and update the array of values when the file is modified
    char localDir[DIR_LENGTH], dirBase[DIR_LENGTH * 2  + 1];
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
    std::ifstream file(dir);
    if (!file.good())
    {
        //send a file not found error
        std::cerr << "ReadASCII: File Open Failed: " << dir << ": " << strerror(errno) << std::endl;
        fileBad = true;
        rowNum = 0;
        return asynError;
    }
    std::vector<std::vector<std::string>> valuesTable = splitStreamToColumns(file);
    file.close();

    populateLookupTable(valuesTable);
    addTableParameters();

    std::cerr << "ReadASCII: read " << rowNum + 1 << " lines from file: " << dir << std::endl;

    fileBad = false;
    return asynSuccess;
    
}

void ReadASCII::addTableParameters()
{
    // If any of this is already done its fine, it will simply do nothing
    for (LookupTableColumn& column : lookupTable)
    {
        // findParam throws if it doesnt find parameter so we catch it to create params
        try
        {
            findParam(column.header);
        }
        catch (...)
        {
            addParameter(column.header, asynParamFloat64);
        }
        try
        {
            findParam(ARRAY_PARAMETER_PREFIX + column.header);
        }
        catch (...)
        {
            addParameter(ARRAY_PARAMETER_PREFIX + column.header, asynParamFloat64Array);
        }
        doCallbacksFloat64Array(column.values.data(), column.rows, findParam(ARRAY_PARAMETER_PREFIX + column.header)->index, 0);
    }
}

asynStatus ReadASCII::populateLookupTable(const std::vector<std::vector<std::string>>& values)
{
    lookupTable = std::vector<LookupTableColumn>();

    //first column are setpoints - grab expected format from it
    if (values.size() == 0)
    {
        std::cerr << "ReadASCII: Warning - lookup file is empty" << std::endl;
        return asynError;
    }
    rowNum = values[0].size() - 1; // -1 as header row

    //for every column
    for (const std::vector<std::string>& column : values)
    {
        LookupTableColumn newCol = LookupTableColumn();
        int rowCountColumn = 0;

        //first value is column header
        if (column.size() > 0)
        {
            std::string header = column[0];
            boost::algorithm::to_lower(header);
            if (header == "heater")
            {
                std::cerr << "ReadASCII: Warning! Outdated column header name is in use. Please view latest changes to ReadASCII. " <<
                    "'Heater' will be replaced with 'MH' when reading the file." << std::endl;
                header = "MH";
                newCol.header = "MH";
            }
            else
            {
                newCol.header = column[0];
            }
        }
        else
        {
            std::cerr << "ReadASCII: Empty column detected in lookup table" << std::endl;
        }

        //for every value in column
        for (int i = 1; i < column.size(); i++)
        {
            //rest of values in a column should be floats
            try
            {
                newCol.values.push_back(std::stof(column[i]));
            }
            catch (std::invalid_argument e)
            {
                std::cerr << "ReadASCII: Not-a-number value in lookup table found while trying to access " << i << " element in column: " << newCol.header <<
                    " Missing values will be set to 0." << std::endl;
                newCol.values.push_back(epicsFloat64(0.0f));
            }
            rowCountColumn++;
        }
        //check if there is a match between setpoints column and other columns
        if (rowCountColumn != rowNum)
        {
            std::cerr << "ReadASCII: Warning - column malformed. Incorrect amount of rows in column: " << newCol.header << " Missing values will be set to 0." << std::endl;
            while (rowCountColumn < rowNum)
            {
                newCol.values.push_back(epicsFloat64(0.0f));
                rowCountColumn++;
            }
        }
        newCol.rows = rowCountColumn;
        lookupTable.push_back(newCol);
    }
    return asynSuccess;
}

std::vector<std::vector<std::string>> ReadASCII::splitStreamToColumns(std::istream& stream)
{
    std::vector<std::vector<std::string>> columns = std::vector<std::vector<std::string>>();

    int i = 0;
    std::string line;
    while (std::getline(stream, line, '\n'))
    {
        std::vector<std::string> split_line = splitLine(line, ' ');
        int j = 0;
        while (columns.size() < split_line.size())
        {
            columns.push_back(std::vector<std::string>());
        }

        for (const std::string& s : split_line)
        {
            columns[j].push_back(s);
            j++;
        }

        i++;
    }

    return columns;
}

std::vector<std::string> ReadASCII::splitLine(const std::string& line, char delim)
{
    std::vector<std::string> out;
    std::istringstream iss(line);
    std::string str;
    while (std::getline(iss, str, delim)) { out.push_back(str); }
    return out;
}

asynStatus ReadASCII::addParameter(const std::string& name, const asynParamType& type, int* index)
{
    Parameter param;
    param.name = name;
    param.type = type;
    asynStatus status = asynSuccess;
    status = createParam(name.c_str(), type, &param.index);
    if (status == asynSuccess)
    {
        //link parameter struct index to another index variable if needed
        if (index != nullptr)
        {
            *index = param.index;
        }
        parameters.push_back(param);
        std::cerr << "ReadASCII: Added parameter: " << name <<", type: "<< param.type<< ", new index: " << param.index <<std::endl;
    }
    return status;
}

ReadASCII::Parameter* ReadASCII::findParam(const std::string& name)
{
    std::vector<Parameter>::iterator it = std::find_if(parameters.begin(), parameters.end(),
        [&](Parameter param) {return param.name == name; });
    if (it != parameters.end())
    {
        return &(*it);
    }
    else
    {
        throw std::out_of_range("Parameter not found!");
    }
}

ReadASCII::Parameter* ReadASCII::findParam(const int& index)
{
    std::vector<Parameter>::iterator it = std::find_if(parameters.begin(), parameters.end(),
        [&](Parameter param) {return param.index == index; });
    if (it != parameters.end())
    {
        return &(*it);
    }
    else
    {
        throw std::out_of_range("Parameter not found!");
    }
}

asynStatus ReadASCII::updateParameter(epicsFloat64 value, const std::string& name)
{
    asynStatus status;
    try
    {
        status = setDoubleParam(findParam(name)->index, value);
    }
    catch (...)
    {
        std::cerr << "Error updating parameter " << name << std::endl;
        status = asynError;
    }
    return status;
}

const ReadASCII::LookupTableColumn* ReadASCII::findColumnByHeader(const std::string& header) const
{
    std::vector<LookupTableColumn>::const_iterator it = std::find_if(lookupTable.begin(), lookupTable.end(),
        [&](LookupTableColumn column) {return column.header == header; });
    if (it != lookupTable.end())
    {
        return &(*it);
    }
    else
    {
        throw std::out_of_range("Column not found!");
    }
}

bool ReadASCII::isModified(const char *checkDir)
{
    //Checks if a given directory has been modified since last check. Returns true if modified.
    double diff = 0.0;
    struct stat buf;

    if (stat(checkDir, &buf) == 0)
    {
        diff = difftime(buf.st_mtime, lastModified);

        lastModified = buf.st_mtime;

        return (0.0 == diff ? false : true);
    }
    else{
        //send a file not found error?
        std::cerr << "ReadASCII: File Modified Check Failed: " << checkDir << std::endl;
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
/// \param[in] rampDir @copydoc initArg1
/// \param[in] stepsPerMinute @copydoc initArg2
/// \param[in] logOnSetPoint @copydoc initArg3
int ReadASCIIConfigure(const char *portName, const char *rampDir, const int stepsPerMinute, const bool logOnSetPoint)
{
    try
    {
        new ReadASCII(portName, rampDir, stepsPerMinute, logOnSetPoint);
        return(asynSuccess);
    }
    catch(const std::exception& ex)
    {
        std::cerr << "ReadASCIIDriver failed: " << ex.what() << std::endl;
        return(asynError);
    }
}

// EPICS iocsh shell commands 

static const iocshArg initArg0 = { "portName", iocshArgString};			 ///< The name of the asyn driver port we will create
static const iocshArg initArg1 = { "PIDDir", iocshArgString};           ///< Initial directory for the PID file
static const iocshArg initArg2 = { "stepsPerMinute", iocshArgInt };      ///< Initial steps per minute
static const iocshArg initArg3 = { "quietOnSetPoint", iocshArgInt };     ///< Stop logging on every set point change set to 1 to supress logging (useful if you change the set points very frequently), defaults to 0

static const iocshArg * const initArgs[] = { &initArg0, &initArg1, &initArg2, &initArg3 };

static const iocshFuncDef initFuncDef = {"ReadASCIIConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    ReadASCIIConfigure(args[0].sval, args[1].sval, args[2].ival, (bool)args[3].ival);
}

static void ReadASCIIRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ReadASCIIRegister);

}

