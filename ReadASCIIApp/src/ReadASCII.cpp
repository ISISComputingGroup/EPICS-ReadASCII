#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <fstream>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <errlog.h>

#include <sys/stat.h>

#include "ReadASCII.h"

#include <macLib.h>
#include <epicsGuard.h>

#include <epicsExport.h>

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

    createParam(P_RampingString, asynParamInt32, &P_Ramping);
    createParam(P_RampOnString, asynParamInt32, &P_RampOn);
    createParam(P_LookUpOnString, asynParamInt32, &P_LookUpOn);

    createParam(P_TargetString, asynParamFloat64, &P_Target);
    createParam(P_SPRBVString, asynParamFloat64, &P_SPRBV);
    createParam(P_RampRateString, asynParamFloat64, &P_RampRate);
    createParam(P_StepsPerMinString, asynParamFloat64, &P_StepsPerMin);

    //Init
    setStringParam(P_Dir, "Default.txt");
    setStringParam(P_DirBase, searchDir);

    setDoubleParam(P_RampRate, 1.0);
    setDoubleParam(P_StepsPerMin, stepsPerMinute);
    setIntegerParam(P_Ramping, 0);
    setIntegerParam(P_RampOn, 0);
    setIntegerParam(P_LookUpOn, 0);

    char localDir[DIR_LENGTH], dir[DIR_LENGTH];
    getStringParam(P_DirBase, DIR_LENGTH, dir);
    getStringParam(P_Dir, DIR_LENGTH, localDir);
    strcat(dir, "/");
    strcat(dir, localDir);
    status = createParams(dir);

    createParam(P_CurTempString, asynParamFloat64, &P_CurTemp);

    // Do no initialise P_SPOut here because it will be sent to the eurotherm

    lastModified = 0;
    fileBad = true; //Set so that program doesn't attempt to read file before base dir is set
    rowNum = 0;
    quietOnSetPoint = setQuietOnSetPoint;

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

    if (function == P_Dir || function == P_DirBase) {
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
                std::unordered_map<std::string, int>::iterator it;
                for (it = pv_index_values_updated.begin(); it != pv_index_values_updated.end(); it++)
                {
                    setParamTableValue(it->first, it->second, value);
                }
                setParamTableValue(config_setpoint_value.first, config_setpoint_value.second, value);
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
                updateControlValues(getSPInd(curTemp));
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
            setDoubleParam(config_setpoint_value.second, startTemp);
            if (!quietOnSetPoint) {
                std::cerr << "ReadASCII: Setting SP to " << startTemp << " and ramping" << std::endl;
            }
            //update PIDs
            if (LUTOn)
            {
                updateControlValues(getSPInd(startTemp));
            }

            //start ramping
            setIntegerParam(P_Ramping, 1);

            //send event to start ramp
            epicsEventSignal(eventId_);

        } else {
            //directly output SP
            setDoubleParam(config_setpoint_value.second, value);
            setIntegerParam(P_Ramping, 0);
            if (!quietOnSetPoint) {
                std::cerr << "ReadASCII: Setting SP to " << value << " (no ramp)" << std::endl;
            }
            //update PIDs
            if (LUTOn)
            {
                updateControlValues(getSPInd(value));
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
    std::unordered_map<std::string, int>::iterator it;
    for (it = pv_index_arrays.begin(); it != pv_index_arrays.end(); it++)
    {
        if (pv_data_column_reference.find(it->first) != pv_data_column_reference.end())
        {
            try
            {
                if (function == it->second) {
                    memcpy(value, pv_data_column_reference[it->first]->data(), ncopy * sizeof(epicsFloat64));
                }
            }
            catch (std::out_of_range)
            {
                std::cerr << FILE_FORMAT_INCORRECT << std::endl;
            }
        }
        else
        {
            std::cerr << "ReadASCII: WARNING - parameter name: " << it->first << " doesn't have associated table column" << std::endl;
        }
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
        getDoubleParam(config_setpoint_value.second, &curSP);

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
        getDoubleParam(config_setpoint_value.second, &curSP);

        //target may have changed
        double oldTarget = target;
        getDoubleParam(P_Target, &target);

        if (oldTarget != target)
        {
            //start back at current temp
            getDoubleParam(P_CurTemp, &curSP);
            setDoubleParam(config_setpoint_value.second, curSP);
            std::cerr << "ReadASCII: RAMP: new SP " << newSP << std::endl;
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

        setDoubleParam(config_setpoint_value.second, newSP);
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

int ReadASCII::getSPInd (double SP)
{
    //NOTE: this assumes the lookup table is in order

    for (int i = 0; i<rowNum; i++)
    {
        double SPLookUp = getSetpointValue(i);
        if (SP < SPLookUp)
        {
            if (i==0)
            {
                std::cerr << "ReadASCII: SP below Look Up Lower Range, " << SP << " < " << getSetpointValue(0) << std::endl;
                return 0;
            }
            else
                return i - 1;
        }
    }
    if (rowNum > 0)
    {
        std::cerr << "ReadASCII: SP above Look Up Higher Range, " << SP << " > " << getSetpointValue(rowNum-1) << std::endl;
    }
    return rowNum - 1;
}

void ReadASCII::updateControlValues(int index)
{
    if (index < 0 || index >= rowNum)
    {
        return;
    }
    std::unordered_map<std::string, int>::iterator it;
    std::string prnt = "READASCII: Setting new control and tuning values: ";
    for (it = pv_index_values_updated.begin(); it != pv_index_values_updated.end(); it++)
    {
        setDoubleParam(it->second, -1);
        float ret = setParamTableValue(it->first, it->second, index);
        prnt += it->first + ":" + std::to_string(ret) + ", ";
    }
    std::cerr << prnt + "\n";
}

void ReadASCII::checkLookUp (double newSP, double oldSP)
{
    //Checks if the SP has crossed a threshold in the file by comparing their indices
    int newInd, oldInd;

    newInd = getSPInd(newSP);
    oldInd = getSPInd(oldSP);

    if (newInd != oldInd)
    {
        updateControlValues(newInd);
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

asynStatus ReadASCII::readFile(const char *dir, bool retry)
{
    std::ifstream file(dir);
    std::string line;

    std::vector<std::string> headers;

    std::vector<std::vector<epicsFloat64>> settingsTableNew;


    int i = -1;
    bool fileEdited = false;
    while (std::getline(file, line, '\n'))
    {
        // grab first line to get column headers
        if (i == -1)
        {
            for (std::string str : splitLine(line))
            {
                bool paramFound = false;

                // find out if this parameter exists
                if (config_setpoint_value.first == str)
                {
                    paramFound = true;
                }
                std::unordered_map<std::string, int>::iterator it;
                for (it = pv_index_values_updated.begin(); it != pv_index_values_updated.end(); it++)
                {
                    if (it->first == str)
                    {
                        paramFound = true;
                    }
                }
                if (paramFound)
                {
                    // if parameter exists then we can associate data
                    headers.push_back(str);
                }
                else
                {
                    fileEdited = true;
                }
            }
        }
        else
        {
            // initialize column vectors from first line
            int j = 0;
            if (i == 0)
            {
                for (std::string str : splitLine(line))
                {
                    if (j < headers.size())
                    {
                        settingsTableNew.push_back(std::vector<epicsFloat64>());
                    }
                    j++;
                }
            }
            j = 0;
            for (std::string str : splitLine(line))
            {
                try
                {
                    if (j < headers.size())
                    {
                        settingsTableNew[j].push_back(epicsFloat64(std::stof(str)));
                    }
                }
                catch (std::out_of_range)
                {
                    std::cerr << FILE_FORMAT_INCORRECT << std::endl;
                }
                catch (std::invalid_argument)
                {
                    std::cerr << FILE_FORMAT_INCORRECT << std::endl;
                }
                j++;
            }
        }
        i++;
    }

    if (!fileEdited || retry)
    {
        settingsTable = settingsTableNew;

        rowNum = i;
        std::cerr << "ReadASCII: read " << rowNum << " lines from file: " << dir << std::endl;
        file.close();
        pv_data_column_reference.clear();
        for (int j = 0; j < settingsTable.size(); j++)
        {
            try
            {
                pv_data_column_reference[ArrayPrefix + headers[j]] = &settingsTable[j];
                pv_data_column_reference[headers[j]] = &settingsTable[j];
                doCallbacksFloat64Array(settingsTable[j].data(), settingsTable[j].size(), pv_index_arrays[ArrayPrefix + headers[j]], 0);
            }
            catch (std::out_of_range)
            {
                std::cerr << FILE_FORMAT_INCORRECT << std::endl;
            }
        }

        fileBad = false;
    }
    else
    {
        std::cerr << "ReadASCII: File headers modified during run \n";
        settingsTable.clear();
        pv_data_column_reference.clear();
        createParams(dir);
        readFile(dir, true);
    }
    return asynSuccess;


}

asynStatus ReadASCII::createParams(const char* dir)
{
    std::ifstream file(dir);
    std::string line;
    std::getline(file, line, '\n');

    std::vector<std::string> names;

    for (std::string str : splitLine(line))
    {
        names.push_back(str);
    }

    int j = 0;

    for (std::string name : names)
    {
        std::string a = ArrayPrefix;
        addParameter(a + names[j], asynParamFloat64Array, ReadASCII::IndexType::ARRAY);
        if (j == 0)
        {
            addParameter(names[j], asynParamFloat64, ReadASCII::IndexType::SETPOINT);
        }
        else
        {
            addParameter(names[j], asynParamFloat64, ReadASCII::IndexType::VALUE);
        }
        j++;
    }
    return asynSuccess;
}

epicsFloat64 ReadASCII::getSetpointValue(unsigned int tableRowIndex)
{
    try
    {
        if (pv_data_column_reference.find(config_setpoint_value.first) != pv_data_column_reference.end())
        {
            return epicsFloat64(pv_data_column_reference[config_setpoint_value.first]->at(tableRowIndex));
        }
        else
        {
            std::cerr << "ReadASCII: Setpoint: " << config_setpoint_value.first << " does not have associated table data \n";
        }
    }
    catch (std::out_of_range e)
    {
        std::cerr << "ReadASCII: Exception Caught when reading setpoint parameter: " << e.what();
    }
    std::cerr << "ReadASCII: Critical error, could not read setpoint, unexpected behaviour inbound \n";
    return epicsFloat64(0);
}

std::vector<std::string> ReadASCII::splitLine(std::string line)
{
    std::vector<std::string> out;
    while (line.size() > 0)
    {
        std::size_t pos = line.find(' ');
        if (pos == std::string::npos)
        {
            break;
        }
        out.push_back(line.substr(0, pos));
        line = line.substr(pos + 1);
    }
    out.push_back(line);
    return out;
}

void ReadASCII::addParameter(std::string name, asynParamType type, IndexType ind, std::vector<epicsFloat64>* columnReference)
{
    switch (ind)
    {
    case ReadASCII::IndexType::ARRAY:
    {
        if (pv_index_arrays.find(name) == pv_index_arrays.end())
        {
            pv_index_arrays[name] = 0;
            if (createParam(name.c_str(), type, &pv_index_arrays[name]) == asynSuccess)
            {
                lastParam = &pv_index_arrays[name];
            }
        }
        break;
    }
    case ReadASCII::IndexType::VALUE:
    {
        if (pv_index_values_updated.find(name) == pv_index_values_updated.end())
        {
            pv_index_values_updated[name] = 0;
            if (createParam(name.c_str(), type, &pv_index_values_updated[name]) == asynSuccess)
            {
                lastParam = &pv_index_values_updated[name];
            }
        }
        break;
    }
    case ReadASCII::IndexType::SETPOINT:
    {
        config_setpoint_value = std::make_pair(name, 0);
        if (createParam(name.c_str(), type, &config_setpoint_value.second) == asynSuccess)
        {
            lastParam = &config_setpoint_value.second;
        }
        break;
    }
    }
    try
    {
        if (columnReference != 0)
        {
            pv_data_column_reference[name] = columnReference;
        }
    }
    catch (std::out_of_range)
    {
        std::cerr << FILE_FORMAT_INCORRECT << std::endl;
    }
}

epicsFloat64 ReadASCII::setParamTableValue(std::string name, int paramIndex, unsigned int tableRow)
{
    if (pv_data_column_reference.find(name) != pv_data_column_reference.end())
    {
        try
        {
            epicsFloat64 f = pv_data_column_reference[name]->at(tableRow);
            setDoubleParam(paramIndex, f);
            return f;
        }
        catch (std::out_of_range e)
        {
            std::cerr << "ReadASCII: Exception Caught when writing to parameter: " << e.what();
        }
    }
    else
    {
        std::cerr << "ReadASCII: WARNING - parameter name: " << name << " doesn't have associated table column. Setting value to zero" << std::endl;
    }
    setDoubleParam(paramIndex, 0);
    return epicsFloat64(0.0f);
}

bool ReadASCII::isModified(const char *checkDir)
{
    //Checks if a given directory has been modified since last check. Returns true if modified.
    double diff = 0.0;
    struct stat buf;
    time_t newModified;

    if (stat(checkDir, &buf) >= 0)
    {
        newModified = buf.st_mtime;

        diff = difftime(newModified, lastModified);

        lastModified = newModified;

        if (0.0 == diff) return false;
        else return true;
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

