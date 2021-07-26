#ifndef READASCII_H
#define READASCII_H

/// @file ReadASCII.h Header for read ASCII driver

#include "asynPortDriver.h"
#include <time.h>
#include <unordered_map>

#define DIR_LENGTH 260

/// Driver for Ramping SPs and Reading PIDs
class ReadASCII : public asynPortDriver 
{
public:
    ReadASCII(const char* portName, const char *searchDir, const int stepsPerMinute, const bool logOnSetPoint);

    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);

    void readFilePoll(void);
    void rampThread(void);

protected:
    // these are connected to PVs
    int P_Dir; // string
    int P_DirBase; // string
    int P_Index; //int

    int P_Ramping; //int
    int P_RampOn; //int
    int P_LookUpOn; //int

    int P_RampRate; //float
    int P_StepsPerMin; //float
    int P_CurTemp; //float
    int P_Target; //float
    int P_SPRBV; //float

    int* lastParam = 0;

    // for float64array values. <name, parameter index>
    std::unordered_map<std::string, int> pv_index_arrays;

    // for single float64 values, will be sent to updatePID. <name, parameter index>
    std::unordered_map<std::string, int> pv_index_values_updated;

    // first column in settings table, declaring setpoints for updatePID. <name, parameter index>
    std::pair<std::string, int> config_setpoint_value;

    // holds reference from parameter to settingsTable column
    std::unordered_map<std::string, std::vector<epicsFloat64>*> pv_data_column_reference;

private:
    // settings values table <column <value>>
    std::vector<std::vector<epicsFloat64>> settingsTable;

    epicsEventId eventId_;
    time_t lastModified;
    char lastDir[DIR_LENGTH];
    int rowNum; //keeps track of how many rows there are in settingsTable
    bool fileBad; //stops the driver from repeatedly checking bad files

    bool isModified(const char *checkDir);
    void checkLookUp (double newSP, double oldSP);
    int getSPInd (double SP);
    asynStatus readFileBasedOnParameters();
    std::vector<std::string> splitLine(std::string line);
    bool quietOnSetPoint;

    // updates values in control and tuning using settingsTable
    void updateControlValues(int index);

    // reads the file and updates settingsTable unless headers changed. retry is for situations where file was modified during run
    asynStatus readFile(const char* dir, bool retry = false);

    // opens lookup file and creates parameters from column headers
    asynStatus createParams(const char* dir);

    epicsFloat64 getSetpointValue(unsigned int tableRowIndex);

    enum class IndexType
    {
        ARRAY, VALUE, SETPOINT
    };
    // creates parameter and optionally associates column in settingsTable with data
    void addParameter(std::string name, asynParamType type, IndexType ind, std::vector<epicsFloat64>* columnReference = 0);

    // will update parameter with it's settingsTable lookup value and return value set
    epicsFloat64 setParamTableValue(std::string name, int paramIndex, unsigned int tableRow);
    
#define FIRST_READASCII_PARAM P_Dir
};

#define NUM_READASCII_PARAMS (lastParam - &FIRST_READASCII_PARAM + 1)
 
#define P_DirString "DIR"
#define P_DirBaseString "DIRBASE"
#define P_IndexString "IND"

#define P_RampingString "CURRMP"
#define P_RampOnString "RMP"
#define P_LookUpOnString "LUT"

#define P_TargetString "TGT"
#define P_SPRBVString "TGT:RBV"
#define P_RampRateString "RATE"
#define P_StepsPerMinString "STPNUM"
#define P_CurTempString "CUR"

#define ArrayPrefix "ARR"

#define FILE_FORMAT_INCORRECT "ReadASCII: Warning! File is incorrectly formatted"

#endif /* READASCII_H */
