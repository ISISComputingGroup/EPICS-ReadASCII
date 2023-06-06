#ifndef READASCII_H
#define READASCII_H

/// @file ReadASCII.h Header for read ASCII driver

#include <time.h>
#include <map>
#include "asynPortDriver.h"

#define DIR_LENGTH 260
#define DEFAULT_RAMP_FILE "Default.txt"

class ReadASCIITest;

/// Driver for Ramping SPs and Reading PIDs
class epicsShareClass ReadASCII : public asynPortDriver 
{
    friend class ReadASCIITest;
    
public:
    ReadASCII(const char* portName, const char *searchDir, const double rampRate, const int stepsPerMinute, const bool logOnSetPoint);

    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);

    void readFilePoll(void);
    void rampThread(void);

protected:
    int P_Dir; // string
    int P_DirBase; // string
    int P_Index; //int

    int P_Ramping; //int
    int P_RampOn; //int
    int P_LookUpOn; //int
	int P_LookUpTableNotDefault; //int

    int P_RampRate; //float
    int P_StepsPerMin; //float
    int P_CurTemp; //float
    int P_Target; //float
    int P_SPRBV; //float

    //Defines information about a parameter
    struct Parameter
    {
        int index;
        std::string name;
        asynParamType type;

        Parameter() {}
    };

    //Holds all parameters in use by ReadASCII
    std::vector<Parameter> parameters;

    struct LookupTableColumn
    {
        std::string header;
        std::vector<epicsFloat64> values;
        int rows;

        LookupTableColumn() {}
    };

    //LookupTableColumn lookupSetpoints;
    //First column is always reserved for setpoints
    std::vector<LookupTableColumn> lookupTable;

private:
    ReadASCII(); // private constructor, only used by test framework

    //This is for dynamically creating asyn parameters
    asynStatus drvUserCreate(asynUser* pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize);

    epicsEventId eventId_;
    time_t lastModified;
    int rowNum;
    bool fileBad; //stops the driver from repeatedly checking bad files
    bool dynamicParameters = false; //set to true to enable dynamic creation of parameters

    bool isModified(const char *checkDir);
    asynStatus readFile(const char *dir);
    void checkLookUp (double newSP, double oldSP);
    int getSPInd (double SP);
    void updateTableValues(int index);
    asynStatus readFileBasedOnParameters();
    bool quietOnSetPoint;

    //Creates asyn parameters and adds them to the parameters list. Optionally links another variable to the created index.
    asynStatus addParameter(const std::string& name, const asynParamType& type, int* index = nullptr);

    //Throws exception if doesnt find a parameter
    Parameter* findParam(const std::string& name);
    Parameter* findParam(const int& index);

    //Resets the lookup table and fills it with current values in the file
    asynStatus populateLookupTable(const std::vector<std::vector<std::string>>& values);

    //Creates missing parameters based on table column headers
    void addTableParameters();

    asynStatus updateParameter(epicsFloat64 value, const std::string& name);

    const LookupTableColumn* findColumnByHeader(const std::string& header) const;
    static std::vector<std::vector<std::string>> splitStreamToColumns(std::istream& stream);
    static std::vector<std::string> splitLine(const std::string& line, char delim);
};
#define ARRAY_PARAMETER_PREFIX "ARR"

#define P_DirString "DIR"
#define P_DirBaseString "DIRBASE"
#define P_IndexString "IND"

#define P_RampingString "CURRMP"
#define P_RampOnString "RMP"
#define P_LookUpOnString "LUT"
#define P_LookUpTableNotDefaultString "LUTNDF"


#define P_TargetString "TGT"
#define P_SPRBVString "TGT:RBV"
#define P_RampRateString "RATE"
#define P_StepsPerMinString "STPNUM"
#define P_CurTempString "CUR"

#define P_SPOutString "SP"

#endif /* READASCII_H */
