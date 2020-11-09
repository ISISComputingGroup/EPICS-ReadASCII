#ifndef READASCII_H
#define READASCII_H

/// @file ReadASCII.h Header for read ASCII driver

#include "asynPortDriver.h"
#include <time.h>

#define DIR_LENGTH 260

/// Driver for Ramping SPs and Reading PIDs
class ReadASCII : public asynPortDriver 
{
public:
    ReadASCII(const char* portName, const char *searchDir, const int stepsPerMinute);


    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

    void readFilePoll(void);
    void rampThread(void);

protected:
    int P_Dir; // string
    int P_DirBase; // string
    int P_Index; //int

    int P_SPArr; //float Array
    int P_PArr; //float Array
    int P_IArr; //float Array
    int P_DArr; //float Array
    int P_MaxHeatArr; //float Array

    int P_Ramping; //int
    int P_RampOn; //int
    int P_LookUpOn; //int

    int P_RampRate; //float
    int P_StepsPerMin; //float
    int P_CurTemp; //float
    int P_Target; //float
    int P_SPOut; //float
    int P_SPRBV; //float

    int P_P; //float
    int P_I; //float
    int P_D; //float
    int P_MaxHeat; //float

private:
    epicsFloat64 *pSP_;
    epicsFloat64 *pP_;
    epicsFloat64 *pI_;
    epicsFloat64 *pD_;
    epicsFloat64 *pMaxHeat_;

    epicsEventId eventId_;
    time_t lastModified;
    char lastDir[DIR_LENGTH];
    int rowNum;
    bool fileBad; //stops the driver from repeatedly checking bad files

    bool isModified(const char *checkDir);
    asynStatus readFile(const char *dir);
    void checkLookUp (double newSP, double oldSP);
    int getSPInd (double SP);
    void updatePID(int index);
    asynStatus readFileBasedOnParameters();
    
#define FIRST_READASCII_PARAM P_Dir
#define LAST_READASCII_PARAM P_MaxHeat
};

#define NUM_READASCII_PARAMS (&LAST_READASCII_PARAM - &FIRST_READASCII_PARAM + 1)
 
#define P_DirString "DIR"
#define P_DirBaseString "DIRBASE"
#define P_IndexString "IND"

#define P_SPArrString "ARRSP"
#define P_PArrString "ARRP"
#define P_IArrString "ARRI"
#define P_DArrString "ARRD"
#define P_MaxHeatArrString "ARRMH"

#define P_RampingString "CURRMP"
#define P_RampOnString "RMP"
#define P_LookUpOnString "LUT"

#define P_TargetString "TGT"
#define P_SPRBVString "TGT:RBV"
#define P_RampRateString "RATE"
#define P_StepsPerMinString "STPNUM"
#define P_CurTempString "CUR"

#define P_SPOutString "SP"
#define P_PString "P"
#define P_IString "I"
#define P_DString "D"
#define P_MaxHeatString "MH"

#endif /* READASCII_H */
