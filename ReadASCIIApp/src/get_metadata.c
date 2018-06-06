#include <stdlib.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <epicsExport.h>

#include "get_metadata.h"

/**
 *  Get the units from the calibration file.
 */
int get_metadata(aSubRecord *prec) 
{
	return get_metadata_impl(prec);	
}

epicsRegisterFunction(get_metadata); /* must also be mentioned in asubFunctions.dbd */
