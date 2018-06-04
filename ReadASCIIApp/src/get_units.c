#include <stdlib.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <epicsExport.h>

#include "get_units.h"

/**
 *  Get the units from the calibration file.
 */
int get_units(aSubRecord *prec) 
{
	return get_units_impl(prec);	
}

epicsRegisterFunction(get_units); /* must also be mentioned in asubFunctions.dbd */
