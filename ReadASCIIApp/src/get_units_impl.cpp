#include <string.h>
#include <stdlib.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <menuFtype.h>
#include <errlog.h>
#include <epicsString.h>
#include <epicsExport.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "get_units.h"


/**
 * Gets the units of a file from that file.
 */
std::string get_units_from_file(std::string filepath) 
{
    std::string units_prefix = "# units:";
    std::string default_units = "K";
    std::string units = default_units;
    
    std::ifstream infile(filepath);
    std::string line;
    while (std::getline(infile, line))
    {
        try {
            // Ideally would use line.starts_with but that's only in C++20
            if (line.substr(0, units_prefix.size()) == units_prefix) {
                units = line.substr(units_prefix.size(), line.size());
            }
        } catch (std::out_of_range) {
            // Out of range error thrown if line is shorter than len(units_prefix)
            // Ignore this case
        }
    }
    return units;
}


/**
 * Extracts a std::string from an epics aSubRecord.
 */
std::string str_from_epics(void* raw_rec)
{
    epicsOldString* rec = reinterpret_cast<epicsOldString*>(raw_rec);
    char buffer[sizeof(epicsOldString)+1];  // +1 for null terminator in the case where epics str is exactly 40 chars (unterminated)
    buffer[sizeof(epicsOldString)] = '\0';
    return std::string(strncpy(buffer, *rec, sizeof(epicsOldString)));
}


/**
 * Extracts data from the aSub record, gets the units of the file and puts the units back into the record.
 */
int get_units_impl(aSubRecord *prec)
{
    std::string base_dir = str_from_epics(prec->a);
    std::string sensor_dir = str_from_epics(prec->b);
    std::string sensor_file = str_from_epics(prec->c);
    
    std::string units = get_units_from_file(base_dir + "/" + sensor_dir + "/" + sensor_file);
    
    strcpy(*reinterpret_cast<epicsOldString*>(prec->vala), units.c_str());
    return 0;
}
