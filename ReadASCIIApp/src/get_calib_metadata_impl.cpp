#include <string.h>
#include <stdlib.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <menuFtype.h>
#include <errlog.h>
#include <epicsString.h>
#include <epicsExport.h>
#include <libjson.h>
#include <string>
#include <vector>
#include <deque>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "get_calib_metadata.h"
#include "errlog.h"


/**
 * Gets a property's value by name from JSON.
 */
std::string get_property_value_from_json(const std::string& json, const std::string& property_name) {
    
    std::stringstream ss;
    ss << json;
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt.get<std::string>(property_name);
}


/**
 * Gets the metadata of a file from that file.
 */
std::string get_metadata_from_file(const std::string& filepath, const std::string& property_name, const std::string& property_default) 
{
    std::string comment_prefix = "#";
    std::string file_format = comment_prefix + " ISIS calibration";
    
    std::ifstream infile(filepath);
    
    std::string firstLine;
    if (!std::getline(infile, firstLine) || firstLine.size() < file_format.size() || firstLine.substr(0, file_format.size()) != firstLine) {
        // Not our format - don't parse
        return property_default;
    }
    
    
    std::deque<std::string> comment_lines;
    std::string line;
    while (std::getline(infile, line))
    {
        try {
            // Ideally would use line.starts_with but that's only in C++20
            if (line.substr(0, comment_prefix.size()) == comment_prefix) {
                comment_lines.push_back(line.substr(comment_prefix.size(), line.size()));
            }
        } catch (std::out_of_range) {
            // Out of range error thrown if line is shorter than len(comment_prefix)
            // Ignore this case
        }
    }
    
    if (comment_lines.size() == 0) {
        // Magic bytes existed, but no other lines.
        errlogPrintf("get_calib_metadata: magic bytes existed but no JSON.\n");
        return property_default;
    }
    
    std::string comment_lines_s = "";
    BOOST_FOREACH(std::string value, comment_lines) {
        comment_lines_s += value;
    }
    
    try {
        std::string version_s = get_property_value_from_json(comment_lines_s, "format_version");
        std::string::size_type sz;
        float version_f = std::stof(version_s, &sz);
        if(version_f > 1.0) {
            errlogPrintf("get_calib_metadata: warning: calibration file format is newer than 1.0. Attempting to parse anyway.\n");
        }
        return get_property_value_from_json(comment_lines_s, property_name);
    } catch (const std::exception &e) {
        errlogPrintf("get_calib_metadata: Error parsing JSON: %s\n", e.what());
        return property_default;
    }
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
 * Extracts data from the aSub record, gets the named metadata of the file and puts the metadata value back into the record.
 */
int get_calib_metadata_impl(aSubRecord *prec)
{
    
    bool types_valid = prec->fta == menuFtypeSTRING;
    types_valid = types_valid && prec->ftb == menuFtypeSTRING;
    types_valid = types_valid && prec->ftc == menuFtypeSTRING;
    types_valid = types_valid && prec->ftd == menuFtypeSTRING;
    types_valid = types_valid && prec->fte == menuFtypeSTRING;
    types_valid = types_valid && prec->ftva == menuFtypeSTRING;
    
    if(!types_valid) {
        // Error condition - aSub record didn't have expected types.
        // Return 1 signifies the error to epics
        errlogPrintf("get_calib_metadata: Field types were invalid\n");
        return 1;
    }
    
    std::string base_dir = str_from_epics(prec->a);
    std::string sensor_dir = str_from_epics(prec->b);
    std::string sensor_file = str_from_epics(prec->c);
    std::string property_name = str_from_epics(prec->d);
    std::string property_default = str_from_epics(prec->e);
    
    std::string value = get_metadata_from_file(base_dir + "/" + sensor_dir + "/" + sensor_file, property_name, property_default);
    
    strncpy(*reinterpret_cast<epicsOldString*>(prec->vala), value.c_str(), std::min(sizeof(epicsOldString), value.size()+1));
    return 0;
}
