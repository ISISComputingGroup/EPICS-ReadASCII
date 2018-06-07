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

#include "get_metadata.h"


std::string get_property_value_from_json(std::string json, std::string property_name) {
    
    std::stringstream ss;
    ss << json;
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt.get<std::string>(property_name);
}


/**
 * Gets the metadata of a file from that file.
 */
std::string get_metadata_from_file(std::string filepath, std::string property_name, std::string property_default) 
{
    std::string comment_prefix = "#";
    std::string file_format = "ISIS calibration";
    
    std::deque<std::string> all_lines;
    
    std::ifstream infile(filepath);
    std::string line;
    
    while (std::getline(infile, line))
    {
        try {
            // Ideally would use line.starts_with but that's only in C++20
            if (line.substr(0, comment_prefix.size()) == comment_prefix) {
                all_lines.push_back(line.substr(comment_prefix.size(), line.size()));
            }
        } catch (std::out_of_range) {
            // Out of range error thrown if line is shorter than len(comment_prefix)
            // Ignore this case
        }
    }
    
    if (all_lines.size() == 0) {
        // No commented block in file.
        return property_default;
    }
    
    std::string header_line = all_lines.front();
    all_lines.pop_front();
    
    if (header_line.find(file_format) == std::string::npos) {
        // Header didn't contain our expected magic bytes - refuse to parse.
        return property_default;
    }
    
    if (all_lines.size() == 0) {
        // Commented block and magic bytes existed, but no other lines.
        return property_default;
    }
    
    std::string all_lines_s = "";
    for (auto const& value: all_lines) {
        all_lines_s += value;
    }
    
    try {
        return get_property_value_from_json(all_lines_s, property_name);
    } catch (std::exception &e) {
        std::cout "Error parsing JSON: " << e.what() << std::endl;
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
int get_metadata_impl(aSubRecord *prec)
{
    std::string base_dir = str_from_epics(prec->a);
    std::string sensor_dir = str_from_epics(prec->b);
    std::string sensor_file = str_from_epics(prec->c);
    std::string property_name = str_from_epics(prec->d);
    std::string property_default = str_from_epics(prec->e);
    
    std::string value = get_metadata_from_file(base_dir + "/" + sensor_dir + "/" + sensor_file, property_name, property_default);
    
    strcpy(*reinterpret_cast<epicsOldString*>(prec->vala), value.c_str());
    return 0;
}
