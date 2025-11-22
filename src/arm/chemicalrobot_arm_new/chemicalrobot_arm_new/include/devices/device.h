#ifndef DEVICE_H
#define DEVICE_H

#include <string>
#include <map>
#include <vector>
#include "chemical_common.h"

namespace chemical
{
    class device
    {
    public:
        DeviceStatus _status;
        std::map<std::string, void*> _attribute;
        std::vector<std::string> _script_history;
        std::vector<std::string> _current_scripts;
        std::vector<std::string>::iterator _curr_point, _check_point;
        
        device(){}
        virtual ~device(){}

        void* getAttribute(std::string key)
        {
            return _attribute[key];
        }

        void setAttribute(std::string key, void* value)
        {
            _attribute[key] = value;
        }
        
        
        virtual DeviceErrorCode init(void*) = 0;
        virtual DeviceErrorCode connect() = 0;
        virtual DeviceErrorCode execute_script(std::string script) = 0;
        virtual DeviceErrorCode execute_scripts(std::vector<std::string> scripts) = 0;
        virtual DeviceErrorCode back_to_checkpoint() = 0;
        virtual DeviceErrorCode getdata(std::string dataname,std::string& data) = 0;


    };
}
#endif