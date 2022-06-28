//
// Created by jwscoggins on 6/20/21.
//

#include "EnvironmentInterface.h"
#include <stdlib.h>
namespace cortex
{
    bool
    EnvironmentInterface::unset(const std::string &name) {
        return unsetenv(name.c_str()) == 0;
    }

    std::pair<bool, std::string>
    EnvironmentInterface::get(const std::string &name) {
        char *result = getenv(name.c_str());
        if (result) {
            return std::pair<bool, std::string>(true, result);
        } else {
            return std::pair<bool, std::string>(false, "");
        }
    }

    bool
    EnvironmentInterface::set(const std::string &name, const std::string &value, bool overwrite) {
        return setenv(name.c_str(), value.c_str(), overwrite) == 0;
    }
}