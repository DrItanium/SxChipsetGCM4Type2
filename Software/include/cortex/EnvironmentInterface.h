//
// Created by jwscoggins on 6/20/21.
//

#ifndef HITAGIMON_ENVIRONMENTINTERFACE_H
#define HITAGIMON_ENVIRONMENTINTERFACE_H
#include <string>
#include <utility>
namespace cortex
{
    class EnvironmentInterface
    {
    public:
        static bool unset(const std::string &name);
        static std::pair<bool, std::string> get(const std::string &name);
        static bool set(const std::string &name, const std::string &value, bool overwrite = false);
    private:
        EnvironmentInterface();
        ~EnvironmentInterface();
        EnvironmentInterface(const EnvironmentInterface &);
        EnvironmentInterface &operator=(const EnvironmentInterface &);
    };
} // end namespace cortex
#endif //HITAGIMON_ENVIRONMENTINTERFACE_H
