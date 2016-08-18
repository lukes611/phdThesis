#pragma once

#include <string>


//programmers can ask python questions using the scripts in the python folder

namespace ll_script
{

std::string askPython(std::string cmd);
std::string askPython(std::string cmd, std::string args);

}