#include "LScript.h"
#include <iostream>

using namespace std;

namespace ll_script
{

string askPython(string cmd)
{
	string dir = "./code/script/python/";
	string shellCmd = string(string("python ") + dir + cmd).c_str();
	FILE * fi = _popen(shellCmd.c_str(), "r");
	if(!fi)
	{
		cout << "could not execute script " << cmd << endl;
		exit(-1);
	}
	string ret = "";
	while(true)
	{
		char buf[10]; int ar = fread(buf, sizeof(char), 9, fi);
		if(ar <= 0) break;
		buf[ar] = 0x00;
		ret += buf;
	}
	fclose(fi);
	return ret;
}

std::string askPython(std::string cmd, std::string args)
{
	return askPython(cmd + " " + args);
}

}