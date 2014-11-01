/*********************************************************************************
* Copyright (c) 2010-2011, 
* Jim Stevens, Paul Tschirhart, Ishwar Singh Bhati, Mu-Tien Chang, Peter Enns, 
* Elliott Cooper-Balis, Paul Rosenfeld, Bruce Jacob
* University of Maryland
* Contact: jims [at] cs [dot] umd [dot] edu
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

#ifndef HYBRIDSYSTEM_UTIL_H
#define HYBRIDSYSTEM_UTIL_H

#include <string>
#include <iostream>
#include <fstream>
#include <list>
#include <sstream>
#include <stdint.h>
#include <cstdlib>
#include <time.h>
#include <random>

using namespace std;

// Utility Library for HybridSim

void convert_uint64_t(uint64_t &var, string value, string infostring = "");
string strip(string input, string chars = " \t\f\v\n\r");
list<string> split(string input, string chars = " \t\f\v\n\r", size_t maxsplit=string::npos);

void confirm_directory_exists(string path);

uint64_t rand64_r(unsigned int *seed);

// Utilities borrowed from DRAMSim2
unsigned inline hybridsim_log2(unsigned value)
{
	unsigned logbase2 = 0;
	unsigned orig = value;
	value>>=1;
	while (value>0)
	{
		value >>= 1;
		logbase2++;
	}
	if (1U<<logbase2 < orig)
		logbase2++;
	return logbase2;
}

bool inline hybridsim_check_power2(unsigned value)
{
	unsigned logbase2 = 0;
	unsigned orig = value;

	value>>=1;
	while (value>0)
	{
		value >>= 1;
		logbase2++;
	}
	if(1U<<logbase2 != orig)
	{
		return false;
	}
	return true;
}

#endif
