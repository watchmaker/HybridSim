/*********************************************************************************
* Copyright (c) 2010-2011, 
* Paul Tschirhart, Jim Stevens, Ishwar Singh Bhati, Mu-Tien Chang, Peter Enns, 
* Elliott Cooper-Balis, Paul Rosenfeld, Bruce Jacob
* University of Maryland
* Contact: pkt3c [at] umd [dot] edu
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

#ifndef HYBRIDSIM_ADDRESSDECODE_H
#define HYBRIDSIM_ADDRESSDECODE_H

#include <iostream>
#include <string>
#include <stdlib.h>
#include "config.h"
#include "util.h"

using namespace std;

namespace HybridSim
{
	class AddressSet
	{
		public:
			uint64_t channel;
			uint64_t rank;
			uint64_t bank;
			uint64_t row;
			uint64_t column;

			AddressSet() : channel(0), rank(0), bank(0), row(0), column(0) {}
       			string str() 
			{ 
				stringstream out; 
				out << "channel=" << channel << " rank=" << rank << " bank=" << bank << " row=" << row << " column=" << column; 
				return out.str(); 
			}	
	};

	class AddressDecode
	{
		public:
			AddressDecode();

			// the actual decode stuff
			AddressSet getDecode(uint64_t addr);

		private:
			// values used for decoding
			uint64_t channelBitWidth;
			uint64_t rankBitWidth;
			uint64_t bankBitWidth;
			uint64_t rowBitWidth;
			uint64_t colBitWidth;
			uint64_t colOffset;
			uint64_t byteOffset;
			uint64_t colLowBitWidth;
			uint64_t colHighBitWidth;
	};
}

#endif
