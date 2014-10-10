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

#ifndef HYBRIDSIM_TAGBUFFER_H
#define HYBRIDSIM_TAGBUFFER_H

#include <iostream>
#include <string>

#include "config.h"
#include "util.h"

using std::string;

namespace HybridSim
{
	class TagBuffer: public SimulatorObject
	{
		// just making it all public for now
	public:
		TagBuffer();
		~TagBuffer();
		
		// just to keep the cycle count valid
		void update();

		void addTags(uint64_t* tags, uint64_t tag_count);

		bool haveTag(uint64_t set_index);
		
		// making this a vector of sets
		// a tag line represents the storage of an entire DRAM buffer set's tags
		// the 
		unordered_map<uint64_t, list<tag_line> > tag_buffer;		
	};
}

#endif
