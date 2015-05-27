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

		void initializeTagBuffer();

		// just to set up the vector that we use to track the sets that are used		
		void initializeSetTracking();

		// like the set tracking function, this just sets up the vector we use to track access strides
		void initializeStrideTracking();

		// just to keep the cycle count valid
		void update();

		void addTags(vector<uint64_t> tags, bool prefetched, uint64_t demand_set);

		uint64_t haveTags(uint64_t set_index);

		void printBufferUsage();
		void printStrides();
		
		// making this a vector of sets
		// a tag line represents the storage of an entire DRAM buffer set's tags
		// the 
		unordered_map<uint64_t, list<tag_line> > tag_buffer;
		ofstream debug_tag_buffer;
		vector<uint64_t> sets_accessed;
		vector<uint64_t> sets_hit;
		
		class EvictedTagEntry
		{
			public:
				uint64_t set_index;
				uint64_t cycle_evicted;

				EvictedTagEntry()
				{
					set_index = NUM_SETS+1;
					cycle_evicted = 0;
				}	
				EvictedTagEntry(uint64_t si, uint64_t ce)
				{
					set_index = si;
					cycle_evicted = ce;
				}
		};

		list<EvictedTagEntry> victim_tag_list;
		unordered_map<uint64_t, uint64_t> victim_tag_requested;  // victim address - time from eviction
		unordered_map<uint64_t, uint64_t> victim_tag_histogram;
		
		// trying to capture the distribution of access distances	
		ofstream record_strides;
		uint64_t last_set_accessed;
		vector<uint64_t> access_stride_histogram;
		// trying to capture set reuse
		//unordered_map<uint64_t, uint64_t> last_access;
		//unordered_map<uint64_t, uint64_t> reuse_histogram; // reuse distance
		//uint64_t reuse_average;	
		//uint64_t num_sets_accessed;

		uint64_t num_tags_buffered;
		uint64_t num_tags_used;
	};
}

#endif
