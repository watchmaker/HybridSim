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

#include "TagBuffer.h"

using namespace std;

namespace HybridSim {
	
	TagBuffer::TagBuffer()
	{
		//initializing the parallel set structures for the tag cache
		tag_buffer = vector<unodered_map<buffer_line> >(NUM_TAG_SETS, unordered_map<buffer_line>());
	}

	// right now this just steps to keep the clock cycle count accurate for 
	// replacement purposes
	// this will get called from the update in HybridSim
	void TagBuffer::update()
	{		
		step();
	}

	// tags contains the set number corresponding to the group of tags that is being buffered
	void TagBuffer::addTags(uint64_t* tags, bool prefetched)
	{
		// NOTE: to do fully associative, set the number of sets to 1 and the number of ways to whatever size you want
	
		// cycle through the different sets that this is adding tags for
		for(uint64_t tags_index = 0; tags_index < SETS_PER_TRANS; tags_index++)
		{
			uint64_t set_index = tags[tags_index]; // get the set number
			uint64_t tag_buffer_set = set_index % NUM_TAG_SETS;
			
			// if we're out of room, we have to overwrite something
			if(tag_buffer[tag_buffer_set].size() > NUM_TAG_WAYS)
			{
				// all of these replacement algorithms are virtually identical to the general replacement policy algorithms that I already
				// have in place for general cache replacement
				// TODO: move these functions and the other replacement functions to some common object
				std::list<tag_line>::iterator victim = tag_buffer[tag_buffer_set].end();
				// LRU replacement based on time stamps
				if(tagReplacement == lru)
				{
					bool search_init = true;
					uint64_t oldest_ts = 0;					
					// search the tag buffer for the oldest 
					// no need to write back though, they are just tags
					for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
					{
						//first is the key which is the tag address
						//second is the actual tag line structure with the time stamp
						// we do less than here cause older time stamps will be smaller
						// also make sure that we didn't just add this thing in
						if((it->ts < oldest_ts && it->ts != currentClockCycle) || search_init)
						{
							search_init = false; 
							oldest_ts = it->ts;
							victim = it;
						}
					}
					
					// now replace the victim with the new stuff
					victim.set_index = tags[tags_index];
					victim.valid = true;
					victim.used = false;
					victim.prefetched = prefetched;
					victim.ts = currentClockCycle;					
					
				}
				else if(tagReplacement == nru)
				{
					// search the tag buffer for the oldest 
					// no need to write back though, they are just tags
					bool done = false;
					for(std::vector<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
					{
						//first is the key which is the tag address
						//second is the actual tag line structure with the time stamp
						// we do less than here cause older time stamps will be smaller
						if(it->used == true && it->ts != currentClockCycle)
						{
							victim = it;
							done = true;
							break;
						}
					}
					
					if(done == true)
					{
						// now replace the victim with the new stuff
						victim.set_index = tags[tags_index];
						victim.valid = true;
						victim.used = false;
						victim.prefetched = prefetched;
						victim.ts = currentClockCycle;
					}
					// if nothing has been used, just pop from the front
					else
					{
						// making use of a lot of standard library algorithms here to avoid random selection bias
						std::uniform_int_distribution<uint64_t> set_list_dist(0, set_address_list.size()-1);
						std::default_random_engine rand_gen;
						
						// might have to do this multiple times to fine a line that is not one that we just replaced
						bool done = false;
						while(!done)
						{
							advance(victim, set_list_dist(rand_gen));
							if(victim.ts != currentClockCycle)
								done = true;
						}

						// now replace the victim with the new stuff
						victim.set_index = tags[tags_index];
						victim.valid = true;
						victim.used = false;
						victim.prefetched = prefetched;
						victim.ts = currentClockCycle;
					}
				}
				else if(tagReplacement == fifo)
				{
					tag_buffer[tag_buffer_set].pop_front();

					// add a new thing to the buffer
					buffer_line new_line = buffer_line();
					new_line.set_index = set_index;
					new_line.valid = true;
					new_line.used = false;
					new_line.prefetched = prefetched;
					new_line.ts = currentClockCycle;
					
					tag_buffer[tag_buffer_set].push_back(new_line);
				}
				// default random replacement
				else
				{
					// making use of a lot of standard library algorithms here to avoid random selection bias
					std::list<tag_line>::iterator victim = tag_buffer[tag_buffer_set].end();
					std::uniform_int_distribution<uint64_t> set_list_dist(0, set_address_list.size()-1);
					std::default_random_engine rand_gen;
					
					// might have to do this multiple times to fine a line that is not one that we just replaced
					bool done = false;
					while(!done)
					{
						advance(victim, set_list_dist(rand_gen));
						if(victim.ts != currentClockCycle)
							done = true;
					}
					
					// now replace the victim with the new stuff
					victim.set_index = tags[tags_index];
					victim.valid = true;
					victim.used = false;
					victim.prefetched = prefetched;
					victim.ts = currentClockCycle;
				}
			}
			// still filling the buffer, so we need to add new entries
			else
			{
				// make sure we have space to add the thing we're adding now
				assert(tag_buffer[tag_buffer_set].size() < NUM_TAG_WAYS);
				
				// add a new thing to the buffer
				buffer_line new_line = buffer_line();
				new_line.set_index = set_index;
				new_line.valid = true;
				new_line.used = false;
				new_line.prefetched = prefetched;
				new_line.ts = currentClockCycle;
				
				tag_buffer[tag_buffer_set].push_back(new_line);
			}
		}
	}

	bool TagBuffer::haveTags(uint64_t set_index)
	{
		uint64_t tag_buffer_set = 0;
		// fully associative
		if(NUM_TAG_WAYS != 0)
		{										
			tag_buffer_set = set_index % NUM_TAG_SETS;
		}
			
		// search the buffer for this set's tags
		for(std::vector<buffer_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
		{
			if(it.set_index == set_index)
			{	
				// TODO: Not sure if this is going to work, need to check it
				it.used = true;
				return true;
			}
		}
		return false;
	}

} // Namespace HybridSim
