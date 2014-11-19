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
		if (DEBUG_COMBO_TAG)
		{
			debug_tag_buffer.open("tag_buffer.log", ios_base::out | ios_base::trunc);
			if (!debug_tag_buffer.is_open())
			{
				cerr << "ERROR: HybridSim debug_tag_buffer file failed to open.\n";
				abort();
			}
		}
	}

	void TagBuffer::initializeTagBuffer()
	{
		//initializing the parallel set structures for the tag cache
		//tag_buffer = unordered_map<uint64_t, list<tag_line> >(NUM_TAG_SETS, list<tag_line>());
	}

	void TagBuffer::initializeSetTracking()
	{
		sets_accessed = vector<uint64_t>(NUM_TAG_SETS, 0);
	}

	// right now this just steps to keep the clock cycle count accurate for 
	// replacement purposes
	// this will get called from the update in HybridSim
	void TagBuffer::update()
	{		
		step();
	}

	// tags contains the set number corresponding to the group of tags that is being buffered
	void TagBuffer::addTags(vector<uint64_t> tags, bool prefetched)
	{
		// NOTE: to do fully associative, set the number of sets to 1 and the number of ways to whatever size you want
		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "ADDING TAGS";
			debug_tag_buffer << "-----------------\n";
		}

		//cout << "\nadding tags \n==========\n";
		// cycle through the different sets that this is adding tags for
		for(uint64_t tags_index = 0; tags_index < tags.size(); tags_index++)
		{
			uint64_t set_index = tags[tags_index]; // get the set number
			uint64_t tag_buffer_set;
						
			//cout << "set index " << set_index << "\n";

			if(ENABLE_SET_CHANNEL_INTERLEAVE)
			{
				tag_buffer_set = (set_index / NUM_CHANNELS) % NUM_TAG_SETS;
			}
			else
			{
				tag_buffer_set = (set_index) % NUM_TAG_SETS;
			}
			
			if(DEBUG_COMBO_TAG)
			{
				debug_tag_buffer << "added tag for set index " << set_index << "\n";
				debug_tag_buffer << "this mapped to tag buffer set " << tag_buffer_set << "\n";
				sets_accessed[tag_buffer_set] = sets_accessed[tag_buffer_set] + 1;
			}

			// if we have an entry for this tag set
			if(tag_buffer.find(tag_buffer_set) != tag_buffer.end())
			{		
				// if we're out of room, we have to overwrite something
				if(tag_buffer[tag_buffer_set].size() >= NUM_TAG_WAYS)
				{
					// all of these replacement algorithms are virtually identical to the general replacement policy algorithms that I already
					// have in place for general cache replacement
					// TODO: move these functions and the other replacement functions to some common object
					std::list<tag_line>::iterator victim = tag_buffer[tag_buffer_set].begin();
					// LRU replacement based on time stamps
					// Reuse the same method with the nnn replacement strategy
					// NNN replacement differs in how adjacent tags also have their time stamps updated on a hit
					if(tagReplacement == tag_lru || tagReplacement == tag_nnn)
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
						
						if(DEBUG_COMBO_TAG)
						{
							debug_tag_buffer << "selected victim tag with set index " << (*victim).set_index << "\n";
						}
						
						// now replace the victim with the new stuff
						(*victim).set_index = tags[tags_index];
						(*victim).valid = true;
						(*victim).used = false;
						(*victim).prefetched = prefetched;
						(*victim).ts = currentClockCycle;					
						
					}
					// uses a fifo like heuristic to evict things that haven't been used, this should give plenty of time for things to be used
					// the queue acts like a timer, but the used tags should be spared for a while
					else if(tagReplacement == tag_lrnu)
					{
						// loop through the set looking for something to evict
						bool done = false;
						while(!done)
						{
							// we always pop
							tag_line cur_line = tag_buffer[tag_buffer_set].front();
							tag_buffer[tag_buffer_set].pop_front();

							// if the tag has been used, mark it as unused and push it back onto the list
							if(cur_line.used == true)
							{
								cur_line.used = false;
								tag_buffer[tag_buffer_set].push_back(cur_line);
								continue;
							}
						
							// we found something that wasn't used so just don't add it back
							// add a new thing to the buffer instead
							tag_line new_line = tag_line();
							new_line.set_index = set_index;
							new_line.valid = true;
							new_line.used = false;
							new_line.prefetched = prefetched;
							new_line.ts = currentClockCycle;
							
							tag_buffer[tag_buffer_set].push_back(new_line);
							
							// we're done here
							done = true;

							// a possible variation on this might be to use random replacement if everything had been used instead of just
							// evicting the oldest thing (that might actually be the most valuable)
						}
					}
					// recently used, assumes that things that have been used won't be used again and kicks them out
					else if(tagReplacement == tag_ru)
					{
						// search the tag buffer for anything that has been used to evict
						// no need to write back though, they are just tags
						bool done = false;
						for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
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
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
						}
						// if nothing has been used, just pop from a random location
						else
						{					      
							if(tag_buffer[tag_buffer_set].size() > 1)
							{
								// making use of a lot of standard library algorithms here to avoid random selection bias
								std::uniform_int_distribution<uint64_t> set_list_dist(0, tag_buffer[tag_buffer_set].size()-1);
								std::default_random_engine rand_gen;
								
								// might have to do this multiple times to fine a line that is not one that we just replaced
								bool done = false;
								while(!done)
								{
									advance(victim, set_list_dist(rand_gen));
									if((*victim).ts != currentClockCycle)
										done = true;
								}
							}
							
							// if the set was only 1 entry then we're just replacing the first entry (the only one)
							// victim was set to point to the beginning at the beginning of this
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
						}
					}
					else if(tagReplacement == tag_fifo)
					{
						if(DEBUG_COMBO_TAG)
						{
							debug_tag_buffer << "selected victim tag with set index " << tag_buffer[tag_buffer_set].front().set_index << "\n";
						}
						
						tag_buffer[tag_buffer_set].pop_front();
						
						// add a new thing to the buffer
						tag_line new_line = tag_line();
						new_line.set_index = set_index;
						new_line.valid = true;
						new_line.used = false;
						new_line.prefetched = prefetched;
						new_line.ts = currentClockCycle;
						
						tag_buffer[tag_buffer_set].push_back(new_line);
					}
					else if(tagReplacement == tag_mru)
					{
						// search the tag buffer for the most recently used thing to evict
						// no need to write back though, they are just tags
						bool search_init = true;
						uint64_t newest_ts = 0;	
						for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
						{
							//first is the key which is the tag address
							//second is the actual tag line structure with the time stamp
							// we do less than here cause older time stamps will be smaller
							if((it->ts > newest_ts && it->ts != currentClockCycle && it->used == true) || search_init)
							{
								search_init = false; 
								newest_ts = it->ts;
								victim = it;
							}
						}
						
						if(search_init == false)
						{
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
						}
						// if nothing has been used, just pop from a random location
						else
						{

							if(tag_buffer[tag_buffer_set].size() > 1)
							{
								// making use of a lot of standard library algorithms here to avoid random selection bias
								std::uniform_int_distribution<uint64_t> set_list_dist(0, tag_buffer[tag_buffer_set].size()-1);
								std::default_random_engine rand_gen;
							
								// might have to do this multiple times to fine a line that is not one that we just replaced
								bool done = false;
								while(!done)
								{
									advance(victim, set_list_dist(rand_gen));
									if((*victim).ts != currentClockCycle)
										done = true;
								}
							}
							
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
						}
					}
					// default random replacement
					else
					{
						if(tag_buffer[tag_buffer_set].size() > 1)
						{
							// making use of a lot of standard library algorithms here to avoid random selection bias
							std::list<tag_line>::iterator victim = tag_buffer[tag_buffer_set].end();
							std::uniform_int_distribution<uint64_t> set_list_dist(0, tag_buffer[tag_buffer_set].size()-1);
							std::default_random_engine rand_gen;
							
							// might have to do this multiple times to fine a line that is not one that we just replaced
							bool done = false;
							while(!done)
							{
								advance(victim, set_list_dist(rand_gen));
								if((*victim).ts != currentClockCycle)
									done = true;
							}
						}
						
						// now replace the victim with the new stuff
						(*victim).set_index = tags[tags_index];
						(*victim).valid = true;
						(*victim).used = false;
						(*victim).prefetched = prefetched;
						(*victim).ts = currentClockCycle;
					}
				}
		       
				// still filling the buffer, so we need to add new entries
				else
				{
					// make sure we have space to add the thing we're adding now
					assert(tag_buffer[tag_buffer_set].size() < NUM_TAG_WAYS);
					
					// add a new thing to the buffer
					tag_line new_line = tag_line();
					new_line.set_index = set_index;
					new_line.valid = true;
					new_line.used = false;
					new_line.prefetched = prefetched;
					new_line.ts = currentClockCycle;
					
					tag_buffer[tag_buffer_set].push_back(new_line);
				}
			}
			// need to add a tag buffer map entry for this set
			else
			{
				// add a new thing to the buffer
				tag_line new_line = tag_line();
				new_line.set_index = set_index;
				new_line.valid = true;
				new_line.used = false;
				new_line.prefetched = prefetched;
				new_line.ts = currentClockCycle;
				
				tag_buffer[tag_buffer_set] = list<tag_line>();
				assert(tag_buffer.count(tag_buffer_set) == 1);
				tag_buffer[tag_buffer_set].push_back(new_line);
			}
		}

		if(DEBUG_COMBO_TAG)
		{
 			debug_tag_buffer << "=================\n\n";
		}
	}

	// being lazy here and returning a code so we can know if this was a prefetch hit or not
	// this is just so I don't have to pass the logger over to this other thing
	// return codes:
	// 0 : miss
	// 1 : hit
	// 2 : prefetch hit
	uint64_t TagBuffer::haveTags(uint64_t set_index)
	{
		uint64_t tag_buffer_set = 0;
		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "CHECKING FOR TAGS";
			debug_tag_buffer << "-----------------\n";
		}
		
		if(ENABLE_SET_CHANNEL_INTERLEAVE)
		{
			tag_buffer_set = (set_index / NUM_CHANNELS) % NUM_TAG_SETS;
		}
		else
		{
			tag_buffer_set = (set_index) % NUM_TAG_SETS;
		}
		
		//cout << "\nlooking for tag " << set_index << "\n";

		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "looking for tag with set index " << set_index << "\n";
			debug_tag_buffer << "this mapped to tag buffer set " << tag_buffer_set << "\n";
		}
			
		// search the buffer for this set's tags
		for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
		{
			if((*it).set_index == set_index)
			{	
				if(DEBUG_COMBO_TAG)
				{
					debug_tag_buffer << "got a HIT!!! \n";
					debug_tag_buffer << "================\n\n";
				}

				// TODO: Not sure if this is going to work, need to check it
				(*it).used = true;
				// update the timestamp (not sure if this is the right way to go about this)
				//(*it).ts = currentClockCycle;
				// set the return value before we move the iterator
				uint64_t return_value = 0;
				if((*it).prefetched == true)
					return_value = 2;
				else
					return_value = 1;
				// if we hit this guy then its likely we're going to hit the next few as well because they should be
				// sequentially just after the one we hit
				if(tagReplacement == tag_nnn)
				{
					// just do the immediate neighbors, not everything, if the neighbor is hit soon
					// it'll update his neighbor and so on.
					uint64_t num_to_update = 1;
					/*if(ENABLE_TAG_PREFETCH && TAG_PREFETCH_WINDOW == 0)
					{
						// the most tags we could have is a whole rows worth
						num_to_update = 6;
					}
					// if we're prefetching more than just to the end of the row
					// then however much we're prefetching could be useful
					else if(ENABLE_TAG_PREFETCH)
					{
						num_to_update = TAG_PREFETCH_WINDOW-1;
					}
					// the most other tags we can get without prefetching
					// there would definitely be sequential would be 2
					else
					{
						num_to_update = 2;
						}*/
					
					std::list<tag_line>::iterator temp = it;
					// update the time stamps of the forward neighbors to preserve them
					for(uint64_t i = 0; i < num_to_update; i++)
					{
						// step the iterator forward to the next sequential tag
						it++;
						// make sure we don't run off the end of the world
						if(it == tag_buffer[tag_buffer_set].end())
						{
							break;
						}
						
						// set the tags time stamp to slightly later than the last tag
						// this should place prefence on the tags that immediately follow
						// the tag that was hit
						(*it).ts = currentClockCycle + i;
					}

					// update going backwards too
					for(uint64_t i = 0; i < num_to_update; i++)
					{
						// make sure we don't run off the end of the world
						if(temp == tag_buffer[tag_buffer_set].begin())
						{
							break;
						}	
						temp--;
						(*temp).ts = currentClockCycle + i;
					}
				}
				
				return return_value;
			}
		}
		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "big ole MISS... \n";
			debug_tag_buffer << "================\n\n";
		}
		return 0;
	}
	
	void TagBuffer::printBufferUsage()
	{
		debug_tag_buffer << "********************************************\n";
		debug_tag_buffer << ">>>>>>> Final Tag Set Usage Counts <<<<<<<<\n";
		for(uint64_t j = 0; j < NUM_TAG_SETS; j++)
		{
			debug_tag_buffer << "Set " << j << " : " << sets_accessed[j] << "\n";
		}
		debug_tag_buffer.flush();
		debug_tag_buffer.close();
	}

} // Namespace HybridSim
