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
	
	}

	void TagBuffer::initializeTagBuffer()
	{
		//initializing the parallel set structures for the tag cache
		//tag_buffer = unordered_map<uint64_t, list<tag_line> >(NUM_TAG_SETS, list<tag_line>());

		// setting up the bloom filter, if we need it
		if(ENABLE_BLOOM)
		{
			period_count = 0;
			offset_rating = 0;
			under_review = 0;
			// this should automatically set the offset vector to be the size of whatever amount we're prefetching
			offset_enable = vector<bool>((SETS_PER_TAG_GROUP+1) + (0.5*TAG_PREFETCH_WINDOW*SETS_PER_TAG_GROUP) + (0.5*TAG_PREFETCH_WINDOW*(SETS_PER_TAG_GROUP+1)), false);

			// How many elements roughly do we expect to insert?
			parameters.projected_element_count = EVAL_MAX;

			// Maximum tolerable false positive probability? (0,1)
			parameters.false_positive_probability = 0.01; // 1 in 10000

			// Simple randomizer (optional)
			parameters.random_seed = 0xA5A5A5A5;

			if (!parameters)
			{
				std::cout << "Error - Invalid set of bloom filter parameters!" << std::endl;
				abort();
			}

			parameters.compute_optimal_parameters();

			//Instantiate Bloom Filter
			offset_bloom = bloom_filter(parameters);
		}
	}

	void TagBuffer::initializeSetTracking()
	{
		if (DEBUG_COMBO_TAG || ENABLE_TAG_BUFFER_USAGE_LOG)
		{
			debug_tag_buffer.open("tag_buffer.log", ios_base::out | ios_base::trunc);
			if (!debug_tag_buffer.is_open())
			{
				cerr << "ERROR: HybridSim debug_tag_buffer file failed to open.\n";
				abort();
			}
		}

		sets_accessed = vector<uint64_t>(NUM_TAG_SETS, 0);
		sets_hit = vector<uint64_t>(NUM_TAG_SETS, 0);

		// this uses up memory and didn't tell us anything interesting now, but it might in the future
		// Init the reuse histogram
		/*for (uint64_t i = 0; i <= 200; i += 20)
		{
			reuse_histogram[i] = 0;
		}
		reuse_average = 0;
		num_sets_accessed = 0;*/

		// track how many tags we bring in and how many are used
		num_tags_buffered = 0;
		num_tags_used = 0;
	}

	void TagBuffer::initializeStrideTracking()
	{
		if(ENABLE_STRIDE_LOG)
		{
			record_strides.open("stride.log", ios_base::out | ios_base::trunc);
			if (!record_strides.is_open())
			{
				cerr << "ERROR: HybridSim debug_tag_buffer file failed to open.\n";
				abort();
			}	
		}

		last_sets_accessed = vector<uint64_t> (8, NUM_SETS+1);
		access_stride_histogram = vector<vector<uint64_t> >(8, vector<uint64_t>(NUM_SETS, 0));
	}

	// right now this just steps to keep the clock cycle count accurate for 
	// replacement purposes
	// this will get called from the update in HybridSim
	void TagBuffer::update()
	{		
		step();
	}

	// tags contains the set number corresponding to the group of tags that is being buffered
	void TagBuffer::addTags(vector<uint64_t> tags, bool prefetched, uint64_t demand_set, uint64_t tag_set_offset)
	{
		// NOTE: to do fully associative, set the number of sets to 1 and the number of ways to whatever size you want
		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "ADDING TAGS";
			debug_tag_buffer << "-----------------\n";
		}


		if(ENABLE_TAG_BUFFER_USAGE_LOG)
		{
			num_tags_buffered += tags.size();
		}

		//cout << "\nadding tags \n==========\n";
		uint64_t offset = tag_set_offset;
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

			if(DEBUG_TAG_BUFFER || ENABLE_TAG_BUFFER_USAGE_LOG)
			{
				sets_accessed[tag_buffer_set] = sets_accessed[tag_buffer_set] + 1;
			}
			
			// are we even worried about adding this tag?
			// are we even worried about adding this tag?
			bool add_tag = true;
			if(ENABLE_BLOOM)
			{
				add_tag = offset_enable[offset];
			}
			if(add_tag)
			{
				
				// if we have an entry for this tag set
				if(tag_buffer.find(tag_buffer_set) != tag_buffer.end())
				{	
					// first see if we've already got this tag around and we're just updating it
					bool updated = false;
					for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
					{
						if((*it).set_index == set_index)
						{
							updated = true;
							(*it).ts = currentClockCycle;
							break;
						}
					}
					// if we updated this tag then move on to the next one.
					if(updated)
					{
						continue;
					}
					// if we're out of room, we have to overwrite something
					else if(tag_buffer[tag_buffer_set].size() >= NUM_TAG_WAYS)
					{
						// all of these replacement algorithms are virtually identical to the general replacement policy algorithms that I already
						// have in place for general cache replacement
						// TODO: move these functions and the other replacement functions to some common object
						std::list<tag_line>::iterator victim = tag_buffer[tag_buffer_set].begin();
						// UF_LRA Replacement (Used First - Least Recently Added)
						// NNN Replacement (Not Nearest Neighbors) - extension on UF_LRA to protect adjacent sets from eviction as well
						// sort've like LRU replacement 
						if(tagReplacement == tag_uflra || tagReplacement == tag_nnn)
						{
							bool search_init = true;
							bool found_used = false;
							uint64_t oldest_ts = 0;		
							// search the tag buffer for the oldest 
							// no need to write back though, they are just tags
							for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
							{
								//first is the key which is the tag address
								//second is the actual tag line structure with the time stamp
								// we do less than here cause older time stamps will be smaller
								// also make sure that we didn't just add this thing in
								if(search_init)
								{
									search_init = false; 
									oldest_ts = it->ts;
									victim = it;
									if(it->used == true)
										found_used = true;
								}
								else
								{
									if(found_used == false)
									{
										if(it->used == true)
										{
											oldest_ts = it->ts;
											found_used = true;
											victim = it;
										}
										else if(it->ts < oldest_ts && it->ts != currentClockCycle)
										{
											oldest_ts = it->ts;
											victim = it;
										}
									}
									else if(it->used == true)								     
									{
										if(it->ts < oldest_ts && it->ts != currentClockCycle)
										{
											oldest_ts = it->ts;
											victim = it;				
										}
									}
									
								}
								/*if((it->ts < oldest_used && it->ts != currentClockCycle && it->used) || (victim->used == false && it->used == true))
								  {
								  oldest_used = it->ts;
								  victim = it;
								  found_used = true;
								  }*/
							}
							
							if(DEBUG_COMBO_TAG)
							{
								debug_tag_buffer << "selected victim tag with set index " << (*victim).set_index << "\n";
							}

							// store the victim
							if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
							{
								victim_tag_list.pop_front();
							}
							victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
							
							
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
							if(tags[tags_index] == demand_set)
							{
								(*victim).demand = true;
							}
							else
							{
								(*victim).demand = false;
							}
							
						}
						if(tagReplacement == tag_lru)
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
								if(search_init)
								{
									search_init = false; 
									oldest_ts = it->ts;
									victim = it;
								}
								else if(it->ts < oldest_ts && it->ts != currentClockCycle)
								{
									oldest_ts = it->ts;
									victim = it;						       
								}
							}
							
							if(DEBUG_COMBO_TAG)
							{
								debug_tag_buffer << "selected victim tag with set index " << (*victim).set_index << "\n";
							}
							
							// store the victim
							if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
							{
								victim_tag_list.pop_front();
							}
							victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
							
							
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
							if(tags[tags_index] == demand_set)
							{
								(*victim).demand = true;
							}
							else
							{
								(*victim).demand = false;
							}
							
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
								
								// store the victim
								if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
								{
									victim_tag_list.pop_front();
								}
								victim_tag_list.push_back(EvictedTagEntry(cur_line.set_index, currentClockCycle));
								
								// we found something that wasn't used so just don't add it back
								// add a new thing to the buffer instead
								tag_line new_line = tag_line();
								new_line.set_index = set_index;
								new_line.valid = true;
								new_line.used = false;
								new_line.prefetched = prefetched;
								new_line.ts = currentClockCycle;
								if(tags[tags_index] == demand_set)
								{
									new_line.demand = true;
								}
								else
								{
									new_line.demand = false;
								}
								
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
								// store the victim
								if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
								{
									victim_tag_list.pop_front();
								}
								victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
								
								// now replace the victim with the new stuff
								(*victim).set_index = tags[tags_index];
								(*victim).valid = true;
								(*victim).used = false;
								(*victim).prefetched = prefetched;
								(*victim).ts = currentClockCycle;
								if(tags[tags_index] == demand_set)
								{
									(*victim).demand = true;
								}
								else
								{
									(*victim).demand = false;
								}
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
								
								// store the victim
								if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
								{
									victim_tag_list.pop_front();
								}
								victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
								
								// if the set was only 1 entry then we're just replacing the first entry (the only one)
								// victim was set to point to the beginning at the beginning of this
								// now replace the victim with the new stuff
								(*victim).set_index = tags[tags_index];
								(*victim).valid = true;
								(*victim).used = false;
								(*victim).prefetched = prefetched;
								(*victim).ts = currentClockCycle;
								if(tags[tags_index] == demand_set)
								{
									(*victim).demand = true;
								}
								else
								{
									(*victim).demand = false;
								}
							}
						}
						else if(tagReplacement == tag_fifo)
						{
							if(DEBUG_COMBO_TAG)
							{
								debug_tag_buffer << "selected victim tag with set index " << tag_buffer[tag_buffer_set].front().set_index << "\n";
							}
							
							// store the victim
							if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
							{
								victim_tag_list.pop_front();
							}
							victim_tag_list.push_back(EvictedTagEntry(tag_buffer[tag_buffer_set].front().set_index, currentClockCycle));
							
							tag_buffer[tag_buffer_set].pop_front();
							
							// add a new thing to the buffer
							tag_line new_line = tag_line();
							new_line.set_index = set_index;
							new_line.valid = true;
							new_line.used = false;
							new_line.prefetched = prefetched;
							new_line.ts = currentClockCycle;
							if(tags[tags_index] == demand_set)
							{
								new_line.demand = true;
							}
							else
							{
								new_line.demand = false;
							}
						
							tag_buffer[tag_buffer_set].push_back(new_line);
						}
						else if(tagReplacement == tag_mru)
						{
							// search the tag buffer for the most recently used thing to evict
							// no need to write back though, they are just tags
							bool search_init = true;
							bool found_used = false;
							uint64_t newest_ts = 0;	
							for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set].begin(); it != tag_buffer[tag_buffer_set].end(); it++)
							{
								// this is the opposite of LRU
								// basically we're looking for the youngest thing to evict instead of the oldest
								// with a preference for used things to be evicted first
								if(search_init)
								{
									search_init = false; 
									newest_ts = it->ts;
									victim = it;
									if(it->used == true)
										found_used = true;
								}
								else
								{
									if(found_used == false)
									{
										if(it->used == true)
										{
											newest_ts = it->ts;
											found_used = true;
											victim = it;
										}
										else if(it->ts > newest_ts && it->ts != currentClockCycle)
										{
											newest_ts = it->ts;
											victim = it;
										}
									}
									else if(it->used == true)								     
									{
										if(it->ts > newest_ts && it->ts != currentClockCycle)
										{
											newest_ts = it->ts;
											victim = it;				
										}
									}
									
								}
							}
							if(DEBUG_COMBO_TAG)
							{
								debug_tag_buffer << "selected victim tag with set index " << (*victim).set_index << "\n";
							}
							
							// store the victim
							if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
							{
								victim_tag_list.pop_front();
							}
							victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
							
							
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
							if(tags[tags_index] == demand_set)
							{
								(*victim).demand = true;
							}
							else
							{
								(*victim).demand = false;
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
							
							// store the victim
							if(victim_tag_list.size() > VICTIM_LIST_LENGTH)
							{
								victim_tag_list.pop_front();
							}
							victim_tag_list.push_back(EvictedTagEntry((*victim).set_index, currentClockCycle));
							
							// now replace the victim with the new stuff
							(*victim).set_index = tags[tags_index];
							(*victim).valid = true;
							(*victim).used = false;
							(*victim).prefetched = prefetched;
							(*victim).ts = currentClockCycle;
							if(tags[tags_index] == demand_set)
							{
								(*victim).demand = true;
							}
							else
							{
								(*victim).demand = false;
							}
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
						if(tags[tags_index] == demand_set)
						{
							new_line.demand = true;
						}
						else
						{
							new_line.demand = false;
						}
						
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
					if(tags[tags_index] == demand_set)
					{
						new_line.demand = true;
					}
					else
					{
						new_line.demand = false;
					}
					
					tag_buffer[tag_buffer_set] = list<tag_line>();
					assert(tag_buffer.count(tag_buffer_set) == 1);
					tag_buffer[tag_buffer_set].push_back(new_line);
				}
			}
			offset++;
		}
		if(DEBUG_COMBO_TAG)
		{
 			debug_tag_buffer << "=================\n\n";
		}
	}

	void TagBuffer::updateBloom(uint64_t set_index)
	{
		// increment the counter for this evaluation period
		period_count++;
		if(period_count > EVAL_MAX)
		{
			period_count = 0;
			if( offset_rating > RATE_THRESH)
			{
				offset_enable[under_review] = true;
			}
			else
			{
				offset_enable[under_review] = false;
			}
			offset_bloom.clear();
			offset_bloom = bloom_filter(parameters);
			offset_rating = 0;
			under_review++;
			if(under_review >= (SETS_PER_TAG_GROUP+1) + (0.5*TAG_PREFETCH_WINDOW*SETS_PER_TAG_GROUP) + (0.5*TAG_PREFETCH_WINDOW*(SETS_PER_TAG_GROUP+1)))
			{
				under_review = 0;
			}
		}

		// update the bloom filter history
		if(offset_bloom.contains(set_index))
		{
			offset_rating++;
		}

		// update the bloom filter with next access at the current offset
		uint64_t set_index_mod = set_index % SETS_PER_LINE;
		uint64_t set_group_pos = 0;
		if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
		{
			set_group_pos = (set_index_mod) % (SETS_PER_TAG_GROUP + 1);
		}
		else
		{
			set_group_pos = (set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % (SETS_PER_TAG_GROUP);
		}

		// gives us the first set index of the tag group, we base the offsets off of this always
		uint64_t set_index_align = set_index - set_group_pos;

		offset_bloom.insert(set_index_align + under_review);
	}

	bool TagBuffer::offsetEnabled(uint64_t num_tags, uint64_t offset_start)
	{
		uint64_t offset = offset_start;
		// check all of the offsets to see if we're using any of them
		for(uint64_t tags_index = 0; tags_index < num_tags; tags_index++)
		{
			// if any of the offsets are being used, then return true
			if(offset_enable[offset])
			{
				return true;
			}
			offset++;
		}
		// otherwise we're not using any offsets, so retrun false
		return false;
	}

	// being lazy here and returning a code so we can know if this was a prefetch hit or not
	// this is just so I don't have to pass the logger over to this other thing
	// return codes:
	// 0 : miss
	// 1 : hit (demand set)
	// 2 : hit (free set)
	// 3 : prefetch hit
	uint64_t TagBuffer::haveTags(uint64_t set_index)
	{
		// if we're using a bloom filter, update it with this set_index
		if(ENABLE_BLOOM)
		{
			updateBloom(set_index);
		}

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

		if(ENABLE_STRIDE_LOG)
		{
			for(int s = 7; s >= 0; s--)
			{
				if(last_sets_accessed[s] != NUM_SETS+1)
				{
					uint64_t stride_length;
					if(set_index >= last_sets_accessed[s])
						stride_length = set_index - last_sets_accessed[s];
					else
						stride_length = last_sets_accessed[s] - set_index;
					uint64_t temp = access_stride_histogram[s][stride_length];
					access_stride_histogram[s][stride_length] = temp + 1;
				}
				if(s == 0)
				{
					// the old entry for this last set has already been copied
					// so we can just overwrite it
					last_sets_accessed[s] = set_index;
				}
				else
				{
					// overwrite the set_index that we just compared against with the
					// next one up in the queue
					// this should wind up making this thing look like a FIFO
					last_sets_accessed[s] = last_sets_accessed[s-1];
				}
			}
		}

		/*if(ENABLE_TAG_BUFFER_USAGE_LOG)
		{
			// see if this set has been accessed before	    		    
			if(last_access.find(set_index) != last_access.end())
			{
				uint64_t reuse_distance = currentClockCycle - last_access[set_index];
				 // Update the reuse distance histogram.
				uint64_t bin = (reuse_distance / 200) * 20;
				if (reuse_distance >= 200)
					bin = 200;
				uint64_t bin_cnt = reuse_histogram[bin];
				reuse_histogram[bin] = bin_cnt + 1;
				reuse_average += reuse_distance;
			}
			last_access[set_index] = currentClockCycle;
			num_sets_accessed++;
			}*/
		
		uint64_t return_value = 0;
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

				if(DEBUG_TAG_BUFFER || ENABLE_TAG_BUFFER_USAGE_LOG)
				{
					sets_hit[tag_buffer_set] = sets_hit[tag_buffer_set] + 1;
				}

				if(ENABLE_TAG_BUFFER_USAGE_LOG)
				{				
					num_tags_used++;
				}

				// TODO: Not sure if this is going to work, need to check it
				(*it).used = true;
				// update the timestamp (not sure if this is the right way to go about this)
				(*it).ts = currentClockCycle;
				if((*it).prefetched == true)
					return_value = 3;
				else if((*it).demand == true)
					return_value = 1;
				else
					return_value = 2;
			}
		}
		// search the buffer for the adjacent sets tags
		if(tagReplacement == tag_nnn)
		{
			for(uint64_t set_index_var = set_index+1; set_index_var < set_index+9; set_index_var++)
			{
				uint64_t tag_buffer_set2;
				if(ENABLE_SET_CHANNEL_INTERLEAVE)
				{
					tag_buffer_set2 = ((set_index_var) / NUM_CHANNELS) % NUM_TAG_SETS;
				}
				else
			        {
					tag_buffer_set2 = (set_index_var) % NUM_TAG_SETS;
				}

				for(std::list<tag_line>::iterator it = tag_buffer[tag_buffer_set2].begin(); it != tag_buffer[tag_buffer_set2].end(); it++)
			        {
					if((*it).set_index == (set_index_var))
				        {	
						// update the timestamp (not sure if this is the right way to go about this)
						(*it).ts = currentClockCycle;
					}
				}
			}
		}
		if(DEBUG_COMBO_TAG)
		{
			debug_tag_buffer << "big ole MISS... \n";
			debug_tag_buffer << "================\n\n";
		}

		// didn't find the tag
		// add a victim tag requested map entry if the requested tag was a recent victim
		if(!victim_tag_list.empty())
		{
			for(std::list<EvictedTagEntry>::iterator tag_it = victim_tag_list.begin(); tag_it != victim_tag_list.end(); tag_it++)
			{
				if((*tag_it).set_index == set_index)
				{
					victim_tag_requested[set_index] = currentClockCycle - (*tag_it).cycle_evicted;
				}
			}
		}
		return return_value;
	}
	
	void TagBuffer::printBufferUsage()
	{
		debug_tag_buffer << "\n\n********************************************\n";
		debug_tag_buffer << ">>>>>>> Tags Buffered and Use Rate <<<<<<<<\n";
		debug_tag_buffer << "Tags buffered: " << num_tags_buffered << "\n";
		debug_tag_buffer << "Tags used: " << num_tags_used << "\n";
		debug_tag_buffer << "Tags used rate: " << ((float)num_tags_used / num_tags_buffered) << "\n";

		debug_tag_buffer << "********************************************\n";
		debug_tag_buffer << ">>>>>>> Final Tag Set Usage Counts <<<<<<<<\n";
		for(uint64_t j = 0; j < NUM_TAG_SETS; j++)
		{
			debug_tag_buffer << "Set " << j << " : " << sets_accessed[j] << "\n";
		}

		debug_tag_buffer << "\n\n********************************************\n";
		debug_tag_buffer << ">>>>>>> Final Tag Set Hit Counts <<<<<<<<\n";
		for(uint64_t j = 0; j < NUM_TAG_SETS; j++)
		{
			debug_tag_buffer << "Set " << j << " : " << sets_hit[j] << "\n";
		}
		
		debug_tag_buffer << "\n\n********************************************\n";
		debug_tag_buffer << ">>>>>>> Victim Request Histogram <<<<<<<<\n";
		debug_tag_buffer << "number of victim requests is " << victim_tag_requested.size() << "\n";
		unordered_map<uint64_t, uint64_t>::iterator victim_it; 
		for(victim_it = victim_tag_requested.begin(); victim_it != victim_tag_requested.end(); victim_it++) 
		{
			uint64_t bin = ((*victim_it).second / 50) * 50;
			if ((*victim_it).second >= 25000)
				bin = 25000;
			uint64_t bin_cnt = victim_tag_histogram[bin];
			victim_tag_histogram[bin] = bin_cnt + 1;
		}
		for(uint64_t victim_bin = 0; victim_bin <= 25000; victim_bin += 50)
		{
			debug_tag_buffer << victim_bin << " : " << victim_tag_histogram[victim_bin] << "\n";
		}

		/*debug_tag_buffer << "\n\n********************************************\n";
		debug_tag_buffer << ">>>>>>> Set Reuse Histogram <<<<<<<<\n";
		for (uint64_t bin = 0; bin <= 200; bin += 20)
		{
			debug_tag_buffer << bin << ": " << reuse_histogram[bin] << "\n";
		}

		debug_tag_buffer << "Set Reuse Average: " << (float)(reuse_average / num_sets_accessed) << "\n";*/

		debug_tag_buffer.flush();
		debug_tag_buffer.close();
	}

	void TagBuffer::printStrides()
	{
		record_strides << "********************************************\n";
		record_strides << ">>>>>>> Stride Histogram <<<<<<<<\n";
		for(uint64_t si = 0; si < 8; si++)
		{
			record_strides << ">>>>>>>> Stides that are " << si << " accesses away <<<<<<<< \n";
			for(uint64_t h = 0; h < NUM_SETS; h++)
			{
				// we're really only interested in the strides that actually happened
				if(access_stride_histogram[si][h] > 0)
				{
					record_strides << "stride " << h << " : " << access_stride_histogram[si][h] << "\n";
				}
			}
		}
		record_strides.flush();
		record_strides.close();
	}

} // Namespace HybridSim
