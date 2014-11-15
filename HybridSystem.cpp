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

#include "HybridSystem.h"

using namespace std;

namespace HybridSim {

	HybridSystem::HybridSystem(uint id, string ini)
	{
		if (ini == "")
		{
			hybridsim_ini = "";
			char *base_path = getenv("HYBRIDSIM_BASE");
			if (base_path != NULL)
			{
				hybridsim_ini.append(base_path);
				hybridsim_ini.append("/");
			}
			hybridsim_ini.append("../HybridSim/ini/hybridsim.ini");
		}
		else
		{
			hybridsim_ini = ini;
		}

		string pattern = "/ini/";
		string inipathPrefix;

		unsigned found = hybridsim_ini.rfind(pattern); // Find the last occurrence of "/ini/"
		assert (found != string::npos);
		inipathPrefix = hybridsim_ini.substr(0,found);
		inipathPrefix.append("/");

		iniReader.read(hybridsim_ini);
		if (ENABLE_LOGGER)
			log.init();

		// Make sure that there are more cache pages than pages per set. 
		assert(CACHE_PAGES >= SET_SIZE);

		systemID = id;
		cerr << "Creating Cache using NVDIMM with " << nvdimm_ini << "\n";
		llcache = NVDSim::getNVDIMMInstance(1,nvdimm_ini,"ini/def_system.ini",inipathPrefix,"");
	

		cerr << "Creating Backing Store using DRAMSim with " << dram_ini << "\n";
		uint64_t dram_size = (TOTAL_PAGES * PAGE_SIZE) >> 20;
		dram_size = (dram_size == 0) ? 1 : dram_size; // DRAMSim requires a minimum of 1 MB, even if HybridSim isn't going to use it.
		dram_size = (OVERRIDE_DRAM_SIZE == 0) ? dram_size : OVERRIDE_DRAM_SIZE; // If OVERRIDE_DRAM_SIZE is non-zero, then use it.
		back = DRAMSim::getMemorySystemInstance(dram_ini, sys_ini, inipathPrefix, "resultsfilename", dram_size);
		cerr << "Done with creating memories" << endl;
		
		// Set up the callbacks for NVDIMM.
		typedef NVDSim::Callback <HybridSystem, void, uint64_t, uint64_t, uint64_t, bool> nvdsim_callback_t;
		NVDSim::Callback_t *nv_read_cb = new nvdsim_callback_t(this, &HybridSystem::CacheReadCallback);
		NVDSim::Callback_t *nv_write_cb = new nvdsim_callback_t(this, &HybridSystem::CacheWriteCallback);
		NVDSim::Callback_t *nv_crit_cb = new nvdsim_callback_t(this, &HybridSystem::CacheCriticalLineCallback);
		llcache->RegisterCallbacks(nv_read_cb, nv_crit_cb, nv_write_cb, NULL);
		
		// Set up the callbacks for DRAM.
		typedef DRAMSim::Callback <HybridSystem, void, uint, uint64_t, uint64_t> dramsim_callback_t;
		DRAMSim::TransactionCompleteCB *read_cb = new dramsim_callback_t(this, &HybridSystem::BackReadCallback);
		DRAMSim::TransactionCompleteCB *write_cb = new dramsim_callback_t(this, &HybridSystem::BackWriteCallback);
		back->RegisterCallbacks(read_cb, write_cb, NULL);

		decoder = AddressDecode();
		if(DEBUG_COMBO_TAG)
		{
			tbuff.initializeSetTracking();
		}

		// Need to check the queue when we start.
		check_queue = true;

		// No delay to start with.
		delay_counter = 0;

		// No active transaction to start with.
		active_transaction_flag = false;

		// Call the restore cache state function.
		// If ENABLE_RESTORE is set, then this will fill the cache table.
		restoreCacheTable();

		cout << "cache pages are " << ACTUAL_CACHE_PAGES << "\n";

		// Load prefetch data.
		if (ENABLE_PERFECT_PREFETCHING)
		{
			// MOVE THIS TO A FUNCTION.

			// Open prefetch file.
			ifstream prefetch_file;
			prefetch_file.open(PREFETCH_FILE, ifstream::in);
			if (!prefetch_file.is_open())
			{
				cerr << "ERROR: Failed to load prefetch file: " << PREFETCH_FILE << "\n";
				abort();
			}

			// Declare variables for parsing string and uint64_t.
			string parse_string;
			uint64_t num_sets, cur_set, set_size, set_counter, tmp_num;

			// Parse prefetch data.
			prefetch_file >> parse_string;
			if (parse_string != "NUM_SETS")
			{
				cerr << "ERROR: Invalid prefetch file format. NUM_SETS does not appear at beginning.\n";
				abort();
			}
			
			prefetch_file >> num_sets;

			for (cur_set = 0; cur_set < num_sets; cur_set++)
			{
				prefetch_file >> parse_string;
				if (parse_string != "SET")
				{
					cerr << "ERROR: Invalid prefetch file format. SET does not appear at beginning of set " << cur_set << ".\n";
					abort();
				}

				prefetch_file >> tmp_num;
				if (tmp_num != cur_set)
				{
					cerr << "ERROR: Invalid prefetch file format. Sets not given in order. (" << cur_set << ")\n";
					abort();
				}
				
				// Read the size fo this set.
				prefetch_file >> set_size;

				// Create a new entry in the maps.
				prefetch_access_number[cur_set] = list<uint64_t>();
				prefetch_flush_addr[cur_set] = list<uint64_t>();
				prefetch_new_addr[cur_set] = list<uint64_t>();
				prefetch_counter[cur_set] = 0;

				// Process each prefetch.
				for (set_counter = 0; set_counter < set_size; set_counter++)
				{
					// Read and store the access counter.
					prefetch_file >> tmp_num;
					prefetch_access_number[cur_set].push_back(tmp_num);

					// Read and store the flush address.
					prefetch_file >> tmp_num;
					prefetch_flush_addr[cur_set].push_back(tmp_num);

					// Read and store the new address.
					prefetch_file >> tmp_num;
					prefetch_new_addr[cur_set].push_back(tmp_num);
				}
			}

			prefetch_file.close();
		}

		// Initialize size/max counters.
		// Note: Some of this is just debug info, but I'm keeping it around because it is useful.
		pending_count = 0; // This is used by TraceBasedSim for MAX_PENDING.
		max_cache_pending = 0;
		pending_pages_max = 0;
		trans_queue_max = 0;
		trans_queue_size = 0; // This is not debugging info.

		tlb_misses = 0;
		tlb_hits = 0;

		total_prefetches = 0;
		unused_prefetches = 0;
		unused_prefetch_victims = 0;
		prefetch_hit_nops = 0;

		unique_one_misses = 0;
		unique_stream_buffers = 0;
		stream_buffer_hits = 0;

		// Create file descriptors for debugging output (if needed).
		if (DEBUG_VICTIM) 
		{
			debug_victim.open("debug_victim.log", ios_base::out | ios_base::trunc);
			if (!debug_victim.is_open())
			{
				cerr << "ERROR: HybridSim debug_victim file failed to open.\n";
				abort();
			}
		}

		if (DEBUG_NVDIMM_TRACE) 
		{
			debug_nvdimm_trace.open("nvdimm_trace.log", ios_base::out | ios_base::trunc);
			if (!debug_nvdimm_trace.is_open())
			{
				cerr << "ERROR: HybridSim debug_nvdimm_trace file failed to open.\n";
				abort();
			}
		}

		if (DEBUG_FULL_TRACE) 
		{
			debug_full_trace.open("full_trace.log", ios_base::out | ios_base::trunc);
			if (!debug_full_trace.is_open())
			{
				cerr << "ERROR: HybridSim debug_full_trace file failed to open.\n";
				abort();
			}
		}

		if (DEBUG_SET_ADDRESSES)
		{
			debug_set_addresses.open("set_addresses.log", ios_base::out | ios_base::trunc);
			if (!debug_set_addresses.is_open())
			{
				cerr << "ERROR: HybridSim debug_set_address file failed to open.\n";
				abort();
			}
		}	
	}

	HybridSystem::~HybridSystem()
	{
		if (DEBUG_VICTIM)
			debug_victim.close();

		if (DEBUG_NVDIMM_TRACE)
			debug_nvdimm_trace.close();

		if (DEBUG_FULL_TRACE)
			debug_full_trace.close();
		
		if(DEBUG_SET_ADDRESSES)
			debug_set_addresses.close();
	}

	// static allocator for the library interface
	HybridSystem *getMemorySystemInstance(uint id, string ini)
	{
		return new HybridSystem(id, ini);
	}


	void HybridSystem::update()
	{
		// Process the transaction queue.
		// This will fill the cache_queue and back_queue.

		if (cache_pending.size() > max_cache_pending)
			max_cache_pending = cache_pending.size();
		if (pending_pages.size() > pending_pages_max)
			pending_pages_max = pending_pages.size();
		if (trans_queue_size > trans_queue_max)
			trans_queue_max = trans_queue_size;

		// Log the queue length.
		bool idle = (trans_queue.empty()) && (pending_pages.empty());
		bool back_idle = (back_queue.empty()) && (back_pending.empty());
		bool cache_idle = (cache_queue.empty()) && (cache_pending.empty());
		if (ENABLE_LOGGER)
			log.access_update(trans_queue_size, idle, back_idle, cache_idle);		


		// Used to see if any work is done on this cycle.
		bool sent_transaction = false;


		list<Transaction>::iterator it = trans_queue.begin();
		while((it != trans_queue.end()) && (pending_pages.size() < NUM_SETS) && (check_queue) && (delay_counter == 0))
		{
			// Compute the page address.
			uint64_t back_addr = ALIGN((*it).address);
			uint64_t page_addr = PAGE_ADDRESS(back_addr);


			// Check to see if this page is open under contention rules.
			if (contention_is_unlocked(back_addr))
			{
				// Lock the page.
				contention_lock(back_addr);

				// Log the page access.
				if (ENABLE_LOGGER)
					log.access_page(page_addr);

				// Set this transaction as active and start the delay counter, which
				// simulates the SRAM cache tag lookup time.
				active_transaction = *it;
				active_transaction_flag = true;
				delay_counter = CONTROLLER_DELAY;
				sent_transaction = true;

				// Check that this page is in the TLB.
				// Do not do this for SYNC_ALL_COUNTER transactions because the page address refers
				// to the cache line, not the back page address, so the TLB isn't needed.
				if ((*it).transactionType != SYNC_ALL_COUNTER)
					check_tlb(page_addr);

				// Delete this item and skip to the next.
				it = trans_queue.erase(it);
				trans_queue_size--;

				break;
			}
			else
			{
				// Log the set conflict.
				if (ENABLE_LOGGER)
					log.access_contention_conflict(SET_INDEX(page_addr));

				// Skip to the next and do nothing else.
				++it;
			}
		}

		// See if there are any transactions ready to be processed.
		// moved this to right after the active transaction is set because there were situations
		// where we could have the pending state change between setting up a transaction and when it was
		// actually processed, leading to pending conflicts
		if ((active_transaction_flag) && (delay_counter == 0))
		{
				ProcessTransaction(active_transaction);
				active_transaction_flag = false;
		}

		// If there is nothing to do, wait until a new transaction arrives or a pending set is released.
		// Only set check_queue to false if the delay counter is 0. Otherwise, a transaction that arrives
		// while delay_counter is running might get missed and stuck in the queue.
		if ((sent_transaction == false) && (delay_counter == 0))
		{
			this->check_queue = false;
		}


		// Process CACHE transaction queue until it is empty or addTransaction returns false.
		// Note: This used to be a while, but was changed ot an if to only allow one
		// transaction to be sent to the CACHE per cycle.
		bool not_full = true;
		if (not_full && !cache_queue.empty())
		{
			Transaction tmp = cache_queue.front();
			bool isWrite;
			if (tmp.transactionType == DATA_WRITE)
				isWrite = true;
			else
				isWrite = false;
			not_full = llcache->addTransaction(isWrite, tmp.address);
			if (not_full)
			{
				cache_queue.pop_front();
				cache_pending_set.insert(tmp.address);
			}
		}

		// Process Back transaction queue until it is empty or addTransaction returns false.
		// Note: This used to be a while, but was changed ot an if to only allow one
		// transaction to be sent to the back per cycle.
		not_full = true;
		if (not_full && !back_queue.empty())
		{
			bool isWrite;

			Transaction tmp = back_queue.front();
			if (tmp.transactionType == DATA_WRITE)
				isWrite = true;
			else
				isWrite = false;
			not_full = back->addTransaction(isWrite, tmp.address);
			if (not_full)
			{
				back_queue.pop_front();

				if (DEBUG_NVDIMM_TRACE)
				{
					debug_nvdimm_trace << currentClockCycle << " " << (isWrite ? 1 : 0) << " " << tmp.address << "\n";
					debug_nvdimm_trace.flush();
				}
			}
		}

		// Decrement the delay counter.
		if (delay_counter > 0)
		{
			delay_counter--;
		}


		// Update the logger.
		if (ENABLE_LOGGER)
			log.update();

		// Update the memories.
		llcache->update();
		back->update();

		// Update the tag buffer, not sure if we need this right now but here it is
		tbuff.update();

		// Increment the cycle count.
		step();
	}

	bool HybridSystem::addTransaction(bool isWrite, uint64_t addr)
	{
		if (DEBUG_CACHE)
			cerr << "\n" << currentClockCycle << ": " << "Adding transaction for address=" << addr << " isWrite=" << isWrite << endl;

		TransactionType type;
		if (isWrite)
		{
			type = DATA_WRITE;
		}
		else
		{
			type = DATA_READ;
		}
		Transaction t = Transaction(type, addr, NULL);
		return addTransaction(t);
	}

	bool HybridSystem::addTransaction(Transaction &trans)
	{

		if (REMAP_MMIO)
		{
			if ((trans.address >= THREEPOINTFIVEGB) && (trans.address < FOURGB))
			{
				// Do not add this transaction to the queue because it is in the MMIO range.
				// Just issue the callback and return.
				if (trans.transactionType == DATA_READ)
				{
					if (ReadDone != NULL)
						(*ReadDone)(systemID, trans.address, currentClockCycle);
				}
				else if (trans.transactionType == DATA_WRITE)
				{
					if (WriteDone != NULL)
						(*WriteDone)(systemID, trans.address, currentClockCycle);
				}
				else
					assert(0);

				log.mmio_dropped();

				return true;
			}
			else if (trans.address >= FOURGB)
			{
				// Subtract 0.5 GB from the address to adjust for MMIO.
				trans.address -= HALFGB;

				log.mmio_remapped();
			}
		}

		pending_count += 1;

		trans_queue.push_back(trans);
		trans_queue_size++;

		if ((trans.transactionType == PREFETCH) || (trans.transactionType == FLUSH))
		{
			ERROR("PREFETCH/FLUSH not allowed in addTransaction()");
			abort();
		}

		// Start the logging for this access.
		if (ENABLE_LOGGER)
			log.access_start(trans.address);

		if (DEBUG_FULL_TRACE)
		{
			debug_full_trace << currentClockCycle << " " << ((trans.transactionType == DATA_WRITE) ? 1 : 0) << " " << trans.address << "\n";
			debug_full_trace.flush();
		}

		// Restart queue checking.
		this->check_queue = true;

		return true; // TODO: Figure out when this could be false.
	}

	void HybridSystem::addPrefetch(uint64_t prefetch_addr)
	{
		// Create prefetch transaction.
		Transaction prefetch_transaction = Transaction(PREFETCH, prefetch_addr, NULL);

		// Push the operation onto the front of the transaction queue (so it executes immediately).
		trans_queue.push_front(prefetch_transaction);
		trans_queue_size += 1;

		pending_count += 1;

		// Restart queue checking.
		this->check_queue = true;
	}

	void HybridSystem::addFlush(uint64_t flush_addr)
	{
		// Create flush transaction.
		Transaction flush_transaction = Transaction(FLUSH, flush_addr, NULL);

		// Push the operation onto the front of the transaction queue (so it executes immediately).
		trans_queue.push_front(flush_transaction);
		trans_queue_size += 1;

		pending_count += 1;

		// Restart queue checking.
		this->check_queue = true;
	}

	bool HybridSystem::WillAcceptTransaction()
	{
		// Always true for now since MARSS expects this.
		// Might change later.
		return true;
	}

	uint64_t HybridSystem::getComboDataAddr(uint64_t set_index, uint64_t i)
	{
		// accounts for how many accesses each set takes up
		//uint64_t set_piece = (set_index / NVDSim::NUM_PACKAGES) * SET_SIZE;
		// accounts for the accesses that are lost because they are either used for tags or can't be used
		//uint64_t waste_piece = (((set_index / NVDSim::NUM_PACKAGES) / SETS_PER_LINE) + 1) * (TAG_OFFSET + WASTE_OFFSET);

		uint64_t channel_piece = 0;
		uint64_t rank_piece = 0;
		uint64_t bank_piece = 0;
		uint64_t row_piece =  0;
		uint64_t this_offset_amount = 0;
		uint64_t next_address = 0;
		if(ENABLE_SET_CHANNEL_INTERLEAVE)
		{
			channel_piece = set_index % NVDSim::NUM_PACKAGES;
			rank_piece = (set_index / (NVDSim::NUM_PACKAGES * SETS_PER_LINE)) % NVDSim::DIES_PER_PACKAGE;
			bank_piece = (set_index / (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * SETS_PER_LINE)) % NVDSim::PLANES_PER_DIE;
			row_piece =  (set_index / (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE * SETS_PER_LINE)) % NVDSim::VIRTUAL_BLOCKS_PER_PLANE;
			this_offset_amount = (TAG_OFFSET + WASTE_OFFSET) * NUM_ROWS;
			next_address = (channel_piece + (rank_piece * NVDSim::NUM_PACKAGES) + (bank_piece * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE)) + (row_piece * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE)) + this_offset_amount + (i * NUM_ROWS) + (((set_index / (NVDSim::NUM_PACKAGES)) % SETS_PER_LINE)*SET_SIZE * NUM_ROWS)) * NVDSim::NV_PAGE_SIZE;
		}
		else
		{
			channel_piece = (set_index / SETS_PER_LINE) % NVDSim::NUM_PACKAGES;
			rank_piece = (set_index / (NVDSim::NUM_PACKAGES * SETS_PER_LINE)) % NVDSim::DIES_PER_PACKAGE;
			bank_piece = (set_index / (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * SETS_PER_LINE)) % NVDSim::PLANES_PER_DIE;
			row_piece =  (set_index / (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE * SETS_PER_LINE)) % NVDSim::VIRTUAL_BLOCKS_PER_PLANE;
			this_offset_amount = (TAG_OFFSET + WASTE_OFFSET) * NUM_ROWS;
			next_address = (channel_piece + (rank_piece * NVDSim::NUM_PACKAGES) + (bank_piece * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE)) + (row_piece * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE)) + this_offset_amount + (i * NUM_ROWS) + ((set_index % SETS_PER_LINE)*SET_SIZE * NUM_ROWS)) * NVDSim::NV_PAGE_SIZE;
		}
		// commented for now for future study
		/*
		// accounts for the number of accesses to add in order to space adjacent sets out across different channels
		uint64_t blocks_per_channel = NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE * NVDSim::VIRTUAL_BLOCKS_PER_PLANE * NVDSim::PAGES_PER_BLOCK;
		uint64_t channel_piece = (set_index % NVDSim::NUM_PACKAGES) * blocks_per_channel;	*/

		if(DEBUG_COMBO_TAG)
		{
			cerr << "Combo Data Address Calculation: \n";
			cerr << "set index " << set_index << "\n";
			cerr << "Tag offset " << TAG_OFFSET << "\n";
			cerr << "Waste offset " << WASTE_OFFSET << "\n";
			cerr << "channel piece " << channel_piece << "\n";
			cerr << "rank piece " << rank_piece << "\n";
			cerr << "bank piece " << bank_piece << "\n";
			cerr << "row piece " << row_piece << "\n";
			cerr << "col piece " << (i * NUM_ROWS) + (((set_index / (NVDSim::NUM_PACKAGES)) % SETS_PER_LINE)*SET_SIZE * NUM_ROWS) << "\n";
			cerr << "next address " << next_address << "\n";
		}

		// add it all up to get your address
		return next_address;
	}

	// This is only ever called once per transaction
        void HybridSystem::HitCheck(Transaction &trans, bool tag_miss)
	{
		// trans.address is the original address that we must use to callback.
		// But for our processing, we must use an aligned address (which is aligned to a page in the NV address space).
		uint64_t addr = ALIGN(trans.address);
		
			// Compute the set number and tag
		uint64_t set_index = SET_INDEX(addr);
		uint64_t tag = TAG(addr);

		if(DEBUG_SET_ADDRESSES)
		{
			debug_set_addresses << "=================================================\n";
			debug_set_addresses << "address is: " << addr << "\n"; 
			debug_set_addresses << "set index is " << set_index << "\n";
			debug_set_addresses << "address list is : \n";
		}

		if(ENABLE_LOGGER)
			log.access_set(set_index);

		list<uint64_t> set_address_list;
		bool hit = false;
		uint64_t cache_address = *(set_address_list.begin());
		uint64_t cur_address;
		cache_line cur_line;

		// generate the memory address list for this set
		for (uint64_t i=0; i<SET_SIZE; i++)
		{
			uint64_t next_address = 0;
			if(assocVersion == channel)
			{			
				next_address = (set_index * SET_SIZE) * (i * PAGE_SIZE);
			}
			else if(assocVersion == combo_tag)
			{
				next_address = getComboDataAddr(set_index, i);
			}
			else
			{
				next_address = (i * NUM_SETS + set_index) * PAGE_SIZE;
			}

			if(DEBUG_SET_ADDRESSES)
			{
				debug_set_addresses << "NUM_SETS is " << NUM_SETS << " set_index is " << set_index << " PAGE_SIZE is " << PAGE_SIZE << "\n";
				debug_set_addresses << "way " << i << " address is: " << next_address << " (hex: " << hex << next_address << dec << " )\n";
			}
			set_address_list.push_back(next_address);
		}

		// see if we have a hit
		for (list<uint64_t>::iterator it = set_address_list.begin(); it != set_address_list.end(); ++it)
		{
			cur_address = *it;
			if (cache.count(cur_address) == 0)
			{
				// If i is not allocated yet, allocate it.
				cache[cur_address] = cache_line();
			}
			
			cur_line = cache[cur_address];
				
			if (cur_line.valid && (cur_line.tag == tag))
			{
				hit = true;
				cache_address = cur_address;
				
				if (DEBUG_CACHE)
				{
					cerr << currentClockCycle << ": " << "HIT: " << cur_address << " " << " " << cur_line.str() << 
						" (set: " << set_index << ")" << endl;
				}
				
				break;
			}
		}

		// Place access_process here and combine it with access_cache.
		// Tell the logger when the access is processed (used for timing the time in queue).
		// Only do this for DATA_READ and DATA_WRITE.
		if ((ENABLE_LOGGER) && ((trans.transactionType == DATA_READ) || (trans.transactionType == DATA_WRITE)))
			log.access_process(trans.address, trans.transactionType == DATA_READ, hit);

		// Handle prefetching operations.
		if (ENABLE_PERFECT_PREFETCHING && ((trans.transactionType == DATA_READ) || (trans.transactionType == DATA_WRITE)))
		{
			// Increment prefetch counter for this set.
			prefetch_counter[set_index]++;

			// If there are any prefetches left in this set prefetch list.
			if (!prefetch_access_number[set_index].empty())
			{
				// If the counter exceeds the front of the access list, 
				// then issue a prefetch and pop the front of the prefetch lists for this set.
				// This must be a > because the prefetch must only happen AFTER the access.
				if (prefetch_counter[set_index] > prefetch_access_number[set_index].front())
				{

					// Add prefetch, then add flush (this makes flush run first).
					addPrefetch(prefetch_new_addr[set_index].front());
					addFlush(prefetch_flush_addr[set_index].front());

					// Go to the next prefetch in this set.
					prefetch_access_number[set_index].pop_front();
					prefetch_flush_addr[set_index].pop_front();
					prefetch_new_addr[set_index].pop_front();
				}
			}
		}


		if (hit)
		{
			// Lock the line that was hit (so it cannot be selected as a victim while being processed).
			contention_cache_line_lock(cache_address);

			if ((ENABLE_STREAM_BUFFER) && 
					((trans.transactionType == DATA_READ) || (trans.transactionType == DATA_WRITE)))
			{
				stream_buffer_hit_handler(PAGE_ADDRESS(addr));
			}

			// Issue operation to the CACHE.
			if (trans.transactionType == DATA_READ)
			{
				if(assocVersion == tag_tlb)
				{
					CacheRead(trans.address, addr, cache_address, trans, false);
				}
				// we issued a read to the cache to get the tags but now we have to issue a read
				// to get the data
				else if(assocVersion == combo_tag)
				{
					CacheRead(trans.address, addr, cache_address, trans, false);

					if(ENABLE_TAG_PREFETCH && tag_miss)
					{
						// issue the prefetches here, these should go on the queue after the actual data read
						IssueTagPrefetch(set_index, *(set_address_list.begin()));
					}
				}
				// if we're not doing a separate tag cache then we don't need to issue any reads here
				// we already issued a read to get the data from the cache
				// instead we're just done now
				else
				{
					contention_unlock(addr, trans.address, "CACHE_READ", false, 0, true, cache_address);
					ReadDoneCallback(systemID, trans.address, currentClockCycle);
				}
			}
			else if(trans.transactionType == DATA_WRITE)
				CacheWrite(trans.address, addr, cache_address);
			else if(trans.transactionType == FLUSH)
			{
				Flush(cache_address);
			}
			else if(trans.transactionType == PREFETCH)
			{
				// Prefetch hits are just NOPs.
				uint64_t back_address = addr;
				contention_unlock(back_address, back_address, "PREFETCH (hit)", false, 0, true, cache_address);

				prefetch_hit_nops++;

				return; 
			}
			else if(trans.transactionType == SYNC)
			{
				sync(addr, cache_address, trans);
				//contention_unlock(addr, addr, "SYNC (hit)", false, 0, true, cache_address);
			}
			else
			{
				assert(0);
			}
		}

		if (!hit)
		{
			// Lock the whole page.
			contention_page_lock(addr);

			// Make sure this isn't a FLUSH before proceeding.
			if(trans.transactionType == FLUSH)
			{
				// We allow FLUSH to miss the cache because of non-determinism from marss.
				uint64_t back_address = addr;
				contention_unlock(back_address, back_address, "FLUSH (miss)", false, 0, false, 0);

				// TODO: Add some logging for this event.

				return;
			}

			assert(trans.transactionType != SYNC);

			if ((SEQUENTIAL_PREFETCHING_WINDOW > 0) && (trans.transactionType != PREFETCH))
			{
				issue_sequential_prefetches(addr);
			}

			if ((ENABLE_STREAM_BUFFER) && (trans.transactionType != PREFETCH))
			{
				stream_buffer_miss_handler(PAGE_ADDRESS(addr));
			}

			// Select a victim offset within the set
			// PaulMod: Added victim select function
			cache_address = VictimSelect(set_index, addr, cur_address, cur_line, set_address_list);
			cur_line = cache[cache_address];

			// Log the victim, set, etc.
			// THIS MUST HAPPEN AFTER THE CUR_LINE IS SET TO THE VICTIM LINE.
			uint64_t victim_back_addr = BACK_ADDRESS(cur_line.tag, set_index);
			if ((ENABLE_LOGGER) && ((trans.transactionType == DATA_READ) || (trans.transactionType == DATA_WRITE)))
				log.access_miss(PAGE_ADDRESS(addr), victim_back_addr, set_index, cache_address, cur_line.dirty, cur_line.valid);


			// Lock the victim page so it will not be selected for eviction again during the processing of this
			// transaction's miss and so further transactions to this page cannot happen.
			// Only lock it if the cur_line is valid.
			if (cur_line.valid)
				contention_victim_lock(victim_back_addr);

			// Lock the cache line so no one else tries to use it while this miss is being serviced.
			contention_cache_line_lock(cache_address);


			if (DEBUG_CACHE)
			{
				cerr << currentClockCycle << ": " << "MISS: victim is cache_address " << cache_address <<
						" (set: " << set_index << ")" << endl;
				cerr << cur_line.str() << endl;
				cerr << currentClockCycle << ": " << "The victim is dirty? " << cur_line.dirty << endl;
			}

			if ((cur_line.prefetched) && (cur_line.used == false))
			{
				// An unused prefetch is being removed from the cache, so transfer the count
				// to the unused_prefetch_victims counter.
				unused_prefetches--;
				unused_prefetch_victims++;
			}

			Pending p;
			p.orig_addr = trans.address;
			p.back_addr = addr;
			p.cache_addr = cache_address;
			p.victim_tag = cur_line.tag;
			p.victim_valid = cur_line.valid;
			p.callback_sent = false;
			p.type = trans.transactionType;

			// Read the line that missed from the backing store.
			// This is started immediately to minimize the latency of the waiting user of HybridSim.
			LineRead(p);

			// If the cur_line is dirty, then do a victim writeback process (starting with VictimRead).
			if (cur_line.dirty)
			{
				VictimRead(p);
			}

			// try to read the other tags in parallel with the replacement, these tag lookups should be put
			// in the cache queue after the line read so they shouldn't slow the replacement down
			if(assocVersion == combo_tag && tag_miss && ENABLE_TAG_PREFETCH)
			{
				IssueTagPrefetch(set_index, *(set_address_list.begin()));
			}
		}
	}

	// helper method that returns the tag location within a row
	
	// helper method that does the math to find the tags for a set of data
	// put into its own function to avoid code repetition
	uint64_t HybridSystem::getComboTagAddr(uint64_t set_index, uint64_t data_address)
	{
		AddressSet address_stuff = decoder.getDecode(data_address);
		
		uint64_t set_index_mod = 0;
		if(ENABLE_SET_CHANNEL_INTERLEAVE)
		{
			// dividing by the number of channels spreads the sets out across the channels
			set_index_mod = (set_index / NVDSim::NUM_PACKAGES) % SETS_PER_LINE;
		}
		else
		{
			set_index_mod = set_index % SETS_PER_LINE;
		}
			
		uint64_t set_index_pos = 0;
		if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
		{
			set_index_pos = (set_index_mod) / (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP);
		}
		else
		{
			set_index_pos = (set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) / (SETS_PER_TAG_GROUP);
		}
		
		if(DEBUG_COMBO_TAG)
		{
			cerr << "address stuff " << address_stuff.str() << "\n";
			cerr << "set index mod " << set_index_mod << "\n";
			cerr << "set index pos " << set_index_pos << "\n";
		}

		return (address_stuff.channel + (address_stuff.rank * (NVDSim::NUM_PACKAGES)) + (address_stuff.bank * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE)) + (address_stuff.row * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE)) + (set_index_pos * NUM_ROWS)) * NVDSim::NV_PAGE_SIZE;

		/* Commented for future study
		return ((NVDSim::PAGES_PER_BLOCK * (address_stuff.row + NVDSim::VIRTUAL_BLOCKS_PER_PLANE * 
								   (address_stuff.bank + NVDSim::PLANES_PER_DIE * 
								    (address_stuff.rank + (NVDSim::DIES_PER_PACKAGE * 
											   address_stuff.channel))))) + 
											   set_index_pos) * PAGE_SIZE;*/
	}

	void HybridSystem::ProcessTransaction(Transaction &trans)
	{
		// trans.address is the original address that we must use to callback.
		// But for our processing, we must use an aligned address (which is aligned to a page in the NV address space).
		uint64_t addr = ALIGN(trans.address);


		if (trans.transactionType == SYNC_ALL_COUNTER)
		{
			// SYNC_ALL_COUNTER transactions are handled elsewhere.
			syncAllCounter(addr, trans);
			return;
		}


		if (DEBUG_CACHE)
			cerr << "\n" << currentClockCycle << ": " << "Starting transaction for address " << addr << endl;


		if (addr >= (TOTAL_PAGES * PAGE_SIZE))
		{
			// Note: This should be technically impossible due to the modulo in ALIGN. But this is just a sanity check.
			cerr << "ERROR: Address out of bounds - orig:" << trans.address << " aligned:" << addr << "\n";
			abort();
		}

		if(assocVersion == tag_tlb)				
		{
			HitCheck(trans, false);
		}
		// we're simulating storing the tags in dram, so we have to access dram first and then see if the tag is what we're looking for
		// we have an associative cache and we're distributing the ways across channels
		else if(assocVersion == channel)
		{
			// Compute the set number and tag
			uint64_t set_index = SET_INDEX(addr);
			
			if(DEBUG_SET_ADDRESSES)
			{
				debug_set_addresses << "=================================================\n";
				debug_set_addresses << "address is: " << addr << "\n"; 
				debug_set_addresses << "set index is " << set_index << "\n";
				debug_set_addresses << "address list is : \n";
			}

			uint64_t cache_address = set_index * SET_SIZE * PAGE_SIZE;
			if(DEBUG_SET_ADDRESSES)
			{
				debug_set_addresses << "NUM_SETS is " << NUM_SETS << " set_index is " << set_index << " PAGE_SIZE is " << PAGE_SIZE << "\n";
				debug_set_addresses << "way 0  address is: " << cache_address << " (hex: " << hex << cache_address << dec << " )\n";
			}
			
			contention_cache_line_lock(cache_address);
			CacheRead(trans.address, addr, cache_address, trans, true);
		}
		// we're implementing the loh cache so we're issuing a cache lookup transaction to the first way in the set
		else if(assocVersion == loh)
		{
		}
		// we're doing the combo tag thing, so we need to know what is in the tag cache to see if we need to do a lookup
		else if(assocVersion == combo_tag)
		{
			// Compute the set number and tag
			uint64_t set_index = SET_INDEX(addr);
			if(ENABLE_LOGGER)
			{
				log.tag_buffer_access();
			}

			// first see if we alrady have the tags for this set
			uint64_t had_tags = tbuff.haveTags(set_index);
			if(had_tags != 0)
			{
				// if we do have the tags then there's no need to issue a tag lookup to the cache
				// so we just go directly to the check tags phase
				if(ENABLE_LOGGER)
				{
					if(had_tags == 2)
						log.tag_buffer_prefetch_hit();
					else
						log.tag_buffer_hit();
				}
				HitCheck(trans, false);
			}
			else
			{
				// if we don't have the tags then we have to issue a tag lookup to the cache

				if(DEBUG_SET_ADDRESSES)
				{
					debug_set_addresses << "=================================================\n";
					debug_set_addresses << "address is: " << addr << "\n"; 
					debug_set_addresses << "set index is " << set_index << "\n";
				}
				
				// this is the location of the set data that we want
				// we use this to get the channel, rank, bank and row of the tag that we want
				uint64_t data_address = getComboDataAddr(set_index, 0);
				uint64_t cache_address = getComboTagAddr(set_index, data_address);

				if(DEBUG_SET_ADDRESSES)
				{
					debug_set_addresses << "data address " << data_address << "\n";
					debug_set_addresses << "cache tag address " << cache_address << "\n";
				}
				
				contention_cache_line_lock(cache_address);
				CacheRead(trans.address, addr, cache_address, trans, true);
			}
			
		}
		// we have a direct mapped cache so we're just issuing one transaction to the cache
		else
		{
			// Compute the set number and tag
			uint64_t set_index = SET_INDEX(addr);
			uint64_t cache_address = (set_index) * PAGE_SIZE;
			
			if(DEBUG_SET_ADDRESSES)
			{
				debug_set_addresses << "=================================================\n";
				debug_set_addresses << "address is: " << addr << "\n"; 
				debug_set_addresses << "set index is " << set_index << "\n";
				debug_set_addresses << "address list is : \n";

				debug_set_addresses << "NUM_SETS is " << NUM_SETS << " set_index is " << set_index << " PAGE_SIZE is " << PAGE_SIZE << "\n";
				debug_set_addresses << "way 0  address is: " << cache_address << " (hex: " << hex << cache_address << dec << " )\n";
			}
			
			contention_cache_line_lock(cache_address);
			CacheRead(trans.address, addr, cache_address, trans, true);
		}	
	}

	// ***************************************************************************
	// COMBO TAG PREFETCH STUFF
	// ***************************************************************************
	void HybridSystem::IssueTagPrefetch(uint64_t set_index, uint64_t data_address)
	{
		if(DEBUG_TAG_PREFETCH)
		{
			cerr << "ISSUING PREFETCHES \n";
			cerr << "prefetching for set index " << set_index << "\n";
		}
		// get the base address stuff
		AddressSet address_stuff = decoder.getDecode(data_address);

		// get the set index mod
		uint64_t set_index_mod = 0;
		if(ENABLE_SET_CHANNEL_INTERLEAVE)
		{
			// dividing by the number of channels spreads the sets out across the channels
			set_index_mod = (set_index / NVDSim::NUM_PACKAGES) % SETS_PER_LINE;
		}
		else
		{
			set_index_mod = set_index % SETS_PER_LINE;
		}
		
		// trying to align back to the starting set of each tag group
		uint64_t set_index_pos = 0;
		uint64_t temp_set = 0;
		uint64_t set_index_align = 0;
		uint64_t set_group_pos = 0;
		if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
		{
			set_group_pos = (set_index_mod) % (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP);
			set_index_pos = (set_index_mod - EXTRA_SETS_FOR_ZERO_GROUP) % (SETS_PER_LINE / SETS_PER_TAG_GROUP);
			set_index_align = set_index - set_group_pos;
			temp_set = set_index_align + (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP);
		}
		else
		{
			set_group_pos = (set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % (SETS_PER_TAG_GROUP);
			set_index_pos = set_index_mod % (SETS_PER_LINE / SETS_PER_TAG_GROUP);
			set_index_align = set_index - set_group_pos;
			temp_set = set_index_align + SETS_PER_TAG_GROUP;
		}
		
		if(DEBUG_TAG_PREFETCH)
		{
			cerr << "set index mod was " << set_index_mod << "\n";
			cerr << "set index position was " << set_index_pos << "\n";
			cerr << "set index align was " << set_index_align << "\n";
		}

		uint64_t curr_set_addr = (address_stuff.channel + (address_stuff.rank * (NVDSim::NUM_PACKAGES)) + (address_stuff.bank * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE)) + (address_stuff.row * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE)) + (set_index_pos * NUM_ROWS)) * NVDSim::NV_PAGE_SIZE;
		decoder.getDecode(curr_set_addr);

		// allow for variable length prefetches
		uint64_t index_max = 0;
		if(TAG_PREFETCH_WINDOW == 0)
		{
			index_max = (SETS_PER_LINE / SETS_PER_TAG_GROUP);
		}
		else
		{
			index_max = set_index_pos+TAG_PREFETCH_WINDOW+1;
		}

		// only prefetch going forward
		// set_index_pos should be tags that we already have
		uint64_t temp_index_pos = set_index_pos+1;
		uint64_t temp_chan = address_stuff.channel;
		if(temp_index_pos >= ((SETS_PER_LINE-EXTRA_SETS_FOR_ZERO_GROUP) / SETS_PER_TAG_GROUP))
		{
			temp_index_pos = 0;
			temp_chan = temp_chan+1;
			// if we're at the last chan then just stop
			// might want to add some looping stuff here later but lets see how we do
			if(temp_chan >= NVDSim::NUM_PACKAGES)
			{
				return;
			}
		}
	
		for(uint64_t prefetch_index = set_index_pos+1; prefetch_index < index_max; prefetch_index++)
		{		
			uint64_t curr_tag_addr = (temp_chan + (address_stuff.rank * (NVDSim::NUM_PACKAGES)) + (address_stuff.bank * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE)) + (address_stuff.row * (NVDSim::NUM_PACKAGES * NVDSim::DIES_PER_PACKAGE * NVDSim::PLANES_PER_DIE)) + (temp_index_pos * NUM_ROWS)) * NVDSim::NV_PAGE_SIZE;
	
			if(DEBUG_TAG_PREFETCH)
			{
				cerr << "prefetch number " << prefetch_index << "\n";
				cerr << "prefetch set " << temp_set << "\n";
				cerr << "addreess " << curr_tag_addr << "\n";
			}
			decoder.getDecode(curr_tag_addr);
			
			// make sure we're not already reading these tags
			if(cache_pending.count(curr_tag_addr) == 0)
			{
				if(DEBUG_TAG_PREFETCH)
				{
					cerr << "Prefetch actually issued \n";
					cerr << "prefetching tags from address " << curr_tag_addr << "\n";
				}

				if(ENABLE_LOGGER)
				{
					log.tag_buffer_prefetch();
				}

				// lock this cache page
				contention_cache_line_lock(curr_tag_addr);
				
				// now add the transaction
				Transaction t = Transaction(DATA_READ, curr_tag_addr, NULL);
				cache_queue.push_back(t);
				
				// Add a record in the CACHE's pending table.
				Pending p;
				p.op = TAG_PREFETCH;
				p.cache_addr = curr_tag_addr;
				p.orig_addr = 0;
				p.back_addr = temp_set; // use this to pass on the set index		
				p.victim_tag = 0;
				p.victim_valid = false;
				p.callback_sent = false;
				p.type = DATA_READ;			
				assert(cache_pending.count(curr_tag_addr) == 0);
				cache_pending[curr_tag_addr] = p;
				
				// Assertions for "this can't happen" situations.
				assert(cache_pending.count(curr_tag_addr) != 0);	
			}
			temp_index_pos = temp_index_pos+1;
			if(temp_index_pos >= (SETS_PER_LINE / SETS_PER_TAG_GROUP))
			{
				temp_index_pos = 0;
				temp_chan = temp_chan+1;
				// if we're at the last chan then just stop
				// might want to add some looping stuff here later but lets see how we do
				if(temp_chan >= NVDSim::NUM_PACKAGES)
				{
					break;
				}
			}
			
			// move the set index pointer forward so we're pointing at the next set of tags
			uint64_t temp_pos_mod = prefetch_index % (SETS_PER_LINE / SETS_PER_TAG_GROUP);
			// if we're at the end of a row's tag sets then we'll have a big tag set next
			if(temp_pos_mod == 0)
			{
				temp_set = temp_set + (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP);
			}
			else
			{
				temp_set = temp_set + SETS_PER_TAG_GROUP;
			}
		}
	}

	// this is a helper function that handles the case where we would have a victim read
	// but we've already gotten the data as part of a tag lookup
	void HybridSystem::AlreadyReadVictim(Pending p)
	{
		uint64_t data_addr = p.cache_addr + PAGE_OFFSET(p.back_addr);
		cache_pending.erase(p.cache_addr);
		VictimReadFinish(p.orig_addr, p);
		cache_pending_set.erase(data_addr);
	}

	void HybridSystem::VictimRead(Pending p)
	{
		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "Performing VICTIM_READ for (" << p.back_addr << ", " << p.cache_addr << ")\n";

		// back_addr is the original Back address requested from the top level Transaction.
		// victim is the base address of the CACHE page to read.
		// victim_tag is the cache tag for the victim page (used to compute the victim's back address).

		// Increment the pending set/page counter (this is used to ensure that the pending set/page entry isn't removed until both LineRead
		// and VictimRead (if needed) are completely done.
		contention_increment(p.back_addr);

		// Add a record in the CACHE's pending table.
		p.op = VICTIM_READ;
		// here cache_address is a data address in combo tag mode
		assert(cache_pending.count(p.cache_addr) == 0);
		cache_pending[p.cache_addr] = p;

		if(assocVersion == tag_tlb || assocVersion == combo_tag) 
		{
#if SINGLE_WORD
			// Schedule a read from CACHE to get the line being evicted.
			Transaction t = Transaction(DATA_READ, p.cache_addr, NULL);
			cache_queue.push_back(t);
#else
			// Schedule reads for the entire page.
			cache_pending_wait[p.cache_addr] = unordered_set<uint64_t>();
			for(uint64_t i=0; i<PAGE_SIZE/BURST_SIZE; i++)
			{
				uint64_t addr = p.cache_addr + i*BURST_SIZE;
				cache_pending_wait[p.cache_addr].insert(addr);
				Transaction t = Transaction(DATA_READ, addr, NULL);
				cache_queue.push_back(t);
			}
#endif
		}
		else
		{
#if SINGLE_WORD
			// if we're doing in dram tags then we already have the data
			// so just use the simple helper function rather than scheduling another read
			AlreadyVictimRead(p);
#else
			// Schedule reads for the page but issue one lses than the other case because we've alread read one of the pages
			cache_pending_wait[p.cache_addr] = unordered_set<uint64_t>();
			if(PAGE_SIZE/BURST_SIZE > 1)
			{
				for(uint64_t i=1; i<PAGE_SIZE/BURST_SIZE; i++)
				{
					uint64_t addr = p.cache_addr + i*BURST_SIZE;
					cache_pending_wait[p.cache_addr].insert(addr);
					Transaction t = Transaction(DATA_READ, addr, NULL);
					cache_queue.push_back(t);
				}
			}
			// if there was just one read per page anyway, then we've already read that and we're done
			else
			{
				AlreadyReadVictim(p);
			}
#endif
		}	
		
	}

	void HybridSystem::VictimReadFinish(uint64_t addr, Pending p)
	{
		if (DEBUG_CACHE)
		{
			cerr << currentClockCycle << ": " << "VICTIM_READ callback for (" << p.back_addr << ", " << p.cache_addr << ") offset="
				<< PAGE_OFFSET(addr);
		}

#if SINGLE_WORD
		if (DEBUG_CACHE)
			cerr << " num_left=0 (SINGLE_WORD)\n";
#else
		uint64_t cache_page_addr = p.cache_addr;

		if (DEBUG_CACHE)
			cerr << " num_left=" << cache_pending_wait[cache_page_addr].size() << "\n"; 

		// Remove the read that just finished from the wait set.
		cache_pending_wait[cache_page_addr].erase(addr);

		if (!cache_pending_wait[cache_page_addr].empty())
		{
			// If not done with this line, then re-enter pending map.
			cache_pending[cache_page_addr] = p;
			cache_pending_set.erase(addr);
			return;
		}

		// The line has completed. Delete the wait set object and move on.
		cache_pending_wait.erase(cache_page_addr);
#endif


		// Decrement the pending set counter (this is used to ensure that the pending set entry isn't removed until both LineRead
		// and VictimRead (if needed) are completely done.
		contention_decrement(p.back_addr);

		if (DEBUG_CACHE)
		{
			cerr << "The victim read to CACHE line " << PAGE_ADDRESS(addr) << " has completed.\n";
			cerr << "pending_pages[" << PAGE_ADDRESS(p.back_addr) << "] = " << pending_pages[PAGE_ADDRESS(p.back_addr)] << "\n";
		}

		// contention_unlock will only unlock if the pending_page counter is 0.
		// This means that LINE_READ finished first and that the pending set was not removed
		// in the CacheReadFinish or CacheWriteFinish functions (or LineReadFinish for PREFETCH).
		uint64_t victim_address = BACK_ADDRESS(p.victim_tag, SET_INDEX(p.back_addr));
		contention_unlock(p.back_addr, p.orig_addr, "VICTIM_READ", p.victim_valid, victim_address, true, p.cache_addr);

		// Schedule a write to the back to simulate the transfer
		VictimWrite(p);
	}

	void HybridSystem::VictimWrite(Pending p)
	{
		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "Performing VICTIM_WRITE for (" << p.back_addr << ", " << p.cache_addr << ")\n";

		// Compute victim back address.
		// This is where the victim line is stored in the Back address space.
		uint64_t victim_back_addr = (p.victim_tag * NUM_SETS + SET_INDEX(p.back_addr)) * PAGE_SIZE; 

#if SINGLE_WORD
		// Schedule a write to Back to save the evicted line.
		Transaction t = Transaction(DATA_WRITE, victim_back_addr, NULL);
		back_queue.push_back(t);
#else
		// Schedule writes for the entire page.
		for(uint64_t i=0; i<PAGE_SIZE/BACK_BURST_SIZE; i++)
		{
			Transaction t = Transaction(DATA_WRITE, victim_back_addr + i*BACK_BURST_SIZE, NULL);
			back_queue.push_back(t);
		}
#endif

		// No pending event schedule necessary (might add later for debugging though).
	}

	void HybridSystem::LineRead(Pending p)
	{
		if (DEBUG_CACHE)
		{
			cerr << currentClockCycle << ": " << "Performing LINE_READ for (" << p.back_addr << ", " << p.cache_addr << ")\n";
			cerr << "the page address was " << PAGE_ADDRESS(p.back_addr) << endl;
		}

		uint64_t page_addr = PAGE_ADDRESS(p.back_addr);


		// Increment the pending set counter (this is used to ensure that the pending set entry isn't removed until both LineRead
		// and VictimRead (if needed) are completely done.
		contention_increment(p.back_addr);


#if SINGLE_WORD
		// Schedule a read from Back to get the new line 
		Transaction t = Transaction(DATA_READ, page_addr, NULL);
		back_queue.push_back(t);
#else
		// Schedule reads for the entire page.
		back_pending_wait[page_addr] = unordered_set<uint64_t>();
		for(uint64_t i=0; i<PAGE_SIZE/BACK_BURST_SIZE; i++)
		{
			uint64_t addr = page_addr + i*BACK_BURST_SIZE;
			back_pending_wait[page_addr].insert(addr);
			Transaction t = Transaction(DATA_READ, addr, NULL);
			back_queue.push_back(t);
		}
#endif

		// Add a record in the Back's pending table.
		p.op = LINE_READ;
		back_pending[page_addr] = p;
	}


	void HybridSystem::LineReadFinish(uint64_t addr, Pending p)
	{

		if (DEBUG_CACHE)
		{
			cerr << currentClockCycle << ": " << "LINE_READ callback for (" << p.back_addr << ", " << p.cache_addr << ") offset="
				<< PAGE_OFFSET(addr);
		}

#if SINGLE_WORD
		if (DEBUG_CACHE)
			cerr << " num_left=0 (SINGLE_WORD)\n";
#else
		uint64_t page_addr = PAGE_ADDRESS(p.back_addr);

		if (DEBUG_CACHE)
			cerr << " num_left=" << back_pending_wait[page_addr].size() << "\n"; 

		// Remove the read that just finished from the wait set.
		back_pending_wait[page_addr].erase(addr);

		if (!back_pending_wait[page_addr].empty())
		{
			// If not done with this line, then re-enter pending map.
			back_pending[PAGE_ADDRESS(addr)] = p;
			return;
		}

		// The line has completed. Delete the wait set object and move on.
		back_pending_wait.erase(page_addr);
#endif


		// Decrement the pending set counter (this is used to ensure that the pending set entry isn't removed until both LineRead
		// and VictimRead (if needed) are completely done.
		contention_decrement(p.back_addr);

		if (DEBUG_CACHE)
		{
			cerr << "The line read to Back line " << PAGE_ADDRESS(addr) << " has completed.\n";
		}


		// Update the cache state
		cache_line cur_line = cache[p.cache_addr];
		cur_line.tag = TAG(p.back_addr);
		cur_line.dirty = false;
		cur_line.valid = true;
		cur_line.ts = currentClockCycle;
		cur_line.used = false;
		cur_line.access_count = 1; // PaulMod: To support lfu
		if (p.type == PREFETCH)
		{
			cur_line.prefetched = true;
			total_prefetches++;
			unused_prefetches++;
		}
		else
		{
			cur_line.prefetched = false;
		}
		cache[p.cache_addr] = cur_line;

		// Schedule LineWrite operation to store the line in CACHE.
		LineWrite(p);

		// if we're storing tags along with data, then we need to write the new tag data into the cache as well
		if(assocVersion != tag_tlb && assocVersion != direct)
		{
			Pending tp;
			tp.op = TAG_WRITE;
			tp.orig_addr = p.orig_addr;
			tp.back_addr = p.back_addr;
			
			// calculate the tag address
			uint64_t set_index = SET_INDEX(p.back_addr);
			uint64_t tag_address = getComboTagAddr(set_index, p.cache_addr);

			tp.cache_addr = tag_address;
			tp.victim_tag = p.victim_tag;
			tp.victim_valid = p.victim_valid;
			tp.callback_sent = false;
			tp.type = p.type;

			// schedule the write the memory for this tag
			LineWrite(tp);
		}

		// Use the CacheReadFinish/CacheWriteFinish functions to mark the page dirty (DATA_WRITE only), perform
		// the callback to the requesting module, and remove this set from the pending sets to allow future
		// operations to this set to start.
		// Note: Only write operations are pending at this point, which will not interfere with future operations.
		if (p.type == DATA_READ)
			CacheReadFinish(p.cache_addr, p, true);
		else if(p.type == DATA_WRITE)
			CacheWriteFinish(p);
		else if(p.type == PREFETCH)
		{
			// Do not call cache functions because prefetch does not send data back to the caller.

			// Erase the page from the pending set.
			// Note: the if statement is needed to ensure that the VictimRead operation (if it was invoked as part of a cache miss)
			// is already complete. If not, the pending_set removal will be done in VictimReadFinish().

			uint64_t victim_address = BACK_ADDRESS(p.victim_tag, SET_INDEX(p.back_addr));
			contention_unlock(p.back_addr, p.orig_addr, "PREFETCH", p.victim_valid, victim_address, true, p.cache_addr);
		}
	}


	void HybridSystem::LineWrite(Pending p)
	{
		// After a LineRead from back completes, the LineWrite stores the read line into the CACHE.

		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "Performing LINE_WRITE for (" << p.back_addr << ", " << p.cache_addr << ")\n";

#if SINGLE_WORD
		// Schedule a write to CACHE to simulate the write of the line that was read from Back.
		Transaction t = Transaction(DATA_WRITE, p.cache_addr, NULL);
		cache_queue.push_back(t);
#else
		// Schedule writes for the entire page.
		for(uint64_t i=0; i<PAGE_SIZE/BURST_SIZE; i++)
		{
			Transaction t = Transaction(DATA_WRITE, p.cache_addr + i*BURST_SIZE, NULL);
			cache_queue.push_back(t);
		}
#endif

		// No pending event schedule necessary (might add later for debugging though).
	}


	void HybridSystem::CacheRead(uint64_t orig_addr, uint64_t back_addr, uint64_t cache_addr, Transaction &trans, bool tag_lookup)
	{
		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "Performing CACHE_READ for (" << back_addr << ", " << cache_addr << " , lookup? " << tag_lookup << ")\n";

		// Compute the actual CACHE address of the data word we care about.
		uint64_t data_addr = cache_addr + PAGE_OFFSET(back_addr);
		assert(cache_addr == PAGE_ADDRESS(data_addr));
		

		// if we're doing the channel tag lookup scheme then we need to issue a read to each channel to get that channel's tags
		if(assocVersion == channel)
		{
			for(uint64_t i=0; i<SET_SIZE; i++)
			{
				// we adjust the addresses very slightly here to ensure that we can tell them apart
				// in the contention locking code
				// TODO: These addresses need to be adjusted so they access the different ways
				uint64_t addr = data_addr + (i * PAGE_SIZE);
				cache_pending_wait[cache_addr].insert(addr);

				Transaction t = Transaction(DATA_READ, addr, NULL);
				cache_queue.push_back(t);			
			}
		}		
		else
		{
			Transaction t = Transaction(DATA_READ, data_addr, NULL);
			cache_queue.push_back(t);
		}

		// Update the cache state
		// This could be done here or in CacheReadFinish
		// It really doesn't matter (AFAICT) as long as it is consistent.
		cache_line cur_line = cache[cache_addr];
		cur_line.ts = currentClockCycle;
		cur_line.access_count++;
		if ((cur_line.prefetched) && (cur_line.used == false)) // Note: this if statement must come before cur_line.used is set to true.
			unused_prefetches--;
		cur_line.used = true;
		cache[cache_addr] = cur_line;

		// Add a record in the CACHE's pending table.
		Pending p;
		if(tag_lookup)
		{
			p.op = TAG_READ;
		}
		else
		{
			p.op = CACHE_READ;			
		}
		p.cache_addr = cache_addr;
		p.orig_addr = orig_addr;
		p.back_addr = back_addr;		
		p.victim_tag = 0;
		p.victim_valid = false;
		p.callback_sent = false;
		p.type = trans.transactionType;
		assert(cache_pending.count(cache_addr) == 0);
		cache_pending[cache_addr] = p;

		// Assertions for "this can't happen" situations.
		assert(cache_pending.count(cache_addr) != 0);
	}

	void HybridSystem::CacheReadFinish(uint64_t addr, Pending p, bool line_read)
	{
		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "CACHE_READ callback for (" << p.back_addr << ", " << p.cache_addr << ")\n";

		// Erase the page from the pending set.
		uint64_t victim_address = BACK_ADDRESS(p.victim_tag, SET_INDEX(p.back_addr));
		if(assocVersion == direct && !line_read && !p.callback_sent)
		{
			
			Transaction t = Transaction(p.type, p.orig_addr, NULL);
            // Do not erase the page from the pending set yet because we're still working with it
			contention_cache_line_unlock(p.cache_addr);
			HitCheck(t, false);
		}
		else if(assocVersion == combo_tag && !line_read && !p.callback_sent && p.op == TAG_READ)
		{
			// update the tag buffer to now hold the stuff we just got
			// first get the tag group that this data belongs too
			// to do this we first need to know what set we have
			uint64_t set_index = SET_INDEX(p.back_addr);
			// next we figure out which tag group it is
			if(ENABLE_SET_CHANNEL_INTERLEAVE)
			{
				uint64_t set_index_mod = (set_index / NVDSim::NUM_PACKAGES) % SETS_PER_LINE;
				uint64_t set_index_start = 0;
				if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP, 0);
					set_index_start = set_index - (set_index_mod * NVDSim::NUM_PACKAGES);
					for(uint64_t i=0; i<(SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP); i++)
					{
						tags[i] = set_index_start+(i*NVDSim::NUM_PACKAGES);
					}
					tbuff.addTags(tags, false);					
				}
				else
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP, 0);
					set_index_start = set_index - (((set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % SETS_PER_TAG_GROUP)  * NVDSim::NUM_PACKAGES);
					for(uint64_t i=0; i<SETS_PER_TAG_GROUP; i++)
					{
						tags[i] = set_index_start+(i*NVDSim::NUM_PACKAGES);
					}
					tbuff.addTags(tags, false);						
				}
			}
			else
			{
				uint64_t set_index_mod = (set_index) % SETS_PER_LINE;
				uint64_t set_index_start = 0;
				if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP, 0);
					set_index_start = set_index - set_index_mod;
					for(uint64_t i=0; i<(SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP); i++)
					{
						tags[i] = set_index_start+i;
					}
					tbuff.addTags(tags, false);					
				}
				else
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP, 0);
					set_index_start = set_index - ((set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % SETS_PER_TAG_GROUP);
					for(uint64_t i=0; i<SETS_PER_TAG_GROUP; i++)
					{
						tags[i] = set_index_start+i;
					}
					tbuff.addTags(tags, false);						
				}
			}
			
			Transaction t = Transaction(p.type, p.orig_addr, NULL);
            // Do not erase the page from the pending set yet because we're still working with it
			contention_cache_line_unlock(p.cache_addr);
			HitCheck(t, true);
		}
		else if(assocVersion == combo_tag && !line_read && !p.callback_sent && p.op == TAG_PREFETCH)
		{
			uint64_t set_index = p.back_addr;
			
			if(ENABLE_SET_CHANNEL_INTERLEAVE)
			{
				uint64_t set_index_mod = (set_index / NVDSim::NUM_PACKAGES) % SETS_PER_LINE;
				uint64_t set_index_start = 0;
				if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP, 0);
					set_index_start = set_index - (set_index_mod * NVDSim::NUM_PACKAGES);
					for(uint64_t i=0; i<(SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP); i++)
					{
						tags[i] = set_index_start+(i*NVDSim::NUM_PACKAGES);
					}
					tbuff.addTags(tags, true);					
				}
				else
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP, 0);
					set_index_start = set_index - (((set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % SETS_PER_TAG_GROUP)  * NVDSim::NUM_PACKAGES);
					for(uint64_t i=0; i<SETS_PER_TAG_GROUP; i++)
					{
						tags[i] = set_index_start+(i*NVDSim::NUM_PACKAGES);
					}
					tbuff.addTags(tags, true);						
				}
			}
			else
			{
				uint64_t set_index_mod = (set_index) % SETS_PER_LINE;
				uint64_t set_index_start = 0;
				if(set_index_mod < (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP))
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP, 0);
					set_index_start = set_index - set_index_mod;
					for(uint64_t i=0; i<(SETS_PER_TAG_GROUP + EXTRA_SETS_FOR_ZERO_GROUP); i++)
					{
						tags[i] = set_index_start+i;
					}
					tbuff.addTags(tags, true);					
				}
				else
				{
					vector<uint64_t> tags = vector<uint64_t> (SETS_PER_TAG_GROUP, 0);
					set_index_start = set_index - ((set_index_mod-EXTRA_SETS_FOR_ZERO_GROUP) % SETS_PER_TAG_GROUP);
					for(uint64_t i=0; i<SETS_PER_TAG_GROUP; i++)
					{
						tags[i] = set_index_start+i;
					}
					tbuff.addTags(tags, true);						
				}
			}

			contention_cache_line_unlock(p.cache_addr);
			// we're done here
			// no need to do anything more with these
		}
		else if(assocVersion == channel && !line_read && !p.callback_sent)
		{
			// Remove the read that just finished from the wait set.
			cache_pending_wait[p.cache_addr].erase(addr);
			
			if (!cache_pending_wait[p.cache_addr].empty())
			{
				// If not done with this line, then re-enter pending map.
				cache_pending[p.cache_addr] = p;
				cache_pending_set.erase(addr);
				return;
			}
			
			// The line has completed. Delete the wait set object and move on.
			cache_pending_wait.erase(p.cache_addr);
			
			Transaction t = Transaction(p.type, p.orig_addr, NULL);
			// However, do not totally unlock the page from the pending set yet because we're still working with it
			contention_cache_line_unlock(p.cache_addr);
			HitCheck(t, false);
		}
		else
		{			
			// Read operation has completed, call the top level callback.
			// Only do this if it hasn't been sent already by the critical cache line first callback.
			// Also, do not do this for prefetch since it does not have an external caller waiting on it.
			if (!p.callback_sent)
				ReadDoneCallback(systemID, p.orig_addr, currentClockCycle);
			
			// Erase the page from the pending set.
			// Note: the if statement is needed to ensure that the VictimRead operation (if it was invoked as part of a cache miss)
			// is already complete. If not, the pending_set removal will be done in VictimReadFinish().
			contention_unlock(p.back_addr, p.orig_addr, "CACHE_READ", p.victim_valid, victim_address, true, p.cache_addr);
		}		
	}

	void HybridSystem::CacheWrite(uint64_t orig_addr, uint64_t back_addr, uint64_t cache_addr)
	{
		if (DEBUG_CACHE)
			cerr << currentClockCycle << ": " << "Performing CACHE_WRITE for (" << back_addr << ", " << cache_addr << ")\n";

		// Compute the actual CACHE address of the data word we care about.
		uint64_t data_addr = cache_addr + PAGE_OFFSET(back_addr);
		
		Transaction t = Transaction(DATA_WRITE, data_addr, NULL);
		cache_queue.push_back(t);

		// Finish the operation by updating cache state, doing the callback, and removing the pending set.
		// Note: This is only split up so the LineWrite operation can reuse the second half
		// of CacheWrite without actually issuing a new write.
		Pending p;
		p.orig_addr = orig_addr;
		p.back_addr = back_addr;
		p.cache_addr = cache_addr;
		p.victim_tag = 0;
		p.victim_valid = false;
		p.callback_sent = false;
		p.type = DATA_WRITE;

		CacheWriteFinish(p);
	}

	//void HybridSystem::CacheWriteFinish(uint64_t orig_addr, uint64_t back_addr, uint64_t cache_addr, bool callback_sent)
	void HybridSystem::CacheWriteFinish(Pending p)
	{
		// Update the cache state
		cache_line cur_line = cache[p.cache_addr];
		cur_line.dirty = true;
		cur_line.valid = true;
		if ((cur_line.prefetched) && (cur_line.used == false)) // Note: this if statement must come before cur_line.used is set to true.
			unused_prefetches--;
		cur_line.used = true;
		cur_line.ts = currentClockCycle;
		cur_line.access_count++;
		cache[p.cache_addr] = cur_line;

		if (DEBUG_CACHE)
			cerr << cur_line.str() << endl;

		// Call the top level callback.
		// This is done immediately rather than waiting for callback.
		// Only do this if it hasn't been sent already by the critical cache line first callback.
		if (!p.callback_sent)
			WriteDoneCallback(systemID, p.orig_addr, currentClockCycle);

		// Erase the page from the pending set.
		// Note: the if statement is needed to ensure that the VictimRead operation (if it was invoked as part of a cache miss)
		// is already complete. If not, the pending_set removal will be done in VictimReadFinish().
		uint64_t victim_address = BACK_ADDRESS(p.victim_tag, SET_INDEX(p.back_addr));
		contention_unlock(p.back_addr, p.orig_addr, "CACHE_WRITE", p.victim_valid, victim_address, true, p.cache_addr);
	}

	
	void HybridSystem::Flush(uint64_t cache_addr)
	{
		// The flush transaction simply sets the timestamp of the current cache line to 0.
		// This forces the next miss in this set to remove this line.
		// Note: Flush does not actually cause a write to happen.

		// Update the cache state
		cache_line cur_line = cache[cache_addr];
		cur_line.ts = 0;
		cur_line.access_count = 0;
		cache[cache_addr] = cur_line;

		uint64_t set_index = SET_INDEX(cache_addr);
		uint64_t back_address = BACK_ADDRESS(cur_line.tag, set_index);
		contention_unlock(back_address, back_address, "FLUSH", false, 0, true, cache_addr);
	}

        uint64_t HybridSystem::VictimSelect(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        if(replacementPolicy == lru)
		        return LRUVictim(set_index, addr, cur_address, cur_line, set_address_list);
		else if(replacementPolicy == nru)
			return NRUVictim(set_index, addr, cur_address, cur_line, set_address_list);
	        else if(replacementPolicy == lfu)
		        return LFUVictim(set_index, addr, cur_address, cur_line, set_address_list);
	        else if(replacementPolicy == cflru)
		        return CFLRUVictim(set_index, addr, cur_address, cur_line, set_address_list);
	        else if(replacementPolicy == cflfu)
		        return CFLFUVictim(set_index, addr, cur_address, cur_line, set_address_list);
		else if(replacementPolicy == random)
		        return RandomVictim(set_index, addr, cur_address, cur_line, set_address_list);

		else
		        return LRUVictim(set_index, addr, cur_address, cur_line, set_address_list);
        }

        uint64_t HybridSystem::LRUVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());
		uint64_t min_ts = (uint64_t) 18446744073709551615U; // Max uint64_t
		bool min_init = false;

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "scanning set address list...\n\n";
		}

		uint64_t victim_counter = 0;
		uint64_t victim_set_offset = 0;
		for (list<uint64_t>::iterator it=set_address_list.begin(); it != set_address_list.end(); it++)
		{
			cur_address = *it;
			cur_line = cache[cur_address];

			if (DEBUG_VICTIM)
			{
				debug_victim << "cur_address= 0x" << hex << cur_address << dec << "\n";
				debug_victim << "cur_tag= " << cur_line.tag << "\n";
				debug_victim << "dirty= " << cur_line.dirty << "\n";
				debug_victim << "valid= " << cur_line.valid << "\n";
				debug_victim << "ts= " << cur_line.ts << "\n";
				debug_victim << "min_ts= " << min_ts << "\n\n";
			}

			// If the current line is the least recent we've seen so far, then select it.
			// But do not select it if the line is locked.
			if (((cur_line.ts < min_ts) || (!min_init)) && (!cur_line.locked))
			{
				victim = cur_address;	
				min_ts = cur_line.ts;
				min_init = true;

				victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW MINIMUM!\n\n";
				}
			}

			victim_counter++;
			
		}

		if (DEBUG_VICTIM)
		{
			debug_victim << "Victim in set_offset: " << victim_set_offset << "\n\n";
		}
		
		return victim;
	}

	uint64_t HybridSystem::NRUVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());
		uint64_t min_ts = (uint64_t) 18446744073709551615U; // Max uint64_t
		bool min_init = false;

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "scanning set address list...\n\n";
		}

		uint64_t victim_counter = 0;
		uint64_t victim_set_offset = 0;
		for (list<uint64_t>::iterator it=set_address_list.begin(); it != set_address_list.end(); it++)
		{
			cur_address = *it;
			cur_line = cache[cur_address];

			if (DEBUG_VICTIM)
			{
				debug_victim << "cur_address= 0x" << hex << cur_address << dec << "\n";
				debug_victim << "cur_tag= " << cur_line.tag << "\n";
				debug_victim << "dirty= " << cur_line.dirty << "\n";
				debug_victim << "valid= " << cur_line.valid << "\n";
				debug_victim << "ts= " << cur_line.ts << "\n";
				debug_victim << "min_ts= " << min_ts << "\n\n";
			}

			// If the current line is the least recent we've seen so far, then select it.
			// But do not select it if the line is locked.
			if (((cur_line.ts < min_ts) || (!min_init)) && (!cur_line.locked))
			{
				victim = cur_address;	
				min_ts = cur_line.ts;
				min_init = true;

				victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW MINIMUM!\n\n";
				}
			}

			victim_counter++;
			
		}

		if (DEBUG_VICTIM)
		{
			debug_victim << "Victim in set_offset: " << victim_set_offset << "\n\n";
		}
		
		return victim;
	}

        uint64_t HybridSystem::LFUVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());
		uint64_t min_access_count = (uint64_t) 18446744073709551615U; // Max uint64_t
		bool min_init = false;

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "scanning set address list...\n\n";
		}

		uint64_t victim_counter = 0;
		uint64_t victim_set_offset = 0;
		for (list<uint64_t>::iterator it=set_address_list.begin(); it != set_address_list.end(); it++)
		{
			cur_address = *it;
			cur_line = cache[cur_address];

			if (DEBUG_VICTIM)
			{
				debug_victim << "cur_address= 0x" << hex << cur_address << dec << "\n";
				debug_victim << "cur_tag= " << cur_line.tag << "\n";
				debug_victim << "dirty= " << cur_line.dirty << "\n";
				debug_victim << "valid= " << cur_line.valid << "\n";
				debug_victim << "ts= " << cur_line.ts << "\n";
				debug_victim << "access_count= " << cur_line.access_count << "\n";
				debug_victim << "min_access_count= " << min_access_count << "\n\n";
			}

			// If the current line is the least recent we've seen so far, then select it.
			// But do not select it if the line is locked.
			if (((cur_line.access_count < min_access_count) || (!min_init)) && (!cur_line.locked))
			{
				victim = cur_address;	
				min_access_count = cur_line.access_count;
				min_init = true;

				victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW MINIMUM!\n\n";
				}
			}

			victim_counter++;
			
		}

		if (DEBUG_VICTIM)
		{
			debug_victim << "Victim in set_offset: " << victim_set_offset << "\n\n";
		}
		
		return victim;
        }

        uint64_t HybridSystem::CFLRUVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());		
		uint64_t min_ts = (uint64_t) 18446744073709551615U; // Max uint64_t
		bool min_init = false;

		// because there might not always be a clean page
		uint64_t dirty_victim = *(set_address_list.begin());
		uint64_t dirty_min_ts = (uint64_t) 18446744073709551615U; // Max uint64_t

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "scanning set address list...\n\n";
		}

		uint64_t victim_counter = 0;
		uint64_t victim_set_offset = 0;
		uint64_t dirty_victim_set_offset = 0;
		for (list<uint64_t>::iterator it=set_address_list.begin(); it != set_address_list.end(); it++)
		{
			cur_address = *it;
			cur_line = cache[cur_address];

			if (DEBUG_VICTIM)
			{
				debug_victim << "cur_address= 0x" << hex << cur_address << dec << "\n";
				debug_victim << "cur_tag= " << cur_line.tag << "\n";
				debug_victim << "dirty= " << cur_line.dirty << "\n";
				debug_victim << "valid= " << cur_line.valid << "\n";
				debug_victim << "ts= " << cur_line.ts << "\n";
				debug_victim << "min_ts= " << min_ts << "\n\n";
			}

			// If the current line is the least recent we've seen so far, then select it.
			// But do not select it if the line is locked.
			if ((((cur_line.ts < min_ts) && (!cur_line.dirty)) || (!min_init)) && (!cur_line.locked))
			{
				victim = cur_address;	
				min_ts = cur_line.ts;
				min_init = true;

				victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW MINIMUM!\n\n";
				}
			}
			// see if this good besides it being dirty
			else if (((cur_line.ts < dirty_min_ts) || (!min_init)) && (!cur_line.locked))
			{
				dirty_victim = cur_address;
				dirty_min_ts = cur_line.ts;

				dirty_victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW DIRTY MINIMUM!\n\n";
				}
			}

			victim_counter++;
			
		}

		
		// now we see if we actually found a clean page to replace, if the clean victim is the same as the starting page
		// then we know that we should consider the dirty min instead
		if(victim == *(set_address_list.begin()) && cache[victim].dirty)
		{
			if (DEBUG_VICTIM)
			{
				debug_victim << "Victim in set_offset: " << dirty_victim_set_offset << "\n\n";
			}

			return dirty_victim;
		}
		else
		{
			if (DEBUG_VICTIM)
			{
				debug_victim << "Victim in set_offset: " << victim_set_offset << "\n\n";
			}

			return victim;
		}

		abort();
		return 0; // should never get here
	}

        uint64_t HybridSystem::CFLFUVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());
		uint64_t min_access_count = (uint64_t) 18446744073709551615U; // Max uint64_t
		bool min_init = false;

		// because there might not always be a clean page
		uint64_t dirty_victim = *(set_address_list.begin());
		uint64_t dirty_min_access_count = (uint64_t) 18446744073709551615U; // Max uint64_t

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "scanning set address list...\n\n";
		}

		uint64_t victim_counter = 0;
		uint64_t victim_set_offset = 0;
		uint64_t dirty_victim_set_offset = 0;
		for (list<uint64_t>::iterator it=set_address_list.begin(); it != set_address_list.end(); it++)
		{
			cur_address = *it;
			cur_line = cache[cur_address];

			if (DEBUG_VICTIM)
			{
				debug_victim << "cur_address= 0x" << hex << cur_address << dec << "\n";
				debug_victim << "cur_tag= " << cur_line.tag << "\n";
				debug_victim << "dirty= " << cur_line.dirty << "\n";
				debug_victim << "valid= " << cur_line.valid << "\n";
				debug_victim << "ts= " << cur_line.ts << "\n";
				debug_victim << "access_count= " << cur_line.access_count << "\n";
				debug_victim << "min_access_count= " << min_access_count << "\n\n";
			}

			// If the current line is the least recent we've seen so far, then select it.
			// But do not select it if the line is locked.
			if ((((cur_line.access_count < min_access_count) && (!cur_line.dirty)) || (!min_init)) && (!cur_line.locked))
			{
				victim = cur_address;	
				min_access_count = cur_line.access_count;
				min_init = true;

				victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW MINIMUM!\n\n";
				}
			}
			// see if this good besides it being dirty
			else if (((cur_line.access_count < dirty_min_access_count) || (!min_init)) && (!cur_line.locked))
			{
				dirty_victim = cur_address;
				dirty_min_access_count = cur_line.access_count;

				dirty_victim_set_offset = victim_counter;
				if (DEBUG_VICTIM)
				{
					debug_victim << "FOUND NEW DIRTY MINIMUM!\n\n";
				}
			}

			victim_counter++;
			
		}

		// now we see if we actually found a clean page to replace, if the clean victim is the same as the starting page
		// then we know that we should consider the dirty min instead
		if(victim == *(set_address_list.begin()) && cache[victim].dirty)
		{
			if (DEBUG_VICTIM)
			{
				debug_victim << "Victim in set_offset: " << dirty_victim_set_offset << "\n\n";
			}

			return dirty_victim;
		}
		else
		{
			if (DEBUG_VICTIM)
			{
				debug_victim << "Victim in set_offset: " << victim_set_offset << "\n\n";
			}

			return victim;
		}
		abort();
		return 0; // should never get here
        }

	uint64_t HybridSystem::RandomVictim(uint64_t set_index, uint64_t addr, uint64_t cur_address, cache_line cur_line, list<uint64_t> set_address_list)
        {
	        uint64_t victim = *(set_address_list.begin());
		uint64_t min_access_count = (uint64_t) 18446744073709551615U; // Max uint64_t

		if (DEBUG_VICTIM)
		{
			debug_victim << "--------------------------------------------------------------------\n";
			debug_victim << currentClockCycle << ": new miss. time to pick the unlucky line.\n";
			debug_victim << "set: " << set_index << "\n";
			debug_victim << "new back addr: 0x" << hex << addr << dec << "\n";
			debug_victim << "new tag: " << TAG(addr)<< "\n";
			debug_victim << "selecting random victim...\n\n";
		}

		// making use of a lot of standard library algorithms here to avoid random selection bias
		list<uint64_t>::iterator it = set_address_list.begin();
		std::uniform_int_distribution<uint64_t> set_list_dist(0, set_address_list.size()-1);
		std::default_random_engine rand_gen;

		// might have to do this multiple times to fine a line that is not locked
		bool done = false;
		while(!done)
		{
			advance(it, set_list_dist(rand_gen));
			victim = *it;
			cur_line = cache[victim];
			if(!cur_line.locked)
				done = true;
		}

		if (DEBUG_VICTIM)
		{
			debug_victim << "victim= 0x" << hex << victim << dec << "\n";
			debug_victim << "cur_tag= " << cur_line.tag << "\n";
			debug_victim << "dirty= " << cur_line.dirty << "\n";
			debug_victim << "valid= " << cur_line.valid << "\n";
			debug_victim << "ts= " << cur_line.ts << "\n";
			debug_victim << "access_count= " << cur_line.access_count << "\n";
			debug_victim << "min_access_count= " << min_access_count << "\n\n";
		}
		
		return victim;
        }

	void HybridSystem::RegisterCallbacks( TransactionCompleteCB *readDone, TransactionCompleteCB *writeDone)
	{  
		// Save the external callbacks.
		ReadDone = readDone;
		WriteDone = writeDone;
	}


    void HybridSystem::CacheReadCallback(uint64_t id, uint64_t addr, uint64_t cycle,  bool unmapped)
	{
		// Determine which address to look up in the pending table.
		// addr is cache_addr
		if (cache_pending.count(addr) != 0)
		{
			// Get the pending object for this transaction.
			Pending p = cache_pending[addr];

			// Remove this pending object from cache_pending
			cache_pending.erase(addr);
			assert(cache_pending.count(addr) == 0);

			if (p.op == VICTIM_READ)
			{
				VictimReadFinish(addr, p);
				cache_pending_set.erase(addr);
			}
			else if (p.op == CACHE_READ)
			{
				CacheReadFinish(addr, p, 0);
				cache_pending_set.erase(addr);
			}
			else if (p.op == TAG_READ || p.op == TAG_PREFETCH)
			{
				cache_pending_set.erase(addr);
				CacheReadFinish(addr, p, 0);
			}
			else
			{
				ERROR("CACHEReadCallback received an invalid op.");
				abort();
			}
		}
		else
		{
			ERROR("CACHEReadCallback received an address not in the pending set.");
			abort();
		}
	}

    void HybridSystem::CacheWriteCallback(uint64_t id, uint64_t addr, uint64_t cycle,  bool unmapped)
	{
		// Nothing to do (it doesn't matter when the CACHE write finishes for the cache controller, as long as it happens).
		cache_pending_set.erase(addr);
	}

	void HybridSystem::BackReadCallback(uint id, uint64_t addr, uint64_t cycle)
	{
		if (back_pending.count(PAGE_ADDRESS(addr)) != 0)
		{
			// Get the pending object.
			Pending p = back_pending[PAGE_ADDRESS(addr)];

			// Remove this pending object from back_pending
			back_pending.erase(PAGE_ADDRESS(addr));

			if (p.op == LINE_READ)
			{
				LineReadFinish(addr, p);
			}
			else
			{
				ERROR("BackReadCallback received an invalid op.");
				abort();
			}
		}
		else
		{
			ERROR("BackReadCallback received an address not in the pending set.");
			cerr << "back_pending count was " << back_pending.count(PAGE_ADDRESS(addr)) << "\n";
			cerr << "address: " << addr << " page: " << PAGE_ADDRESS(addr) << " set: " << SET_INDEX(addr) << "\n";
			abort();
		}
	}

	void HybridSystem::CacheCriticalLineCallback(uint64_t id, uint64_t addr, uint64_t cycle, bool unmapped)
	{
		// This function is called to implement critical line first for reads.
		// This allows HybridSim to tell the external user it can make progress as soon as the data
		// it is waiting for is back in the memory controller.

		//cerr << cycle << ": Critical Line Callback Received for address " << addr << "\n";

		if (back_pending.count(PAGE_ADDRESS(addr)) != 0)
		{
			// Get the pending object.
			Pending p = back_pending[PAGE_ADDRESS(addr)];

			// Note: DO NOT REMOVE THIS FROM THE PENDING SET.


			if (p.op == LINE_READ)
			{
				if (p.callback_sent)
				{
					ERROR("BackCriticalLineCallback called twice on the same pending item.");
					abort();
				}
					
				// Make the callback and mark it as being called.
				if (p.type == DATA_READ)
					ReadDoneCallback(systemID, p.orig_addr, currentClockCycle);
				else if(p.type == DATA_WRITE)
					WriteDoneCallback(systemID, p.orig_addr, currentClockCycle);
				else
				{
					// Do nothing because this is a PREFETCH.
				}

				// Mark the pending item's callback as being sent so it isn't sent again later.
				p.callback_sent = true;

				back_pending[PAGE_ADDRESS(addr)] = p;
			}
			else
			{
				ERROR("BackCriticalLineCallback received an invalid op.");
				abort();
			}
		}
		else
		{
			ERROR("BackCriticalLineCallback received an address not in the pending set.");
			abort();
		}

	}

	void HybridSystem::BackWriteCallback(uint id, uint64_t addr, uint64_t cycle)
	{
		// Nothing to do (it doesn't matter when the back write finishes for the cache controller, as long as it happens).

		if (DEBUG_CACHE)
			cerr << "The write to Back line " << PAGE_ADDRESS(addr) << " has completed.\n";
	}



	void HybridSystem::ReadDoneCallback(uint sysID, uint64_t orig_addr, uint64_t cycle)
	{
		if (ReadDone != NULL)
		{
			uint64_t callback_addr = orig_addr;
			if (REMAP_MMIO)
			{
				if (orig_addr >= THREEPOINTFIVEGB)
				{
					// Give the same address in the callback that we originally received.
					callback_addr += HALFGB;
				}
			}

			// Call the callback.
			(*ReadDone)(sysID, callback_addr, cycle);
		}

		// Finish the logging for this access.
		if (ENABLE_LOGGER)
			log.access_stop(orig_addr);
	}


	void HybridSystem::WriteDoneCallback(uint sysID, uint64_t orig_addr, uint64_t cycle)
	{
		if (WriteDone != NULL)
		{
			uint64_t callback_addr = orig_addr;
			if (REMAP_MMIO)
			{
				if (orig_addr >= THREEPOINTFIVEGB)
				{
					// Give the same address in the callback that we originally received.
					callback_addr += HALFGB;
				}
			}

			// Call the callback.
			(*WriteDone)(sysID, callback_addr, cycle);
		}

		// Finish the logging for this access.
		if (ENABLE_LOGGER)
			log.access_stop(orig_addr);
	}

	void HybridSystem::reportPower()
	{
		// TODO: Remove this funnction from the external API.
	}



	string HybridSystem::SetOutputFileName(string tracefilename) { return ""; }

	void HybridSystem::printLogfile()
	{
		// Save the cache table if necessary.
		saveCacheTable();

		cerr << "TLB Misses: " << tlb_misses << "\n";
		cerr << "TLB Hits: " << tlb_hits << "\n";
		cerr << "Total prefetches: " << total_prefetches << "\n";
		cerr << "Unused prefetches in cache: " << unused_prefetches << "\n";
		cerr << "Unused prefetch victims: " << unused_prefetch_victims << "\n";
		cerr << "Prefetch hit NOPs: " << prefetch_hit_nops << "\n";

		if (ENABLE_STREAM_BUFFER)
		{
			cerr << "Unique one misses: " << unique_one_misses << "\n";
			cerr << "Unique stream buffers: " << unique_stream_buffers << "\n";
			cerr << "Stream buffers hits: " << stream_buffer_hits << "\n";
		}

		// Print out the log file.
		if (ENABLE_LOGGER)
		{
			log.print();
		
			// Tell NVDIMM to print logs now
			llcache->saveStats();

			if(DEBUG_COMBO_TAG)
			{
				tbuff.printBufferUsage();
			}
		}
	}


	void HybridSystem::restoreCacheTable()
	{
		if (PREFILL_CACHE)
		{
			// Fill the cache table.
			for (uint64_t i=0; i<ACTUAL_CACHE_PAGES; i++)
			{
				uint64_t cache_addr = i*PAGE_SIZE;
				cache_line line;

				line.valid = true;
				line.dirty = PREFILL_CACHE_DIRTY;
				line.locked = false;
				line.tag = TAG(cache_addr);
				line.data = 0;
				line.ts = 0;

				// Put this in the cache.
				cache[cache_addr] = line;
			}
		}

		if (ENABLE_RESTORE)
		{
			cerr << "PERFORMING RESTORE OF CACHE TABLE!!!\n";

			ifstream inFile;
			confirm_directory_exists("state"); // Assumes using state directory, otherwise the user is on their own.
			inFile.open(HYBRIDSIM_RESTORE_FILE);
			if (!inFile.is_open())
			{
				cerr << "ERROR: Failed to load HybridSim's state restore file: " << HYBRIDSIM_RESTORE_FILE << "\n";
				abort();
			}

			uint64_t tmp;

			// Read the parameters and confirm that they are the same as the current HybridSystem instance.
			inFile >> tmp;
			if (tmp != PAGE_SIZE)
			{
				cerr << "ERROR: Attempted to restore state and PAGE_SIZE does not match in restore file and ini file."  << "\n";
				abort();
			}
			inFile >> tmp;
			if (tmp != SET_SIZE)
			{
				cerr << "ERROR: Attempted to restore state and SET_SIZE does not match in restore file and ini file."  << "\n";
				abort();
			}
			inFile >> tmp;
			if (tmp != ACTUAL_CACHE_PAGES)
			{
				cerr << "ERROR: Attempted to restore state and ACTUAL_CACHE_PAGES does not match in restore file and ini file."  << "\n";
				abort();
			}
			inFile >> tmp;
			if (tmp != TOTAL_PAGES)
			{
				cerr << "ERROR: Attempted to restore state and TOTAL_PAGES does not match in restore file and ini file."  << "\n";
				abort();
			}
				
			// Read the cache table.
			while(inFile.good())
			{
				uint64_t cache_addr;
				cache_line line;

				// Get the cache line data from the file.
				inFile >> cache_addr;
				inFile >> line.valid;
				inFile >> line.dirty;
				inFile >> line.tag;
				inFile >> line.data;
				inFile >> line.ts;

				if (RESTORE_CLEAN)
				{
					line.dirty = 0;
				}

				// The line must not be locked on restore.
				// This is a point of weirdness with the replay warmup design (since we can't restore the system
				// exactly as it was), but it is unavoidable. In flight transactions are simply lost. Although, if
				// replay warmup is done right, the system should run until all transactions are processed.
				line.locked = false;

				// Put this in the cache.
				cache[cache_addr] = line;
			}
		
			inFile.close();

			llcache->loadNVState(NVDIMM_RESTORE_FILE);
		}
	}

	void HybridSystem::saveCacheTable()
	{
		if (ENABLE_SAVE)
		{
			ofstream savefile;
			confirm_directory_exists("state"); // Assumes using state directory, otherwise the user is on their own.
			savefile.open(HYBRIDSIM_SAVE_FILE, ios_base::out | ios_base::trunc);
			if (!savefile.is_open())
			{
				cerr << "ERROR: Failed to load HybridSim's state save file: " << HYBRIDSIM_SAVE_FILE << "\n";
				abort();
			}
			cerr << "PERFORMING SAVE OF CACHE TABLE!!!\n";

			savefile << PAGE_SIZE << " " << SET_SIZE << " " << ACTUAL_CACHE_PAGES << " " << TOTAL_PAGES << "\n";

			for (uint64_t i=0; i < ACTUAL_CACHE_PAGES; i++)
			{
				uint64_t cache_addr= i * PAGE_SIZE;

				if (cache.count(cache_addr) == 0)
					// Skip to next page if this cache_line entry is not in the cache table.
					continue;

				// Get the line entry.
				cache_line line = cache[cache_addr];

				if (!line.valid)
					// If the line isn't valid, then don't need to save it.
					continue;
				
				savefile << cache_addr << " " << line.valid << " " << line.dirty << " " << line.tag << " " << line.data << " " << line.ts << "\n";
			}

			savefile.close();

			llcache->saveNVState(NVDIMM_SAVE_FILE);
		}
	}



	// Page Contention functions
	void HybridSystem::contention_lock(uint64_t back_addr)
	{
		pending_back_addr[back_addr] = 0;
	}

	void HybridSystem::contention_page_lock(uint64_t back_addr)
	{
		// Add to the pending pages map. And set the count to 0.
		pending_pages[PAGE_ADDRESS(back_addr)] = 0;
	}

	void HybridSystem::contention_unlock(uint64_t back_addr, uint64_t orig_addr, string operation, bool victim_valid, uint64_t victim_page, 
			bool cache_line_valid, uint64_t cache_addr)
	{
		uint64_t page_addr = PAGE_ADDRESS(back_addr);

		// If there is no page entry, then this means only the back address was locked (i.e. it is a DRAM hit).
		if (pending_pages.count(page_addr) == 0)
		{
			int num = pending_back_addr.erase(back_addr);
			assert(num == 1);

			// Victim should never be valid if we were only servicing a cache hit.
			assert(victim_valid == false);

			// If the cache line is valid, unlock it.
			if (cache_line_valid)
				contention_cache_line_unlock(cache_addr);

			// Restart queue checking.
			this->check_queue = true;
			pending_count -= 1;

			return;
		}

		// At this point, we know that the page_addr is in the pending_pages list.
		// This implies we also know that there was a cache miss for this access.
		// If the count for the pending_pages entry is > 0, then we DO NOT unlock
		// the page yet.

		// Erase the page from the pending page map.
		// Note: the if statement is needed to ensure that the VictimRead operation (if it was invoked as part of a cache miss)
		// is already complete. If not, the pending_set removal will be done in VictimReadFinish().
		else if (pending_pages[PAGE_ADDRESS(back_addr)] == 0)
		{
			int num = pending_pages.erase(PAGE_ADDRESS(back_addr));
			if (num != 1)
			{
				cerr << "pending_pages.erase() was called after " << operation << " and num was 0.\n";
				cerr << "orig:" << orig_addr << " aligned:" << back_addr << "\n\n";
				abort();
			}

			// Also remove the pending_back_addr entry.
			num = pending_back_addr.erase(back_addr);
			assert(num == 1);

			// If the victim page is valid, then unlock it too.
			if (victim_valid)
				contention_victim_unlock(victim_page);

			// If the cache line is valid, unlock it.
			if (cache_line_valid)
				contention_cache_line_unlock(cache_addr);

			// Restart queue checking.
			this->check_queue = true;
			pending_count -= 1;
		}
	}

	bool HybridSystem::contention_is_unlocked(uint64_t back_addr)
	{
		uint64_t page_addr = PAGE_ADDRESS(back_addr);

		// First see if the set is locked. This is done by looking at the set_counter.
		// If the set counter exists and is equal to the set size, then we should NOT be trying to do any more accesses
		// to the set, because this means that all of the cache lines are locked.
		uint64_t set_index = SET_INDEX(page_addr);
		if (set_counter.count(set_index) > 0)
		{
			if (assocVersion != tag_tlb && set_counter[set_index] >= 1)
			{
				return false;
			}
			else if (set_counter[set_index] == SET_SIZE)
			{
				return false;
			}
		}
		
		// if we're using the combo tag associativity, make sure that there is not a currently outstanding access to the
		// tag page for this address
		
		if(assocVersion == combo_tag)
		{
			uint64_t data_address = getComboDataAddr(set_index, 0);
			uint64_t cache_address = getComboTagAddr(set_index, data_address);
			if(cache_pending.count(cache_address) != 0)
			{
				//cerr << "Issue blocked by tag lookup, this shouldn't happen often \n";
				return false;
			}
		}

		// If the page is not in the penting_pages and pending_back_addr map, then it is unlocked.
		if ((pending_pages.count(page_addr) == 0) && (pending_back_addr.count(back_addr) == 0))
			return true;
		else
			return false;
	}


	void HybridSystem::contention_increment(uint64_t back_addr)
	{
		uint64_t page_addr = PAGE_ADDRESS(back_addr);
		// TODO: Add somme error checking here (e.g. make sure page is in pending_pages and make sure count is >= 0)

		// This implements a counting semaphore for the page so that it isn't unlocked until the count is 0.
		pending_pages[page_addr] += 1;
	}

	void HybridSystem::contention_decrement(uint64_t back_addr)
	{
		uint64_t page_addr = PAGE_ADDRESS(back_addr);
		// TODO: Add somme error checking here (e.g. make sure page is in pending_pages and make sure count is >= 0)

		// This implements a counting semaphore for the page so that it isn't unlocked until the count is 0.
		pending_pages[page_addr] -= 1;
	}

	void HybridSystem::contention_victim_lock(uint64_t page_addr)
	{
		pending_pages[page_addr] = 0;
	}

	void HybridSystem::contention_victim_unlock(uint64_t page_addr)
	{
		int num = pending_pages.erase(page_addr);
		assert(num == 1);
	}

	void HybridSystem::contention_cache_line_lock(uint64_t cache_addr)
	{
		cache_line cur_line = cache[cache_addr];
		cur_line.locked = true;
		cur_line.lock_count++;
		cache[cache_addr] = cur_line;

		uint64_t set_index = SET_INDEX(cache_addr);
		if (set_counter.count(set_index) == 0)
			set_counter[set_index] = 1;
		else
			set_counter[set_index] += 1;
	}

	void HybridSystem::contention_cache_line_unlock(uint64_t cache_addr)
	{
		cache_line cur_line = cache[cache_addr];
		cur_line.lock_count--;
		assert(cur_line.lock_count >= 0);
		if (cur_line.lock_count == 0)
			cur_line.locked = false; // Only unlock if the count for outstanding accesses is 0.
		cache[cache_addr] = cur_line;

		uint64_t set_index = SET_INDEX(cache_addr);
		set_counter[set_index] -= 1;
	}

	// PREFETCHING FUNCTIONS
	void HybridSystem::issue_sequential_prefetches(uint64_t page_addr)
	{
		// Count down from the top address. This must be done because addPrefetch puts transactions at the front
		// of the queue and we want page_addr+PAGE_SIZE to be the first prefetch issued.
		for (int i=SEQUENTIAL_PREFETCHING_WINDOW; i > 0; i--)
		{
			// Compute the next prefetch address.
			uint64_t prefetch_address = page_addr + (i * PAGE_SIZE);

			// If address is above the legal address space for the main memory, then do not issue this prefetch.
			if (prefetch_address >= (TOTAL_PAGES * PAGE_SIZE))
				continue;

			// Add the prefetch.
			addPrefetch(prefetch_address);
			//cerr << currentClockCycle << ": Prefetcher adding " << prefetch_address << " to transaction queue.\n";
		}
	}


	void HybridSystem::sync(uint64_t addr, uint64_t cache_address, Transaction trans)
	{
		// The SYNC command works by reusing the VictimRead infrastructure. This works pretty well
		// except we have to be careful because that code was originaly designed to work when a miss 
		// had occurred. SYNC only happens when there is a hit.

		// TODO: Abtract this code into a common function with the miss path (if possible).

		cache_line cur_line = cache[cache_address];

		uint64_t victim_back_addr = BACK_ADDRESS(cur_line.tag, SET_INDEX(cache_address));

		// The address in the cache line should be the SAME as the address we are syncing on.
		assert(victim_back_addr == addr);

		// Lock the cache line so no one else tries to use it while this miss is being serviced.
		contention_cache_line_lock(cache_address);
	
		Pending p;
		p.orig_addr = trans.address;
		p.back_addr = addr;
		p.cache_addr = cache_address;
		p.victim_tag = cur_line.tag;
		p.victim_valid = false; // MUST SET THIS TO FALSE SINCE SYNC PAGE AND VICTIM PAGE MATCH.
		p.callback_sent = false;
		p.type = trans.transactionType;

		// The line MUST be dirty for a sync operation to be valid.
		assert(cur_line.dirty);

		VictimRead(p);

		// Mark the line clean (since this is the whole point of SYNC).
		cur_line.dirty = false;
		cache[cache_address] = cur_line;
	}


	void HybridSystem::syncAllCounter(uint64_t addr, Transaction trans)
	{
		//cout << "Processing SYNC_ALL_COUNTER " << addr << "\n";
		uint64_t next_addr = addr + PAGE_SIZE;

		//cout << "next_addr = " << next_addr << endl;
		if (next_addr < (ACTUAL_CACHE_PAGES * PAGE_SIZE))
		{
			// Issue SYNC_ALL_COUNTER transaction to next_addr.
			// This is what iterates through all lines.
			// Note: This must be done BEFORE the SYNC is added for the current line so 
			// it is placed after the SYNC in the queue with add_front().
			addSyncCounter(next_addr, false);
		}

		// Look up cache line.
		if (cache.count(addr) == 0)
		{
			// If i is not allocated yet, allocate it.
			cache[addr] = cache_line();
		}
		cache_line cur_line = cache[addr];

		if (cur_line.valid && cur_line.dirty)
		{
			// Compute back address.
			uint64_t back_addr = BACK_ADDRESS(cur_line.tag, SET_INDEX(addr));

			// Issue sync command for back address.
			addSync(back_addr);
			//cout << "Added sync for address " << back_addr << endl;
		}

		// Unlock the page and return.
		contention_unlock(addr, trans.address, "SYNC_ALL_COUNTER", false, 0, false, 0);
	}


	void HybridSystem::addSync(uint64_t addr)
	{
		// Create flush transaction.
		Transaction t = Transaction(SYNC, addr, NULL);

		// Push the operation onto the front of the transaction queue so it stays at the front.
		trans_queue.push_front(t);

		trans_queue_size += 1;

		pending_count += 1;

		// Restart queue checking.
		this->check_queue = true;

	}


	void HybridSystem::addSyncCounter(uint64_t addr, bool initial)
	{
		// Create flush transaction.
		Transaction t = Transaction(SYNC_ALL_COUNTER, addr, NULL);

		if (initial)
		{
			// The initial SYNC_ALL_COUNTER operation must wait to get to the front of the queue.
			trans_queue.push_back(t);
		}
		else
		{
			// Push the operation onto the front of the transaction queue so it stays at the front.
			trans_queue.push_front(t);
		}

		trans_queue_size += 1;

		pending_count += 1;

		// Restart queue checking.
		this->check_queue = true;
	}


	void HybridSystem::mmio(uint64_t operation, uint64_t address)
	{
		if (operation == 0)
		{
			// NOP
			cerr << "\n" << currentClockCycle << " : HybridSim received MMIO NOP.\n";
		}
		else if (operation == 1)
		{
			// SYNC_ALL
			cerr << "\n" << currentClockCycle << " : HybridSim received MMIO SYNC_ALL.\n";
			syncAll();
		}
		else if (operation == 2)
		{
			// TASK_SWITCH
			cerr << "\n" << currentClockCycle << " : HybridSim received MMIO TASK_SWITCH.\n";
			cerr << "\n" << "Address=" << address << " Accesses=" << log.num_accesses << " Reads=" << log.num_reads << " Writes=" << log.num_writes << "\n";
		}
		else if (operation == 3)
		{
			// PREFETCH_RANGE

			// Split address into lower 48 bits for base address and upper 16 bits for number of pages to prefetch.
			uint64_t base_address = address & 0x0000FFFFFFFFFFFF;
			uint64_t prefetch_pages = (address >> 48) & 0x000000000000FFFF;

			// Add one to prefetch_pages. This means 0 in the upper bits means prefetch a single page.
			// This allows the caller to prefetch up to 2^16 pages (256 MB when using 4k pages).
			prefetch_pages += 1;

			cerr << "\n" << currentClockCycle << " : HybridSim received MMIO PREFETCH_RANGE. ";
			cerr << "base_address=" << base_address << " prefetch_pages=" << prefetch_pages << "\n";

			for (uint64_t i=0; i<prefetch_pages; i++)
			{
				addPrefetch(base_address + i*PAGE_SIZE);
			}
		}
		else
		{
			cerr << "\n" << currentClockCycle << " : HybridSim received invalid MMIO operation.\n";
			abort();
		}
	}


	void HybridSystem::syncAll()
	{
		addSyncCounter(0, true);
	}

	void HybridSystem::check_tlb(uint64_t page_addr)
	{
		// A TLB_SIZE of 0 disables the TLB.
		// This means we always have the tags in SRAM on the CPU.
		if (TLB_SIZE == 0)
			return;

		// TLB processing code.
		uint64_t tlb_base_addr = TLB_BASE_ADDRESS(page_addr);
		if (tlb_base_set.count(tlb_base_addr) == 0)
		{
			//cerr << "TLB miss with address " << page_addr << ".\n";
			tlb_misses++;
			// TLB miss.
			if (tlb_base_set.size() == TLB_MAX_ENTRIES)
			{
				// TLB is full, so must pick a victim.
				uint64_t tlb_victim = (*(tlb_base_set.begin())).first;
				uint64_t tlb_victim_ts = (*(tlb_base_set.begin())).second;
				unordered_map<uint64_t, uint64_t>:: iterator tlb_it;
				for (tlb_it = tlb_base_set.begin(); tlb_it != tlb_base_set.end(); tlb_it++)
				{
					uint64_t cur_ts = (*tlb_it).second;
					if (cur_ts < tlb_victim_ts)
					{
						// Found an older entry than the current victim.
						tlb_victim = (*tlb_it).first;
						tlb_victim_ts = cur_ts;
					}
				}

				// Remove the victim entry.
				//cerr << "Evicting " << tlb_victim << " from TLB.\n";
				tlb_base_set.erase(tlb_victim);
			}

			// At this point, there is at least one empty spot in the TLB.
			assert(tlb_base_set.size() < TLB_MAX_ENTRIES);
			
			// Insert the new page with the current clock cycle.
			tlb_base_set[tlb_base_addr] = currentClockCycle;

			// Add TLB_MISS_DELAY to the controller delay.
			delay_counter += TLB_MISS_DELAY;
		}
		else
		{
			// TLB hit. Just update the timestamp for the LRU algorithm.
			//cerr << "TLB hit with address " << page_addr << ".\n";
			tlb_hits++;
			tlb_base_set[tlb_base_addr] = currentClockCycle;
		}
	}

	void HybridSystem::stream_buffer_miss_handler(uint64_t miss_page)
	{
		// Don't do any miss processing for the first or last pages in memory.
		if ((miss_page == 0) || (miss_page == (TOTAL_PAGES - 1)*PAGE_SIZE))
			return;

		// Calculate the neighboring pages.
		uint64_t prior_page = miss_page - PAGE_SIZE;
		uint64_t next_page = miss_page + PAGE_SIZE;

		// Look through the one miss table to see if there are any matches.
		bool stream_detected = false;
		list<pair<uint64_t, uint64_t> >::iterator it;
		for (it=one_miss_table.begin(); it != one_miss_table.end(); it++)
		{
			uint64_t entry_page = (*it).first;
			//uint64_t entry_cycle = (*it).second;

			if (entry_page == miss_page)
			{
				// Somehow we managed to miss the same page twice in a short period of time.
				// Go ahead and remove this entry to make room for this to be readded.
				it = one_miss_table.erase(it);

				if (DEBUG_STREAM_BUFFER)
					cerr << currentClockCycle << " : Stream buffer one miss double hit. miss_page=" << miss_page << "\n";
			}
			if (entry_page == prior_page)
			{
				// Stream detected!
				stream_detected = true;

				if (DEBUG_STREAM_BUFFER)
					cerr << currentClockCycle << " : New stream detected. Allocating stream buffer at addr " << next_page << "\n";


				// Remove the entry for the prior page.
				it = one_miss_table.erase(it);

				// Save the next page in the stream buffer table
				// This is the address we will detect on a hit to the buffer.
				stream_buffers[next_page] = currentClockCycle;
				unique_stream_buffers++;

				if (stream_buffers.size() > NUM_STREAM_BUFFERS)
				{
					// Evict stream buffer with oldest cycle.
					unordered_map<uint64_t, uint64_t>::iterator sb_it;
					uint64_t oldest_key = 0;
					uint64_t oldest_cycle = currentClockCycle;
					for (sb_it=stream_buffers.begin(); sb_it != stream_buffers.end(); sb_it++)
					{
						uint64_t cur_sb_cycle = (*sb_it).second;
						if (cur_sb_cycle < oldest_cycle)
						{
							oldest_key = (*sb_it).first;
							oldest_cycle = cur_sb_cycle;
						}
					}
					stream_buffers.erase(oldest_key);

					if (DEBUG_STREAM_BUFFER)
						cerr << currentClockCycle << " : Stream buffer evicted addr=" << oldest_key << "\n";
				}

				// Issue prefetches to start the stream.
				// Count down from the top address. This must be done because addPrefetch puts transactions at the front
				// of the queue and we want page_addr+PAGE_SIZE to be the first prefetch issued.
				for (int i=STREAM_BUFFER_LENGTH; i > 0; i--)
				{
					// Compute the next prefetch address.
					uint64_t prefetch_address = miss_page + (i * PAGE_SIZE);

					// If address is above the legal address space for the main memory, then do not issue this prefetch.
					if (prefetch_address >= (TOTAL_PAGES * PAGE_SIZE))
						continue;

					// Add the prefetch.
					addPrefetch(prefetch_address);
				}

				break;
			}
		}

		if (!stream_detected)
		{
			// Insert miss address into the one_miss_table.
			one_miss_table.push_back(make_pair(miss_page, currentClockCycle));
			unique_one_misses++;

			if (DEBUG_STREAM_BUFFER)
				cerr << currentClockCycle << " : One miss detected addr=" << miss_page << "\n";
		}

		// Enforce the size of the one miss table.
		if (one_miss_table.size() > ONE_MISS_TABLE_SIZE)
		{
			if (DEBUG_STREAM_BUFFER)
			{
				cerr << currentClockCycle << " : One miss evicted addr=" << one_miss_table.front().first << "\n";
			}

			one_miss_table.pop_front();
		}

	}

	void HybridSystem::stream_buffer_hit_handler(uint64_t hit_page)
	{
		if (stream_buffers.count(hit_page) == 1)
		{
			// Stream buffer hit!
			stream_buffer_hits++;

			uint64_t next_page = hit_page + PAGE_SIZE;
			uint64_t prefetch_address = hit_page + (STREAM_BUFFER_LENGTH * PAGE_SIZE);

			if ((DEBUG_STREAM_BUFFER==1) && (DEBUG_STREAM_BUFFER_HIT==1))
			{
				cerr << currentClockCycle << " : Stream Buffer Hit. hit_page=" << hit_page
					<< " prior cycle=" << stream_buffers[hit_page] << " next_page=" << next_page 
					<< " prefetch_addr=" << prefetch_address << "\n";
			}

			// Remove the current stream buffer entry and replace it with the next page.
			stream_buffers.erase(hit_page);

			// If the prefetch address is out of range, then do nothing.
			if (prefetch_address < (TOTAL_PAGES * PAGE_SIZE))
			{
				// If it is in range, add the prefetch and readd the stream buffer.
				addPrefetch(prefetch_address);
				stream_buffers[next_page] = currentClockCycle;
			}
		}

	}


// Extra functions for C interface (used by Python front end)
class HybridSim_C_Callbacks
{
	public:
	// Lists for tracking received callback data.
	list<uint> done_id;
	list<uint64_t> done_address;
	list<uint64_t> done_cycle;
	list<bool> done_isWrite;

	void read_complete(uint id, uint64_t address, uint64_t cycle)
	{
		done_id.push_back(id);
		done_address.push_back(address);
		done_cycle.push_back(cycle);
		done_isWrite.push_back(false);
	}

	void write_complete(uint id, uint64_t address, uint64_t cycle)
	{
		done_id.push_back(id);
		done_address.push_back(address);
		done_cycle.push_back(cycle);
		done_isWrite.push_back(true);
	}

	void register_cb(HybridSystem *hs)
	{
		typedef CallbackBase<void,uint,uint64_t,uint64_t> Callback_t;
		Callback_t *read_cb = new Callback<HybridSim_C_Callbacks, void, uint, uint64_t, uint64_t>(this, &HybridSim_C_Callbacks::read_complete);
		Callback_t *write_cb = new Callback<HybridSim_C_Callbacks, void, uint, uint64_t, uint64_t>(this, &HybridSim_C_Callbacks::write_complete);
		hs->RegisterCallbacks(read_cb, write_cb);
	}

	bool get_next_result(uint *sysID, uint64_t *addr, uint64_t *cycle, bool *isWrite)
	{
		if (done_id.empty())
		{
			*sysID = 0;
			*addr = 0;
			*cycle = 0;
			*isWrite = false;

			return false;
		}
		else
		{
			// Set the result pointers.
			*sysID = done_id.front();
			*addr = done_address.front();
			*cycle = done_cycle.front();
			*isWrite = done_isWrite.front();

			// Pop the front of each list.
			done_id.pop_front();
			done_address.pop_front();
			done_cycle.pop_front();
			done_isWrite.pop_front();

			return true;
		}
	}
};
HybridSim_C_Callbacks c_callbacks;

extern "C"
{
	HybridSystem *HybridSim_C_getMemorySystemInstance(uint id, char *ini)
	{
		// Note ini is implicitly transformed to C++ string type.
		HybridSystem *hs = getMemorySystemInstance(id, ini);

		// Register callbacks to the HybridSim_C_callbacks object.
		c_callbacks.register_cb(hs);

		return hs;
	}

	bool HybridSim_C_addTransaction(HybridSystem *hs, bool isWrite, uint64_t addr)
	{
		//cout << "C interface... addr=" << addr << " isWrite=" << isWrite << "\n";
		return hs->addTransaction(isWrite, addr);
	}

	bool HybridSim_C_WillAcceptTransaction(HybridSystem *hs)
	{
		return hs->WillAcceptTransaction();
	}

	void HybridSim_C_update(HybridSystem *hs)
	{
		hs->update();
	}

	// use this instead of the callbacks since I can't do callbacks through the Python cdll interface
	// The protocol is to call this repetitively after each update until it returns false.
	// When it returns false, the sysID, addr, cycle, and isWrite are don't cares
	// When it returns true, the output values are set to the completed transaction
	bool HybridSim_C_PollCompletion(HybridSystem *hs, uint *sysID, uint64_t *addr, uint64_t *cycle, bool *isWrite)
	{
		return c_callbacks.get_next_result(sysID, addr, cycle, isWrite);
	}

	void HybridSim_C_mmio(HybridSystem *hs, uint64_t operation, uint64_t address)
	{
		hs->mmio(operation, address);
	}

	void HybridSim_C_syncAll(HybridSystem *hs)
	{
		hs->syncAll();
	}

	void HybridSim_C_reportPower(HybridSystem *hs)
	{
		hs->reportPower();
	}

	void HybridSim_C_printLogfile(HybridSystem *hs)
	{
		hs->printLogfile();
	}

}

} // Namespace HybridSim

// Extra function needed for Sandia SST.
extern "C"
{
    void libhybridsim_is_present(void)
    {
	;
    }
}



