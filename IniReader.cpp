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

#include "IniReader.h"

// Define the globals read from the ini file here.
// Also provide default values here.

namespace HybridSim 
{

// Other constants
uint64_t CONTROLLER_DELAY = 2;

uint64_t ENABLE_LOGGER = 1;
uint64_t ENABLE_SET_CONFLICT_LOG = 0;
uint64_t ENABLE_SET_ACCESSES_LOG = 0;
uint64_t ENABLE_PAGES_USED_LOG = 0;
uint64_t ENABLE_CONTENTION_LOG = 0;
uint64_t ENABLE_MISSED_PAGE_LOG = 0;
uint64_t EPOCH_LENGTH = 200000;
uint64_t HISTOGRAM_BIN = 100;
uint64_t HISTOGRAM_MAX = 20000;
uint64_t CONFLICT_BIN = 25;
uint64_t CONFLICT_MAX = 500;
uint64_t ENABLE_REUSE_LOG = 0;
uint64_t REUSE_BIN = 25000;
uint64_t REUSE_MAX = 500000;

// these values are also specified in the ini file of the nvdimm but have a different name
uint64_t PAGE_SIZE = 4096; // in bytes, so divide this by 64 to get the number of DDR3 transfers per page

uint64_t SET_SIZE = 64; // associativity of cache

uint64_t BURST_SIZE = 64; // number of bytes in a single transaction, this means with PAGE_SIZE=1024, 16 transactions are needed
uint64_t BACK_BURST_SIZE = 4096; // number of bytes in a single flash transaction

// Number of pages total and number of pages in the cache
uint64_t TOTAL_PAGES = 2097152/4; // 2 GB
uint64_t CACHE_PAGES = 1048576/4; // 1 GB

// PaulMod: Associativity version
string ASSOC_VERSION;
AssocVersion assocVersion;

// PaulMod: Tag Buffer Stuff
uint64_t NUM_TAG_WAYS;
uint64_t NUM_TAG_SETS;
uint64_t SETS_PER_LINE;
uint64_t SETS_PER_TAG_GROUP;
uint64_t ENABLE_TAG_BUFFER;
uint64_t ENABLE_SET_CHANNEL_INTERLEAVE;
uint64_t ENABLE_TAG_PREFETCH;
uint64_t TAG_PREFETCH_WINDOW;
string TAG_REPLACEMENT;
TagReplacement tagReplacement;

// PaulMod: Values from the NVDIMM cache
uint64_t NUM_CHANNELS = 1;
uint64_t RANKS_PER_CHANNEL = 1;
uint64_t BANKS_PER_RANK = 1;
uint64_t ROWS_PER_BANK = 1;
uint64_t COL_PER_ROW = 1;

// PaulMod: Replacement Policy
string REPLACEMENT_POLICY;
ReplacementPolicy replacementPolicy;
uint64_t REPLACEMENT_PERIOD;

// Defined in marss memoryHierachy.cpp.
// Need to confirm this and make it more flexible later.
uint64_t CYCLES_PER_SECOND = 667000000;

// INI files
string dram_ini = "ini/DDR3_micron_8M_8B_x8_sg15.ini";
string nvdimm_ini = "ini/samsung_K9XXG08UXM(mod).ini";
string sys_ini = "ini/system.ini";

// Save/Restore options
uint64_t ENABLE_RESTORE = 0;
uint64_t ENABLE_SAVE = 0;
string HYBRIDSIM_RESTORE_FILE = "none";
string NVDIMM_RESTORE_FILE = "none";
string HYBRIDSIM_SAVE_FILE = "none";
string NVDIMM_SAVE_FILE = "none";


	void IniReader::read(string inifile)
	{
		ifstream inFile;
		char tmp[256];
		string tmp2;
		list<string> lines;

		inFile.open(inifile);
		if (!inFile.is_open())
		{
			cerr << "ERROR: Failed to load HybridSim's Ini file: " << inifile << "\n";
			abort();
		}

		while(!inFile.eof())
		{
			inFile.getline(tmp, 256);
			tmp2 = (string)tmp;

			// Filter comments out.
			size_t pos = tmp2.find("#");
			tmp2 = tmp2.substr(0, pos);

			// Strip whitespace from the ends.
			tmp2 = strip(tmp2);

			// Filter newlines out.
			if (tmp2.empty())
				continue;

			// Add it to the lines list.
			lines.push_back(tmp2);
		}
		inFile.close();

		list<string>::iterator it;
		for (it = lines.begin(); it != lines.end(); it++)
		{
			list<string> split_line = split((*it), "=", 2);

			if (split_line.size() != 2)
			{
				cerr << "ERROR: Parsing ini failed on line: " << (*it) << "\n";
				cerr << "There should be exactly one '=' per line\n";
				abort();
			}

			string key = split_line.front();
			string value = split_line.back();

			// Place the value into the appropriate global.
			if (key.compare("CONTROLLER_DELAY") == 0)
				convert_uint64_t(CONTROLLER_DELAY, value, key);
			else if (key.compare("ENABLE_LOGGER") == 0)
				convert_uint64_t(ENABLE_LOGGER, value, key);
			else if (key.compare("ENABLE_SET_CONFLICT_LOG") == 0)
				convert_uint64_t(ENABLE_SET_CONFLICT_LOG, value, key);
			else if (key.compare("ENABLE_SET_ACCESSES_LOG") == 0)
				convert_uint64_t(ENABLE_SET_ACCESSES_LOG, value, key);
			else if (key.compare("ENABLE_PAGES_USED_LOG") == 0)
				convert_uint64_t(ENABLE_PAGES_USED_LOG, value, key);
			else if (key.compare("ENABLE_CONTENTION_LOG") == 0)
				convert_uint64_t(ENABLE_CONTENTION_LOG, value, key);
			else if (key.compare("ENABLE_MISSED_PAGE_LOG") == 0)
				convert_uint64_t(ENABLE_MISSED_PAGE_LOG, value, key);
			else if (key.compare("EPOCH_LENGTH") == 0)
				convert_uint64_t(EPOCH_LENGTH, value, key);
			else if (key.compare("HISTOGRAM_BIN") == 0)
				convert_uint64_t(HISTOGRAM_BIN, value, key);
			else if (key.compare("HISTOGRAM_MAX") == 0)
				convert_uint64_t(HISTOGRAM_MAX, value, key);
			else if (key.compare("CONFLICT_BIN") == 0)
				convert_uint64_t(CONFLICT_BIN, value, key);
			else if (key.compare("CONFLICT_MAX") == 0)
				convert_uint64_t(CONFLICT_MAX, value, key);
			else if (key.compare("ENABLE_REUSE_LOG") == 0)
				convert_uint64_t(ENABLE_REUSE_LOG, value, key);
			else if (key.compare("REUSE_BIN") == 0)
				convert_uint64_t(REUSE_BIN, value, key);
			else if (key.compare("REUSE_MAX") == 0)
				convert_uint64_t(REUSE_MAX, value, key);
			else if (key.compare("PAGE_SIZE") == 0)
				convert_uint64_t(PAGE_SIZE, value, key);
			else if (key.compare("SET_SIZE") == 0)
				convert_uint64_t(SET_SIZE, value, key);
			else if (key.compare("BURST_SIZE") == 0)
				convert_uint64_t(BURST_SIZE, value, key);
			else if (key.compare("BACK_BURST_SIZE") == 0)
				convert_uint64_t(BACK_BURST_SIZE, value, key);
			else if (key.compare("TOTAL_PAGES") == 0)
				convert_uint64_t(TOTAL_PAGES, value, key);
			else if (key.compare("CACHE_PAGES") == 0)
				convert_uint64_t(CACHE_PAGES, value, key);
			else if (key.compare("ASSOC_VERSION") == 0)
			{
				if(value.compare("TAG_TLB") == 0)
				{
					assocVersion = tag_tlb;
				}
				else if(value.compare("DIRECT") == 0)
				{
					assocVersion = direct;
				}
				else if(value.compare("LOH") == 0)
				{
					assocVersion = loh;
				}
				else if(value.compare("COMBO_TAG") == 0)
				{
					assocVersion = combo_tag;
				}
				else if(value.compare("CHANNEL") == 0)
				{
					assocVersion = channel;
				}
				else
				{
					assocVersion = tag_tlb;
				}
			}
			else if (key.compare("NUM_TAG_WAYS") == 0)
				convert_uint64_t(NUM_TAG_WAYS, value, key);
			else if (key.compare("NUM_TAG_SETS") == 0)
				convert_uint64_t(NUM_TAG_SETS, value, key);
			else if (key.compare("SETS_PER_LINE") == 0)
				convert_uint64_t(SETS_PER_LINE, value, key);
			else if (key.compare("SETS_PER_TAG_GROUP") == 0)
				convert_uint64_t(SETS_PER_TAG_GROUP, value, key);
			else if (key.compare("ENABLE_TAG_BUFFER") == 0)
				convert_uint64_t(ENABLE_TAG_BUFFER, value, key);
			else if (key.compare("ENABLE_SET_CHANNEL_INTERLEAVE") == 0)
				convert_uint64_t(ENABLE_SET_CHANNEL_INTERLEAVE, value, key);
			else if (key.compare("ENABLE_TAG_PREFETCH") == 0)
				convert_uint64_t(ENABLE_TAG_PREFETCH, value, key);
			else if (key.compare("TAG_PREFETCH_WINDOW") == 0)
				convert_uint64_t(TAG_PREFETCH_WINDOW, value, key);
			else if (key.compare("TAG_REPLACEMENT") == 0)
			{
			        if(value.compare("LRU") == 0)
			        {
					tagReplacement = tag_lru;
			        }
				else if(value.compare("RU") == 0)
				{
					tagReplacement = tag_ru;
				}
				else if(value.compare("FIFO") == 0)
				{
					tagReplacement = tag_fifo;
				}
				else if(value.compare("RANDOM") == 0)
				{
					tagReplacement = tag_random;
				}
				else if(value.compare("MRU") == 0)
				{
					tagReplacement = tag_mru;
				}
				else if(value.compare("LRNU") == 0)
				{
					tagReplacement = tag_lrnu;
				}
				else
				{
					tagReplacement = tag_lru;
				}
			}
			else if (key.compare("REPLACEMENT_POLICY") == 0)
			{
			        if(value.compare("LRU") == 0)
			        {
					replacementPolicy = lru;
			        }
				else if(value.compare("NRU") == 0)
				{
					replacementPolicy = nru;
				}
				else if(value.compare("LFU") == 0)
				{
					replacementPolicy = lfu;
				}
				else if(value.compare("CFLRU") == 0)
				{
					replacementPolicy = cflru;
				}
				else if(value.compare("CFLFU") == 0)
				{
					replacementPolicy = cflfu;
				}
				else if(value.compare("Random") == 0)
				{
					replacementPolicy = random;
				}
				else if(value.compare("BIP") == 0)
				{
					replacementPolicy = bip;
				}
				else if(value.compare("DIP") == 0)
				{
					replacementPolicy = dip;
				}
				else if(value.compare("RRIP") == 0)
				{
					replacementPolicy = rrip;
				}
				else
				{
					replacementPolicy = lru;
				}
			}
			else if (key.compare("REPLACEMENT_PERIOD") == 0)
				convert_uint64_t(REPLACEMENT_PERIOD, value, key);
			else if (key.compare("CYCLES_PER_SECOND") == 0)
				convert_uint64_t(CYCLES_PER_SECOND, value, key);
			else if (key.compare("dram_ini") == 0)
				dram_ini = value;
			else if (key.compare("nvdimm_ini") == 0)
				nvdimm_ini = value;
			else if (key.compare("sys_ini") == 0)
				sys_ini = value;
			else if (key.compare("ENABLE_RESTORE") == 0)
				convert_uint64_t(ENABLE_RESTORE, value, key);
			else if (key.compare("ENABLE_SAVE") == 0)
				convert_uint64_t(ENABLE_SAVE, value, key);
			else if (key.compare("HYBRIDSIM_RESTORE_FILE") == 0)
				HYBRIDSIM_RESTORE_FILE = value;
			else if (key.compare("HYBRIDSIM_SAVE_FILE") == 0)
				HYBRIDSIM_SAVE_FILE = value;
			else if (key.compare("NVDIMM_RESTORE_FILE") == 0)
				NVDIMM_RESTORE_FILE = value;
			else if (key.compare("NVDIMM_SAVE_FILE") == 0)
				NVDIMM_SAVE_FILE = value;
			else
			{
				cerr << "ERROR: Illegal key/value pair in HybridSim ini file: " << key << "=" << value << "\n";
				cerr << "This could either be due to an illegal key or the incorrect value type for a key\n";
				abort();
			}
		}
	}

	void IniReader::read_nv_ini(string inifile)
	{
		ifstream inFile;
		char tmp[256];
		string tmp2;
		list<string> lines;

		inFile.open(inifile);
		if (!inFile.is_open())
		{
			cerr << "ERROR: Failed to load HybridSim's Ini file: " << inifile << "\n";
			abort();
		}

		while(!inFile.eof())
		{
			inFile.getline(tmp, 256);
			tmp2 = (string)tmp;

			// Filter comments out.
			size_t pos = tmp2.find("#");
			tmp2 = tmp2.substr(0, pos);

			// Strip whitespace from the ends.
			tmp2 = strip(tmp2);

			// Filter newlines out.
			if (tmp2.empty())
				continue;

			// Add it to the lines list.
			lines.push_back(tmp2);
		}
		inFile.close();

		list<string>::iterator it;
		for (it = lines.begin(); it != lines.end(); it++)
		{
			list<string> split_line = split((*it), "=", 2);

			if (split_line.size() != 2)
			{
				cerr << "ERROR: Parsing ini failed on line: " << (*it) << "\n";
				cerr << "There should be exactly one '=' per line\n";
				abort();
			}

			string key = split_line.front();
			string value = split_line.back();

			// Place the value into the appropriate global.
			if (key.compare("NUM_PACKAGES") == 0)
			{
				convert_uint64_t(NUM_CHANNELS, value, key);
			}
			else if (key.compare("DIES_PER_PACKAGE") == 0)
			{
				convert_uint64_t(RANKS_PER_CHANNEL, value, key);
			}
			else if (key.compare("PLANES_PER_DIE") == 0)
			{
				convert_uint64_t(BANKS_PER_RANK, value, key);
			}
			else if (key.compare("VIRTUAL_BLOCKS_PER_PLANE") == 0)
			{
				convert_uint64_t(ROWS_PER_BANK, value, key);
			}
			else if (key.compare("PAGES_PER_BLOCK") == 0)
			{
				convert_uint64_t(COL_PER_ROW, value, key);
			}
			// everything else in that file we don't care about right now
		}
	}
}
