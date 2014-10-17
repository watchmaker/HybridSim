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

//AddressDecode.cpp
//class file for address deocder functionality
//

#include "AddressDecode.h"

using namespace HybridSim;
using namespace std;

AddressDecode::AddressDecode()
{
	channelBitWidth = hybridsim_log2(NVDSim::NUM_PACKAGES);
	rankBitWidth = hybridsim_log2(NVDSim::DIES_PER_PACKAGE);
	bankBitWidth = hybridsim_log2(NVDSim::PLANES_PER_DIE);
	rowBitWidth = hybridsim_log2(NVDSim::VIRTUAL_BLOCKS_PER_PLANE);
	colBitWidth = hybridsim_log2(NVDSim::PAGES_PER_BLOCK);
	colOffset = hybridsim_log2(NVDSim::NV_PAGE_SIZE);

	if(DEBUG_COMBO_TAG)
	{
		cerr << "channel bits " << channelBitWidth << "\n";
		cerr << "rank bits " << rankBitWidth << "\n";
		cerr << "bank bits " << bankBitWidth << "\n";
		cerr << "row bits " << rowBitWidth << "\n";
		cerr << "col bits " << colBitWidth << "\n";
		cerr << "col offset " << colOffset << "\n";
	}
}

AddressSet AddressDecode::getDecode(uint64_t addr)
{
	uint64_t physicalAddress, tempA, tempB;
	AddressSet decoded_addr;

	physicalAddress = addr >> colOffset;
				
	tempA = physicalAddress;
	physicalAddress = physicalAddress >> colBitWidth;
	tempB = physicalAddress << colBitWidth;
	decoded_addr.column = tempA ^ tempB;
				
	tempA = physicalAddress;
	physicalAddress = physicalAddress >> rowBitWidth;
	tempB = physicalAddress << rowBitWidth;
	decoded_addr.row = tempA ^ tempB;
				
	tempA = physicalAddress;
	physicalAddress = physicalAddress >> bankBitWidth;
	tempB = physicalAddress << bankBitWidth;
	decoded_addr.bank = tempA ^ tempB;
			
	tempA = physicalAddress;
	physicalAddress = physicalAddress >> rankBitWidth;
	tempB = physicalAddress << rankBitWidth;
	decoded_addr.rank = tempA ^ tempB;
				
	tempA = physicalAddress;
	physicalAddress = physicalAddress >> channelBitWidth;
	tempB = physicalAddress << channelBitWidth;
	decoded_addr.channel = tempA ^ tempB;

	return decoded_addr;	
}
