/*******************************************************************************
* Copyright (c) 2012-2014, The Microsystems Design Labratory (MDL)
* Department of Computer Science and Engineering, The Pennsylvania State University
* All rights reserved.
* 
* This source code is part of NVMain - A cycle accurate timing, bit accurate
* energy simulator for both volatile (e.g., DRAM) and non-volatile memory
* (e.g., PCRAM). The source code is free and you can redistribute and/or
* modify it by providing that the following conditions are met:
* 
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
* 
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
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
* 
* Author list: 
*   Matt Poremba    ( Email: mrp5060 at psu dot edu 
*                     Website: http://www.cse.psu.edu/~poremba/ )
*******************************************************************************/

#include "Utils/Caches/CacheBank.h"
#include "include/NVMHelpers.h"
#include "src/EventQueue.h"
#include "include/CommonMath.h"
#include <iostream>
#include <cassert>

using namespace NVM;
// 
// const uint16_t bit_waymask_partitionDRAM[4] = {0x007F, 0x0F80, 0x0F80, 0xF000};
// LLC_size = 256kb no_partitionDRAM
// const uint16_t bit_waymask_partitionDRAM_12[6] = {0x0FFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
const uint16_t bit_waymask_partitionDRAM_12[6] = {0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF}; //12 curTick()>0

const uint16_t bit_waymask_partitionDRAM_6_6[6] = {0x003F, 0x003F, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0}; //6-6 curTick()>400

const uint16_t bit_waymask_partitionDRAM_10_6[6] = {0xF03F, 0xF03F, 0xFFC0, 0xFFC0, 0xFFC0, 0xFFC0}; //6-6+4 curTick()>1200

CacheBank::CacheBank( uint64_t sets, uint64_t assoc, uint64_t lineSize )
{   
    uint64_t i, j;
    cacheEntry = new CacheEntry* [ sets ];
    for( i = 0; i < sets; i++ )
    {
        cacheEntry[i] = new CacheEntry[assoc];
        for( j = 0; j < assoc; j++ )
        {
            /* Clear valid bit, dirty bit, etc. */
            cacheEntry[i][j].flags = CACHE_ENTRY_NONE;
        }
    }
    numSets = sets;
    numAssoc = assoc;
    cachelineSize = lineSize;
    tagOff = NVM::Log2(lineSize) + NVM::Log2(numSets);
    //std::cout<<"tag offset is:"<<tagOff<<std::endl;
    state = CACHE_IDLE;
    stateTimer = 0;

    decodeClass = NULL;
    decodeFunc = NULL;
    //std::cout<<"set default function"<<std::endl;
    SetDecodeFunction( this, 
            static_cast<CacheSetDecoder>(&NVM::CacheBank::DefaultDecoder) );

    readTime = 1; // 1 cycle
    writeTime = 1;  // 1 cycle

    isMissMap = false;
}

CacheBank::~CacheBank( )
{
    uint64_t i;
    for( i = 0; i < numSets; i++ )
    {
        delete [] cacheEntry[i];
    }
    delete [] cacheEntry;
}

void CacheBank::SetDecodeFunction( NVMObject *dcClass, CacheSetDecoder dcFunc )
{
    decodeClass = dcClass;
    decodeFunc = dcFunc;
}

//get cache set id
uint64_t CacheBank::DefaultDecoder( NVMAddress &addr )
{
    //std::cout<<"cache bank,default decoder:"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
    return (addr.GetPhysicalAddress( ) >> 
            (uint64_t)mlog2( (int)cachelineSize )) % numSets;
}

uint64_t CacheBank::SetID( NVMAddress& addr )
{
    /*
     *  By default we'll just chop off the bits for the cacheline and use the
     *  least significant bits as the set address, and the remaining bits are 
     *  the tag bits.
     */
    uint64_t setID;

    //if( isMissMap )
    //    setID = (addr.GetPhysicalAddress( )) % numSets;
    //std::cout<<"addr is:"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
    setID = (decodeClass->*decodeFunc)( addr );
    return setID;
}

CacheEntry *CacheBank::FindSet( NVMAddress& addr , uint64_t &set_id )
{
    /*
     *  By default we'll just chop off the bits for the cacheline and use the
     *  least significant bits as the set address, and the remaining bits are 
     *  the tag bits.
     */
    set_id = SetID( addr );

    return cacheEntry[set_id];
}

CacheEntry *CacheBank::FindSet( NVMAddress& addr)
{
    uint64_t set_id=0;
    return FindSet(addr, set_id);
}

//return true if cache exist data stored in addr.GetPhysicalAddress()
//LRU replacement algorithm 
bool CacheBank::Present( NVMAddress& addr, uint64_t &set_id , uint64_t &assoc_id, bool set_dirty )
{
    CacheEntry *set = FindSet( addr, set_id );
    bool found = false;
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
     //std::cout<<"judge weather hit: set address: 0x"<<std::hex<<set[i].address.GetPhysicalAddress()<<"request address: 0x"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
        //if( set[i].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( ) 
         if( GetTag(set[i].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
            && (set[i].flags & CACHE_ENTRY_VALID ) )
        {
            //std::cout<<"judge weather hit: set address: 0x"<<std::hex<<set[i].address.GetPhysicalAddress()<<"request address: 0x"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
            found = true;
            assoc_id = i; 
            if(set_dirty)
            {
                set[i].flags |= CACHE_ENTRY_DIRTY;
            }
            break;
        }
    }
    return found;
}

bool CacheBank::Present( NVMAddress& addr)
{
    uint64_t assoc, set;
    return Present(addr ,set, assoc);
}

/// @jia
///////////////////////////////////////////
//
///////////////////////////////////////////
bool CacheBank::Present_DSid( uint16_t DSid, NVMAddress& addr, uint64_t &set_id , uint64_t &assoc_id, bool set_dirty )
{
    /// return a set of blocks , whose set is set_id
    CacheEntry *set = FindSet( addr, set_id );
    bool found = false;
    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xFF00;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    // bit_waymask = bit_waymask_partitionDRAM_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    // 2020/04/10
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partitionDRAM_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0xF];

        }
*/
    // }
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( GetTag(set[j].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
                    && (set[j].flags & CACHE_ENTRY_VALID ) )
                {
                    found = true;
                    assoc_id = j; 
                    if(set_dirty)
                    {
                        set[j].flags |= CACHE_ENTRY_DIRTY;
                    }
                    break;
                }
            }

        }
        i++;
        maskA = maskA << 1;
    }
    return found;
}
bool CacheBank::Present_DSid( uint16_t DSid, NVMAddress& addr)
{
    uint64_t assoc, set;
    return Present_DSid(DSid, addr ,set, assoc);
}


inline uint64_t CacheBank::GetTag( uint64_t addr )
{
    return addr>>tagOff;    
}

bool CacheBank::SetFull( NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = true;

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        /* If there is an invalid entry (e.g., not used) the set isn't full. */
        //modified on Mon 2 Mar
        if( !((set[i].flags & CACHE_ENTRY_VALID)||(set[i].flags & CACHE_ENTRY_DIRTY)||(set[i].flags & CACHE_ENTRY_EXAMPLE)) )
        {
            rv = false;
            break;
        }
    }

    return rv;
}

/// @jia
/////////////////////////////////////////////
//
/////////////////////////////////////////////
bool CacheBank::SetFull_DSid( uint16_t DSid, NVMAddress& addr ) // build/X86/nvmain/Utils/Caches/CacheBank.cpp
{
    CacheEntry *set = FindSet( addr );
    bool rv = true;
    uint16_t bit_waymask = 0xFFFF;
/*    if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x00FF;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x00FF;
    }else if (DSid == 0x2)
    {
        bit_waymask = 0xFF00;
    }else if (DSid == 0x3)
    {
        bit_waymask = 0xF000;
    }*/
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !((set[j].flags & CACHE_ENTRY_VALID)||(set[j].flags & CACHE_ENTRY_DIRTY)
                    ||(set[j].flags & CACHE_ENTRY_EXAMPLE)) )
                {
                    rv = false;
                    break;
                }
            }

        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}
//it can install successfully only if cache set is not full,return true;
//else return false
bool CacheBank::Install( NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    //assert( !Present( addr ) );
    //std::cout<<"install 0x"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( !(set[i].flags & CACHE_ENTRY_VALID) )
        {
            set[i].address = addr;
            set[i].data = data;
            set[i].flags |= CACHE_ENTRY_VALID; 
            rv = true;
            break;
        }
    }
    return rv;
}
/// @jia
#if 1
bool CacheBank::Install_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;
    uint16_t bit_waymask = 0xFFFF;
    /*if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x00FF;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x0F00;
    }else if (DSid == 0x2)
    {
        bit_waymask = 0xF000;
    }else if (DSid == 0x3)
    {
        bit_waymask = 0xF000;
    }*/
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !(set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].address = addr;
                    set[j].data = data;
                    set[j].flags |= CACHE_ENTRY_VALID; 
                    rv = true;
                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}

#else
bool CacheBank::Install_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;
    uint16_t bit_waymask = 0xFFFF;
    if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x0003;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x0003;
    }
    uint16_t maskA = 0x1;
    int index_set[16];
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            /// append index i
            index_set[ index++ ] = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !(set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].address = addr;
                    set[j].data = data;
                    set[j].flags |= CACHE_ENTRY_VALID; 
                    rv = true;

                    /* Move cache entry to MRU position */
                    CacheEntry tmp;

                    tmp.flags = set[j].flags;
                    tmp.address = set[j].address;
                    tmp.data = set[j].data;

                    int index_start = index_set[0]*16;


                    for( uint64_t k = j; k > index_start; k-- )
                    {
                        set[k].flags = set[k-1].flags;
                        set[k].address = set[k-1].address;
                        set[k].data = set[k-1].data;
                    }

                    set[index_start].flags = tmp.flags;
                    set[index_start].address = tmp.address;
                    set[index_start].data = tmp.data;


                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}
#endif

bool CacheBank::FindAssoc( NVMAddress& addr, uint64_t &set_id , uint64_t &assoc_id )
{
    CacheEntry *set = FindSet( addr, set_id );
    //assert( !Present( addr ) );
    //std::cout<<"install 0x"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( !(set[i].flags & CACHE_ENTRY_VALID) )
        {
            assoc_id = i;
            break;
        }
    }
    return true;
}
/// @jia
bool CacheBank::FindAssoc_DSid( uint16_t DSid, NVMAddress& addr, uint64_t &set_id , uint64_t &assoc_id )
{
    CacheEntry *set = FindSet( addr, set_id );
    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x0F00;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xF000;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !(set[j].flags & CACHE_ENTRY_VALID) )
                {
                    assoc_id = j;
                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return true;
}

bool CacheBank::Install( NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    //assert( !Present( addr ) );
    //std::cout<<"install 0x"<<std::hex<<addr.GetPhysicalAddress()<<std::endl;
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( !(set[i].flags & CACHE_ENTRY_VALID) )
        {
            set[i].address = addr;
            set[i].flags |= CACHE_ENTRY_VALID; 
            rv = true;
            /// @jia:Move cache entry to MRU position 
            CacheEntry tmp;

            tmp.flags = set[i].flags;
            tmp.address = set[i].address;
            // tmp.data = set[i].data;

            for( uint64_t j = i; j > 0; j-- )
            {
                set[j].flags = set[j-1].flags;
                set[j].address = set[j-1].address;
                // set[j].data = set[j-1].data;
            }

            set[0].flags = tmp.flags;
            set[0].address = tmp.address;
            // set[0].data = tmp.data;

            break;
        }
    }
    return rv;
}
/// @jia
//////////////////////////////////////////////////
//
//////////////////////////////////////////////////
#if 1
bool CacheBank::Install_DSid( uint16_t DSid, NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xFF00;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    // bit_waymask = bit_waymask_partitionDRAM_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020/0410
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partitionDRAM_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0xF];

        }*/

    // }
    uint16_t maskA = 0x1;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            /// append index i
            //index_set[ index++ ] = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !(set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].address = addr;
                    set[j].flags |= CACHE_ENTRY_VALID; 
                    rv = true;
                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}
#else
bool CacheBank::Install_DSid( uint16_t DSid, NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    uint16_t bit_waymask = 0xFFFF;
    if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x0003;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x0003;
    }
    uint16_t maskA = 0x1;
    int index_set[16];
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            /// append index i
            index_set[ index++ ] = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( !(set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].address = addr;
                    set[j].flags |= CACHE_ENTRY_VALID; 
                    rv = true;

                    /* Move cache entry to MRU position */
                    CacheEntry tmp;

                    tmp.flags = set[j].flags;
                    tmp.address = set[j].address;
                    tmp.data = set[j].data;

                    int index_start = index_set[0]*16;


                    for( uint64_t k = j; k > index_start; k-- )
                    {
                        set[k].flags = set[k-1].flags;
                        set[k].address = set[k-1].address;
                        set[k].data = set[k-1].data;
                    }

                    set[index_start].flags = tmp.flags;
                    set[index_start].address = tmp.address;
                    set[index_start].data = tmp.data;

                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}
#endif

bool CacheBank::Read( NVMAddress& addr, NVMDataBlock *data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present( addr ) );

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( GetTag(set[i].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            *data = set[i].data;
            rv = true;

            /* Move cache entry to MRU position */
            CacheEntry tmp;

            tmp.flags = set[i].flags;
            tmp.address = set[i].address;
            tmp.data = set[i].data;

            for( uint64_t j = i; j > 0; j-- )
            {
                set[j].flags = set[j-1].flags;
                set[j].address = set[j-1].address;
                set[j].data = set[j-1].data;
            }

            set[0].flags = tmp.flags;
            set[0].address = tmp.address;
            set[0].data = tmp.data;
        }
    }

    return rv;
}
bool CacheBank::Read( NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present( addr ) );

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( GetTag(set[i].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            // *data = set[i].data;
            rv = true;

            /* Move cache entry to MRU position */
            CacheEntry tmp;

            tmp.flags = set[i].flags;
            tmp.address = set[i].address;
            // tmp.data = set[i].data;

            for( uint64_t j = i; j > 0; j-- )
            {
                set[j].flags = set[j-1].flags;
                set[j].address = set[j-1].address;
                // set[j].data = set[j-1].data;
            }

            set[0].flags = tmp.flags;
            set[0].address = tmp.address;
            // set[0].data = tmp.data;
        }
    }

    return rv;
}
/// @jia
bool CacheBank::Read_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock *data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert(Present_DSid(DSid, addr));

    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x0F00;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xF000;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    uint16_t maskA = 0x1;
    int index_set[16];
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            /// append index i
            index_set[ index++ ] = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( GetTag(set[j].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
                    && (set[j].flags & CACHE_ENTRY_VALID) )
                {
                    *data = set[j].data;
                    rv = true;

                    /* Move cache entry to MRU position */
                    CacheEntry tmp;

                    tmp.flags = set[j].flags;
                    tmp.address = set[j].address;
                    tmp.data = set[j].data;

                    int index_start = index_set[0]*16;


                    for( uint64_t k = j; k > index_start; k-- )
                    {
                        set[k].flags = set[k-1].flags;
                        set[k].address = set[k-1].address;
                        set[k].data = set[k-1].data;
                    }

                    set[index_start].flags = tmp.flags;
                    set[index_start].address = tmp.address;
                    set[index_start].data = tmp.data;

                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }

    return rv;
}

bool CacheBank::Write( NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present( addr ) );

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( set[i].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( )
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            set[i].data = data;
            set[i].flags |= CACHE_ENTRY_DIRTY;
            rv = true;

            /* Move cache entry to MRU position */
            CacheEntry tmp;

            tmp.flags = set[i].flags;
            tmp.address = set[i].address;
            tmp.data = set[i].data;

            for( uint64_t j = i; j > 1; j-- )
            {
                set[j].flags = set[j-1].flags;
                set[j].address = set[j-1].address;
                set[j].data = set[j-1].data;
            }

            set[0].flags = tmp.flags;
            set[0].address = tmp.address;
            set[0].data = tmp.data;
        }
    }

    return rv;
}
bool CacheBank::Write( NVMAddress& addr )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present( addr ) );

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( set[i].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( )
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            // set[i].data = data;
            set[i].flags |= CACHE_ENTRY_DIRTY;
            rv = true;

            /* Move cache entry to MRU position */
            CacheEntry tmp;

            tmp.flags = set[i].flags;
            tmp.address = set[i].address;
            // tmp.data = set[i].data;

            for( uint64_t j = i; j > 1; j-- )
            {
                set[j].flags = set[j-1].flags;
                set[j].address = set[j-1].address;
                // set[j].data = set[j-1].data;
            }

            set[0].flags = tmp.flags;
            set[0].address = tmp.address;
            // set[0].data = tmp.data;
        }
    }

    return rv;
}
/// @jia
bool CacheBank::Write_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present_DSid( DSid, addr ));

    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x0F00;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xF000;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    uint16_t maskA = 0x1;
    int index_set[16];
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            /// append index i
            index_set[ index++ ] = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( set[i].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( )
                    && (set[i].flags & CACHE_ENTRY_VALID) )
                {
                    set[i].data = data;
                    set[i].flags |= CACHE_ENTRY_DIRTY;
                    rv = true;

                    /* Move cache entry to MRU position */
                    CacheEntry tmp;

                    tmp.flags = set[j].flags;
                    tmp.address = set[j].address;
                    tmp.data = set[j].data;

                    int index_start = index_set[0]*16;

                    for( uint64_t k = j; k > index_start; k-- )
                    {
                        set[k].flags = set[k-1].flags;
                        set[k].address = set[k-1].address;
                        set[k].data = set[k-1].data;
                    }

                    set[index_start].flags = tmp.flags;
                    set[index_start].address = tmp.address;
                    set[index_start].data = tmp.data;

                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    return rv;
}
/* 
 *  Updates data without changing dirty bit or LRU position
 *  Returns true if the block was found and updated.
 */
bool CacheBank::UpdateData( NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert( Present( addr ) );

    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( set[i].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( )
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            set[i].data = data;
            rv = true;
        }
    }
    return rv;
}

/// @jia
bool CacheBank::UpdateData_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock& data )
{
    CacheEntry *set = FindSet( addr );
    bool rv = false;

    assert(Present_DSid( DSid, addr ));
    uint16_t bit_waymask = 0xFFFF;
    if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x00FF;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x0F00;
    }else if (DSid == 0x2)
    {
        bit_waymask = 0xF000;
    }else if (DSid == 0x3)
    {
        bit_waymask = 0xF000;
    }
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( set[j].address.GetPhysicalAddress( ) == addr.GetPhysicalAddress( )
                    && (set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].data = data;
                    rv = true;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }

    return rv;
}

/* Return true if the victim data is dirty. */
bool CacheBank::ChooseVictim( NVMAddress& addr, NVMAddress *victim ,
                            uint64_t &set_id , uint64_t assoc_id )
{
    bool rv = false;
    CacheEntry *set = FindSet( addr , set_id);
    assert( SetFull( addr ) );
    assert( set[numAssoc-1].flags & CACHE_ENTRY_VALID );
    //the last one of a group as victim
    *victim = set[numAssoc-1].address;
    //std::cout<<"choose victim for:"<<std::hex<<addr.GetPhysicalAddress()<<"address of victim is"<<std::hex<<victim->GetPhysicalAddress()<<std::endl;
    if( set[numAssoc-1].flags & CACHE_ENTRY_DIRTY )
        rv = true;
    assoc_id = numAssoc-1;
    if(assoc_id == numAssoc)
        return false;
    return rv;
}

bool CacheBank::ChooseVictim( NVMAddress& addr, NVMAddress *victim)
{
    uint64_t set_id=0 , assoc_id=0;
    return ChooseVictim(addr, victim , set_id , assoc_id);
}

/// @jia
///////////////////////////////////////////////
//
///////////////////////////////////////////////
bool CacheBank::ChooseVictim_DSid( uint16_t DSid, NVMAddress& addr, NVMAddress *victim ,
                            uint64_t &set_id , uint64_t assoc_id )
{
    bool rv = false;
    CacheEntry *set = FindSet( addr , set_id);
    assert( SetFull_DSid( DSid, addr ) );
    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xFF00;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    // bit_waymask = bit_waymask_partitionDRAM_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020/04/10
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partitionDRAM_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0xF];
        }*/

    // }
    uint16_t maskA = 0x1;
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0)
        {
            index = i;
        }
        i++;
        maskA = maskA << 1;
    }
    //find the maximum index
    int index_end = index*16 + 15;
    assert( set[index_end].flags & CACHE_ENTRY_VALID );
    //the last one of a group as victim
    *victim = set[index_end].address;

    if( set[index_end].flags & CACHE_ENTRY_DIRTY )
        rv = true;
    assoc_id = index_end;
    if(assoc_id == numAssoc)
        return false;
    return rv;
}

bool CacheBank::ChooseVictim_DSid( uint16_t DSid, NVMAddress& addr, NVMAddress *victim )
{
    uint64_t set_id=0 , assoc_id=0;
    return ChooseVictim_DSid(DSid, addr, victim , set_id , assoc_id);
}

bool CacheBank::Evict(NVMAddress &addr , uint64_t &set_id , uint64_t &assoc_id)
{
    CacheEntry *set = FindSet( addr , set_id);
    assert( Present( addr ) );
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( GetTag(set[i].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            set[i].flags = CACHE_ENTRY_NONE;
            assoc_id = i;
            break;
        }
    }
    return true;
}

/// @jia
//////////////////////////////////////////
//
///////////////////////////////////////////
bool CacheBank::Evict_DSid( uint16_t DSid, NVMAddress &addr , uint64_t &set_id , uint64_t &assoc_id )
{
    CacheEntry *set = FindSet( addr , set_id);
    assert( Present_DSid( DSid, addr ) );

    uint16_t bit_waymask = 0xFFFF;
    // if( DSid == 0x0)
    // {
    //     /// DSid:0->bank0
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x1)
    // {
    //     /// DSid:1->bank1
    //     bit_waymask = 0x00FF;
    // }else if (DSid == 0x2)
    // {
    //     bit_waymask = 0xFF00;
    // }else if (DSid == 0x3)
    // {
    //     bit_waymask = 0xF000;
    // }
    // bit_waymask = bit_waymask_partitionDRAM_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020/04/10
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partitionDRAM_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partitionDRAM_6_6[DSid - 0xF];
        }*/

    // }
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( GetTag(set[j].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
                    && (set[j].flags & CACHE_ENTRY_VALID) )
                {
                    set[j].flags = CACHE_ENTRY_NONE;
                    assoc_id = j;
                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }

    return true;
}

bool CacheBank::Evict( NVMAddress& addr, NVMDataBlock *data )
{
    bool rv;
    CacheEntry *set = FindSet( addr );
    assert( Present( addr ) );
    rv = false; 
    for( uint64_t i = 0; i < numAssoc; i++ )
    {
        if( GetTag(set[i].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
            && (set[i].flags & CACHE_ENTRY_VALID) )
        {
            if( set[i].flags & CACHE_ENTRY_DIRTY )
            {
                *data = set[i].data;
                rv = true;
            }
            else
            {
                *data = set[i].data;
                rv = false;
            }
            set[i].flags = CACHE_ENTRY_NONE;
            break;
        }
    }
    return rv;
}

bool CacheBank::Evict_DSid( uint16_t DSid, NVMAddress& addr, NVMDataBlock *data )
{
    bool rv;
    CacheEntry *set = FindSet( addr );
    assert( Present_DSid( DSid, addr ) );
    rv = false; 

    uint16_t bit_waymask = 0xFFFF;
    if( DSid == 0x0)
    {
        /// DSid:0->bank0
        bit_waymask = 0x00FF;
    }else if (DSid == 0x1)
    {
        /// DSid:1->bank1
        bit_waymask = 0x00FF;
    }else if (DSid == 0x2)
    {
        bit_waymask = 0xFF00;
    }else if (DSid == 0x3)
    {
        bit_waymask = 0xF000;
    }
    uint16_t maskA = 0x1;
    //int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            //index = i;
            for( uint64_t j = i*16; j < (i+1)*16; j++ ){
                /// matches the block
                if( GetTag(set[j].address.GetPhysicalAddress()) == GetTag(addr.GetPhysicalAddress()) 
                    && (set[j].flags & CACHE_ENTRY_VALID) )
                {
                    if( set[j].flags & CACHE_ENTRY_DIRTY )
                    {
                        *data = set[j].data;
                        rv = true;
                    }
                    else
                    {
                        *data = set[j].data;
                        rv = false;
                    }
                    set[j].flags = CACHE_ENTRY_NONE;
                    break;
                }
            }
        }
        i++;
        maskA = maskA << 1;
    }
    
    return rv;
}

void CacheBank::SetReadTime( uint64_t rtime )
{
    readTime = rtime;
}

void CacheBank::SetWriteTime( uint64_t wtime )
{
    writeTime = wtime;
}

uint64_t CacheBank::GetReadTime( )
{
    return readTime;
}

uint64_t CacheBank::GetWriteTime( )
{
    return writeTime;
}

uint64_t CacheBank::GetAssociativity( )
{
    return numAssoc;
}

uint64_t CacheBank::GetCachelineSize( )
{
    return cachelineSize;
}

uint64_t CacheBank::GetSetCount( )
{
    return numSets;
}

double CacheBank::GetCacheOccupancy( )
{
    double occupancy;
    uint64_t valid, total;

    valid = 0;
    total = numSets*numAssoc;

    for( uint64_t setIdx = 0; setIdx < numSets; setIdx++ )
    {
        CacheEntry *set = cacheEntry[setIdx];

        for( uint64_t assocIdx = 0; assocIdx < numAssoc; assocIdx++ )
        {
            if( set[assocIdx].flags & CACHE_ENTRY_VALID )
                valid++;
        }
    }

    occupancy = static_cast<double>(valid) / static_cast<double>(total);

    return occupancy;
}

bool CacheBank::IsIssuable( NVMainRequest * /*req*/, FailReason * /*reason*/ )
{
    bool rv = false;

    /* We can issue if the cache is idle. Pretty simple */
    if( state == CACHE_IDLE )
    {
        rv = true;
    }
    else
    {
        rv = false;
    }

    return rv;
}

bool CacheBank::IssueCommand( NVMainRequest *nreq )
{
    NVMDataBlock dummy;
    CacheRequest *req = static_cast<CacheRequest *>( nreq->reqInfo );

    assert( IsIssuable( nreq, NULL ) );

    if( !IsIssuable( nreq, NULL ) )
        return false;

    switch( req->optype )
    {
        case CACHE_READ:
            state = CACHE_BUSY;
            stateTimer = GetEventQueue( )->GetCurrentCycle( ) + readTime;
            req->hit = Present( req->address );
            if( req->hit ) 
                Read( req->address, &(req->data) ); 
            GetEventQueue( )->InsertEvent( EventResponse, this, nreq, stateTimer );
            break;

        case CACHE_WRITE:
            state = CACHE_BUSY;
            stateTimer = GetEventQueue( )->GetCurrentCycle( ) + writeTime;

            if( SetFull( req->address ) )
            {
                NVMainRequest *mmEvict = new NVMainRequest( );
                CacheRequest *evreq = new CacheRequest;
                
                ChooseVictim( req->address, &(evreq->address) );
                Evict( evreq->address, &(evreq->data) );

                evreq->optype = CACHE_EVICT;

                *mmEvict = *nreq;
                mmEvict->owner = nreq->owner;
                mmEvict->reqInfo = static_cast<void *>( evreq );
                mmEvict->tag = nreq->tag;

                GetEventQueue( )->InsertEvent( EventResponse, this, mmEvict,
                                               stateTimer );

                //20200416-delete operated by jia
                // delete mmEvict, evreq;
            }

            req->hit = Present( req->address );
            if( req->hit ) 
                Write( req->address, req->data );
            else
                Install( req->address, req->data );

            GetEventQueue( )->InsertEvent( EventResponse, this, nreq, stateTimer );

            break;

        default:
            std::cout << "CacheBank: Unknown operation `" << req->optype << "'!"
                << std::endl;
            break;
    }
    return true;
}

bool CacheBank::RequestComplete( NVMainRequest *req )
{
    GetParent( )->RequestComplete( req );
    state = CACHE_IDLE;
    return true;
}

void CacheBank::Cycle( ncycle_t /*steps*/ )
{
}
