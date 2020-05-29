/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2009 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of an associative set
 */

#ifndef __MEM_CACHE_TAGS_CACHESET_HH__
#define __MEM_CACHE_TAGS_CACHESET_HH__

#include <cassert>
#include <vector>
#include <fstream>
#include "base/types.hh"

/**
 * An associative set of cache blocks.
 */
//------------------------



//------------------------

// LLC_size = 512KB
// const uint16_t bit_waymask_partition[4] = {0x007F, 0x0F80, 0x0F80, 0xF000};
// LLC_size = 256KB
// const uint16_t bit_waymask_partition_12[6] = {0x03FF, 0x03FF, 0xFC00, 0xFC00, 0xFC00, 0xFC00}; //10-6

const uint16_t bit_waymask_partition[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}; 
// const uint16_t bit_waymask_partition_6_6[6] = {0x003F, 0x003F, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0}; //6-6
// const uint16_t bit_waymask_partition_10_6[6] = {0xF03F, 0xF03F, 0x0FC0, 0x0FC0, 0x0FC0, 0x0FC0}; //10-6
// const uint16_t bit_waymask_partition[6] = {0x0FFF, 0x0FFF, 0xF000, 0xF000, 0xF000, 0xF000}; //12-4
// const uint16_t bit_waymask_partition[6] = {0x1FFF, 0x1FFF, 0xE000, 0xE000, 0xE000, 0xE000}; //13-3

// 2020-1-5
// const uint16_t bit_waymask_partition[6] = {0x000F, 0x00F0, 0x0F00, 0xF000, 0xE000, 0xE000}; //4-4-4-4

// const uint16_t bit_waymask_partition_d[6] = {0x000F, 0x00F0, 0x0F00, 0xF000, 0xE000, 0xE000}; //4-4-4-4

// const uint16_t bit_waymask_partition_t[6] = {0x000F, 0x00F0, 0x0F00, 0xF000, 0xE000, 0xE000}; //4-4-4-4
// 2019-12-26
// 6-4-6  // 6-2-2-2-2-2
 // const uint16_t bit_waymask_partition[6] = {0x003F, 0x00C0, 0x0300, 0x0C00, 0x3000, 0xC000};
//  3-3-2-2-3-3
 // const uint16_t bit_waymask_partition[6] = {0x0007, 0x0038, 0x00C0, 0x0300, 0x1C00, 0xE000};
//  6-4-6
 // const uint16_t bit_waymask_partition[6] = {0x003F, 0x03C0, 0xFC00, 0xFFFF, 0xFFFF, 0xFFFF};
 
template <class Blktype>
class CacheSet
{
  public:
    /** The associativity of this set. */
    int assoc;

    /** Cache blocks in this set, maintained in LRU order 0 = MRU. */
    std::vector<Blktype*> blks;

    /**
     * Find a block matching the tag in this set.
     * @param way_id The id of the way that matches the tag.
     * @param tag The Tag to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the block if found. Set way_id to assoc if none found
     */
    Blktype* findBlk(Addr tag, bool is_secure, int& way_id) const ;
    Blktype* findBlk(Addr tag, bool is_secure) const ;

    // l2 cache : pkt ->getDSid()
    Blktype* findBlk_l2(uint16_t DSid, Addr tag, bool is_secure, int& way_id, std::vector<int>bit_waymask_partitions) const ;
    Blktype* findBlk_l2(uint16_t DSid, Addr tag, bool is_secure, std::vector<int>bit_waymask_partitions) const ;

    /**
     * Move the given block to the head of the list.
     * @param blk The block to move.
     */
    void moveToHead(Blktype *blk);
    void moveToHead_l2(uint16_t DSid, Blktype *blk, std::vector<int>bit_waymask_partitions);

    /**
     * Move the given block to the tail of the list.
     * @param blk The block to move
     */
    void moveToTail(Blktype *blk);
    void moveToTail_l2(uint16_t DSid, Blktype *blk, std::vector<int>bit_waymask_partitions);

};

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, int& way_id) const
{
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;
    for (int i = 0; i < assoc; ++i) {
        if (blks[i]->tag == tag && blks[i]->isValid() &&
            blks[i]->isSecure() == is_secure) {
            way_id = i;
            return blks[i];
        }
    }
    return nullptr;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure) const
{
    int ignored_way_id;
    return findBlk(tag, is_secure, ignored_way_id);
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk_l2(uint16_t DSid, Addr tag, bool is_secure, int& way_id, std::vector<int>bit_waymask_partitions) const
{                                                                  //  build/X86/mem/cache/tags/cacheset.hh:
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    uint16_t bit_waymask = 0xFFFF;
    bit_waymask = bit_waymask_partitions[DSid];
        // if(DSid == 0x0){
        //     bit_waymask = 0x0FFF;
        // }else if (DSid == 0x1)
        // {
        //     bit_waymask = 0x0C00;
        // }else if (DSid == 0x2)
        // {
        //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
        //     bit_waymask = 0xF000;
        // }else if (DSid == 0x3)
        // {
        //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
        //     bit_waymask = 0xFFFF;
        // }
    // bit_waymask = bit_waymask_partition_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020-------------------------------------------------------
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partition_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0xF];
            if (curTick() > 150000000000)
            {
                bit_waymask = bit_waymask_partition_10_6[DSid - 0xF];
            }
        }*/
    //2020-------------------------------------------------------
    // }
    uint16_t maskA = 0x1;
    int index = 0;
    int i = 0;
    while(maskA){
        if ((maskA & bit_waymask) != 0){
            index = i;
            if (blks[i]->tag == tag && blks[i]->isValid() &&
                blks[i]->isSecure() == is_secure) {
                way_id = i;
                return blks[i];
            }
        }
        i++;
        maskA = maskA << 1;
    }
    //If no block is found way_id is set to maximum index.
    way_id = index;
    //
    return NULL;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk_l2(uint16_t DSid, Addr tag, bool is_secure, std::vector<int>bit_waymask_partitions) const
{
    int ignored_way_id;
    return findBlk_l2(DSid, tag, is_secure, ignored_way_id, bit_waymask_partitions);
}


template <class Blktype>
void
CacheSet<Blktype>::moveToHead(Blktype *blk)
{
    // nothing to do if blk is already head
    if (blks[0] == blk)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = 0;
    Blktype *next = blk;

    do {
        assert(i < assoc);
        std::swap(blks[i], next);
        ++i;
    } while (next != blk);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToHead_l2(uint16_t DSid, Blktype *blk, std::vector<int>bit_waymask_partitions)
{
    // nothing to do if blk is already head
    // if (blks[0] == blk)
    //     return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    uint16_t bit_waymask = 0xFFFF;
    bit_waymask = bit_waymask_partitions[DSid];
    // if(DSid == 0x0){
    //     bit_waymask = 0x0FFF;
    // }else if (DSid == 0x1)
    // {
    //     bit_waymask = 0x0C00;
    // }else if (DSid == 0x2)
    // {
    //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
    //     bit_waymask = 0xF000;
    // }else if (DSid == 0x3)
    // {
    //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
    //     bit_waymask = 0xFFFF;
    // }
    //uint16_t bit_waymask = 0x000F;
    // bit_waymask = bit_waymask_partition_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020-------------------------------------------------------
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partition_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0xF];
            if (curTick() > 150000000000)
            {
                bit_waymask = bit_waymask_partition_10_6[DSid - 0xF];
            }
        }*/

    // }
    uint16_t maskA = 0x1;
    int i = 0;
    int count = 0;
    Blktype *next = blk; 

    // do {
    //     assert(i < assoc);
    //     if ((maskA & bit_waymask) != 0){
    //         // nothing to do if blk is already head
    //         if( (count == 0) && (blks[i] == blk) ){
    //             return;
    //         }
    //         // swap blks[i] and next
    //         std::swap(blks[i], next);
    //         // counter
    //         count ++ ;
    //     }
    //     ++i;
    //     //printf("LRU::insertBlock:%d\n",i);
    //     maskA = maskA << 1;
    // } while (next != blk); 

    for( i=0; i<assoc; i++ ){

        if ((maskA & bit_waymask) != 0){
            // nothing to do if blk is already tail
            // do {  
                if((count == 0)&&(blks[i] == blk)){
                    return;
                }
                // swap blks[i] and next
                std::swap(blks[i], next);

                if(next == blk){
                    return;
                }
                // counter
            // } while (next != blk);

            count ++ ;
        }
        maskA = maskA << 1;
    }
    printf("moveToHead_l2 failed!!!!!!!!!!!!!!!!!!!\n");

}


template <class Blktype>
void
CacheSet<Blktype>::moveToTail(Blktype *blk)
{
    // nothing to do if blk is already tail
    if (blks[assoc - 1] == blk)
        return;

    // write 'next' block into blks[i], moving from LRU to MRU
    // until we overwrite the block we moved to tail.

    // start by setting up to write 'blk' into tail
    int i = assoc - 1;
    Blktype *next = blk;

    do {
        assert(i >= 0);
        std::swap(blks[i], next);
        --i;
    } while (next != blk);
}


template <class Blktype>
void
CacheSet<Blktype>::moveToTail_l2(uint16_t DSid, Blktype *blk, std::vector<int>bit_waymask_partitions) // build/X86/mem/cache/tags/cacheset.hh:304
{
    // nothing to do if blk is already tail
    // if (blks[assoc - 1] == blk)
    //     return;

    // write 'next' block into blks[i], moving from LRU to MRU
    // until we overwrite the block we moved to tail.

    // start by setting up to write 'blk' into tail
    uint16_t bit_waymask = 0xFFFF;
    bit_waymask = bit_waymask_partitions[DSid];
    // if(DSid == 0x0){
    //     bit_waymask = 0x0FFF;
    // }else if (DSid == 0x1)
    // {
    //     bit_waymask = 0x0C00;
    // }else if (DSid == 0x2)
    // {
    //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
    //     bit_waymask = 0xF000;
    // }else if (DSid == 0x3)
    // {
    //     //printf("DSid:0x%x  ->   bit_waymask: 0x00F0 \n", uint16_t(pkt->getDSid()));
    //     bit_waymask = 0xFFFF;
    // }

    //uint16_t bit_waymask = 0x000F;
    // bit_waymask = bit_waymask_partition_12[DSid];
    // if (curTick() > 40000000000 && curTick() < 80000000000)
    // {
    //2020-------------------------------------------------------
/*        if (DSid < 0x6){
          bit_waymask = bit_waymask_partition_12[DSid];
        }
        else if(DSid >= 0x6 && DSid < 0xF){
            //DSid = DSid - 0x6;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0x6];
        }
        else if(DSid >= 0xF){
            //DSid = DSid - 0xF;
            bit_waymask = bit_waymask_partition_6_6[DSid - 0xF];
            if (curTick() > 150000000000)
            {
                bit_waymask = bit_waymask_partition_10_6[DSid - 0xF];
            }
        }*/

    // }
    uint16_t maskA = 0x8000;
    int i = 0;
    int count = 0;
    Blktype *next = blk; 

    //
    // do {
    //     assert(i <= assoc);
    //     if ((maskA & bit_waymask) != 0){
    //         // nothing to do if blk is already tail
    //         if((count == 0)&&(blks[i] == blk)){
    //             return;
    //         }
    //         // swap blks[i] and next
    //         std::swap(blks[i], next);
    //         // counter
    //         count ++ ;
    //     }
    //     ++i;
    //     maskA = maskA >> 1;
    // } while (next != blk);

    for( i=assoc-1; i>=0; i-- ){

        if ((maskA & bit_waymask) != 0){
            // nothing to do if blk is already tail
            // do {  
                if((count == 0)&&(blks[i] == blk)){
                    return;
                }
                // swap blks[i] and next
                std::swap(blks[i], next);

                if(next == blk){
                    printf("DSid:0x%x, moveToTail_l2 successed!!!~~~!!!\n", uint16_t(DSid));
                    return;
                }
                // counter
            // } while (next != blk);

            count ++ ;
        }
        maskA = maskA >> 1;
    }
    printf("DSid:0x%x, moveToTail_l2 failed~~~\n", uint16_t(DSid));

}

#endif
