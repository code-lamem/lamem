/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <functional>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/CacheRepl.hh"
#include "mem/cache/base.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/cacheset.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

// const uint16_t bit_waymask_partition[4] = {0x00FF, 0x1F00, 0x1F00, 0xE000};
/**
 * A BaseSetAssoc cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways). A cache line is mapped onto a set, and can be placed
 * into any of the ways of this set.
 */
class BaseSetAssoc : public BaseTags
{
  public:
    /** Typedef the block type used in this tag store. */
    typedef CacheBlk BlkType;
    /** Typedef the set type used in this tag store. */
    typedef CacheSet<CacheBlk> SetType;

  protected:
    /** The associativity of the cache. */
    const unsigned assoc;
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** The cache blocks. */
    std::vector<BlkType> blks;

    /** The number of sets in the cache. */
    const unsigned numSets;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** The cache sets. */
    std::vector<SetType> sets;

    /** The amount to shift the address to get the set. */
    int setShift;
    /** The amount to shift the address to get the tag. */
    int tagShift;
    /** Mask out all bits that aren't part of the set index. */
    unsigned setMask;

    /** Replacement policy */
    BaseReplacementPolicy *replacementPolicy;
    std::vector<int>bit_waymask_partitions;

  public:
    /** Convenience typedef. */
     typedef BaseSetAssocParams Params;

    /**
     * Construct and initialize this tag store.
     */
    BaseSetAssoc(const Params *p);

    /**
     * Destructor
     */
    virtual ~BaseSetAssoc() {};

    /**
     * This function updates the tags when a block is invalidated. It also
     * updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;
    void invalidate_l2(uint16_t DSid, CacheBlk *blk) override;
    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache
     * access and should only be used as such. Returns the access latency as a
     * side effect.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param lat The access latency.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override
    {
        BlkType *blk = findBlock(addr, is_secure);

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                dataAccesses += 1;
            }
        } else {
            dataAccesses += allocAssoc;
        }

        if (blk != nullptr) {
            // If a cache hit
            lat = accessLatency;
            // Check if the block to be accessed is available. If not,
            // apply the accessLatency on top of block->whenReady.
            if (blk->whenReady > curTick() &&
                cache->ticksToCycles(blk->whenReady - curTick()) >
                accessLatency) {
                lat = cache->ticksToCycles(blk->whenReady - curTick()) +
                accessLatency;
            }

            // Update number of references to accessed block
            blk->refCount++;

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData);
        } else {
            // If a cache miss
            lat = lookupLatency;
        }
        /// @jia:
        if (blk != nullptr) {
            // move this block to head of the MRU list
            sets[blk->set].moveToHead(blk);
        }

        return blk;
    }

    CacheBlk* accessBlock_l2(uint16_t DSid, Addr addr, bool is_secure, Cycles &lat) override
    {
        BlkType *blk = findBlock_l2(DSid, addr, is_secure);
        // if(DSid == 0x0){
        //     lookupLatency = Cycles(0);
        //     accessLatency = Cycles(0);
        // }

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        /////////////////////////////////////
        uint16_t bit_waymask = 0xFFFF;
        bit_waymask = bit_waymask_partitions[DSid];
        // if(DSid == 0x0){
        //     bit_waymask = 0x00FF;
        // }else if (DSid == 0x1)
        // {
        //     bit_waymask = 0x1F00;
        // }else if (DSid == 0x2)
        // {
        //     bit_waymask = 0x1F00;
        // }else if (DSid == 0x3)
        // {
        //     bit_waymask = 0xE000;
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

    // }

        uint16_t maskA = 0x1;
        int index = 0;
        while(maskA){
            if ((maskA & bit_waymask) != 0)
            {
                index++;
            }
            maskA = maskA << 1;
        }
        tagAccesses += index;
        if (sequentialAccess) {
            if (blk != nullptr) {
                dataAccesses += 1;
            }
        } else {
            dataAccesses += index;
        }

        if (blk != nullptr) {
            // If a cache hit
            lat = accessLatency;
            // Check if the block to be accessed is available. If not,
            // apply the accessLatency on top of block->whenReady.
            if (blk->whenReady > curTick() &&
                cache->ticksToCycles(blk->whenReady - curTick()) >
                accessLatency) {
                lat = cache->ticksToCycles(blk->whenReady - curTick()) +
                accessLatency;
            }

            // Update number of references to accessed block
            blk->refCount++;

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData);
        } else {
            // If a cache miss
            lat = lookupLatency;
        }
        /// @jia:
        if (blk != nullptr) {
            // move this block to head of the MRU list
            sets[blk->set].moveToHead_l2(DSid, blk, bit_waymask_partitions);
        }

        return blk;
    }
    /**
     * Finds the given address in the cache, do not update replacement data.
     * i.e. This is a no-side-effect find of a block.
     * @param addr The address to find.
     * @param is_secure True if the target memory space is secure.
     * @param asid The address space ID.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* findBlock(Addr addr, bool is_secure) const override;

    CacheBlk* findBlock_l2(uint16_t DSid, Addr addr, bool is_secure) const override;

    /**
     * Find a block given set and way.
     *
     * @param set The set of the block.
     * @param way The way of the block.
     * @return The block.
     */
    ReplaceableEntry* findBlockBySetAndWay(int set, int way) const override;

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         std::vector<CacheBlk*>& evict_blks) const override
    {
        // Get possible locations for the victim block
        //std::vector<CacheBlk*> locations = getPossibleLocations(addr);

        // Choose replacement victim from replacement candidates
        // CacheBlk* victim = static_cast<CacheBlk*>(replacementPolicy->getVictim(
        //                        std::vector<ReplaceableEntry*>(
        //                            locations.begin(), locations.end())));

        /// @jia:
        int set = extractSet(addr);

        BlkType *victim = sets[set].blks[assoc - 1];

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        DPRINTF(CacheRepl, "set %x, way %x: selecting blk for replacement\n",
            victim->set, victim->way);

        return victim;
    }

    CacheBlk* findVictim_l2(uint16_t DSid, Addr addr, const bool is_secure,  // build/X86/mem/cache/tags/base_set_assoc.hh:
                         std::vector<CacheBlk*>& evict_blks) const override
    {
        // Get possible locations for the victim block
        // std::vector<CacheBlk*> locations = getPossibleLocations(addr);

        // // Choose replacement victim from replacement candidates
        // CacheBlk* victim = static_cast<CacheBlk*>(replacementPolicy->getVictim(
        //                        std::vector<ReplaceableEntry*>(
        //                            locations.begin(), locations.end())));
        /// @jia:
        int set = extractSet(addr);
        // grab a replacement candidate
        //represent the lower 4-bits
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
        BlkType *victim = sets[set].blks[index];
        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        DPRINTF(CacheRepl, "set %x, way %x: selecting blk for replacement\n",
            victim->set, victim->way);

        return victim;
    }
    /**
     * Find all possible block locations for insertion and replacement of
     * an address. Should be called immediately before ReplacementPolicy's
     * findVictim() not to break cache resizing.
     * Returns blocks in all ways belonging to the set of the address.
     *
     * @param addr The addr to a find possible locations for.
     * @return The possible locations.
     */
    const std::vector<CacheBlk*> getPossibleLocations(Addr addr) const
    {
        return sets[extractSet(addr)].blks;
    }

    // const std::vector<CacheBlk*> getPossibleLocations_l2(Addr addr, uint16_t DSid) const
    // {
    //     uint16_t bit_waymask = 0xFFFF;
    //     if(DSid == 0x0){
    //         bit_waymask = 0x000F;
    //     }else if (DSid == 0x1)
    //     {
    //         bit_waymask = 0x00F0;
    //     }

    //     return sets[extractSet(addr)].blks;
    // }

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override
    {
        // Insert block
        BaseTags::insertBlock(pkt, blk);

        // Increment tag counter
        tagsInUse++;

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData);
        /// @jia: moveTohead
        int set = extractSet(pkt->getAddr());
        sets[set].moveToHead(blk);
    }

    void insertBlock_l2(uint16_t DSid, const PacketPtr pkt, CacheBlk *blk) override  // build/X86/mem/cache/tags/base_set_assoc.hh:456
    {
        // Insert block
        BaseTags::insertBlock_l2(DSid, pkt, blk);

        // Increment tag counter
        tagsInUse++;

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData);
        /// @jia: moveTohead
        int set = extractSet(pkt->getAddr());
        sets[set].moveToHead_l2(DSid, blk, bit_waymask_partitions);
    }
    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Generate the tag from the given address.
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(Addr addr) const override
    {
        return (addr >> tagShift);
    }

    /**
     * Regenerate the block address from the tag and set.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return ((blk->tag << tagShift) | ((Addr)blk->set << setShift));
    }

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            visitor(blk);
        }
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            if (visitor(blk)) {
                return true;
            }
        }
        return false;
    }

  private:
    /**
     * Calculate the set index from the address.
     *
     * @param addr The address to get the set from.
     * @return The set index of the address.
     */
    int extractSet(Addr addr) const
    {
        return ((addr >> setShift) & setMask);
    }
};

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
