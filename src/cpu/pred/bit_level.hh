// #include <vector>

// #include "base/sat_counter.hh"
// #include "base/types.hh"
// #include "cpu/pred/bpred_unit.hh"
// #include "params/BitLevelBP.hh"

// using namespace gem5
// {
//     using namespace branch_prediction
//     {
//         class BitLevelBP : public BPredUnit
//         {
//         public:
//             BitLevelBP(const )
//         }
//     }
// }

/*
 * Copyright (c) 2011, 2014 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 */

#ifndef __CPU_PRED_2BIT_LOCAL_PRED_HH__
#define __CPU_PRED_2BIT_LOCAL_PRED_HH__

#include <vector>

#include "base/sat_counter.hh"
#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/BitlevelBP.hh"

namespace gem5
{

    namespace branch_prediction
    {
        class BitlevelBP : public BPredUnit
        {
        public:
            /**
             * Default branch predictor constructor.
             */
            BitlevelBP(const BitlevelBPParams &params);

            virtual void uncondBranch(ThreadID tid, Addr pc, void *&bp_history);

            /**
             * Looks up the given address in the branch predictor and returns
             * a true/false value as to whether it is taken.
             * @param branch_addr The address of the branch to look up.
             * @param bp_history Pointer to any bp history state.
             * @return Whether or not the branch is taken.
             */
            bool lookup(ThreadID tid, Addr branch_addr, void *&bp_history);

            /**
             * Updates the branch predictor to Not Taken if a BTB entry is
             * invalid or not found.
             * @param branch_addr The address of the branch to look up.
             * @param bp_history Pointer to any bp history state.
             * @return Whether or not the branch is taken.
             */
            void btbUpdate(ThreadID tid, Addr branch_addr, void *&bp_history);

            /**
             * Updates the branch predictor with the actual result of a branch.
             * @param branch_addr The address of the branch to update.
             * @param taken Whether or not the branch was taken.
             */
            void update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history,
                        bool squashed, const StaticInstPtr &inst, Addr corrTarget);

            void squash(ThreadID tid, void *bp_history)
            {
                assert(bp_history == NULL);
            }

        private:
            /**
             *  Returns the taken/not taken prediction given the value of the
             *  counter.
             *  @param count The value of the counter.
             *  @return The prediction based on the counter value.
             */
            unsigned int threshold;

            inline bool getPrediction(uint8_t &count);

            inline int computeWeightedSum(Addr branch_addr);

            std::vector<std::pair<int, int>> segments;

            class Table
            {
            private:
                unsigned numberOfWeights;
                unsigned table_size;
                std::vector<std::vector<int>> table;

                unsigned int index(unsigned int hashValue) const
                {
                    return hashValue % table_size;
                }

            public:
                Table(int size, int bits) : numberOfWeights(bits), table_size(size),
                                            table(size, std::vector<int>(bits, 3)) {}

                std::vector<int> operator[](unsigned int hashValue) const
                {
                    return table[index(hashValue)];
                }

                int computeWeight(unsigned int hashValue, Addr targetAddr)
                {
                    std::vector<int> &weights = table[index(hashValue)];

                    int x = 0;

                    for (int i = 0; i < numberOfWeights; i++)
                    {
                        x += weights[i] * ((targetAddr >> i) & 1);
                    }

                    return x;
                }

                void update(unsigned hashValue, Addr branch_addr)
                {
                    std::vector<int> &weights = table[index(hashValue)];

                    for (int i = 0; i < numberOfWeights; i++)
                    {
                        if (branch_addr & (1 << i))
                        {
                            weights[i] += 1;
                        }
                        else
                        {
                            weights[i] -= 1;
                        }
                    }
                }
            };

            class BranchTargetBuffer
            {
            private:
                std::vector<Addr> btb;

                unsigned int index(Addr pc) const
                {
                    return (pc >> 2) % btb.size();
                }

            public:
                BranchTargetBuffer(int size) : btb(size, 0) {}

                Addr operator[](Addr pc) const
                {
                    return btb[index(pc)];
                }

                void update(Addr pc, Addr branchAddr)
                {
                    btb[index(pc)] = branchAddr;
                }

                bool seen(Addr pc)
                {
                    return btb[index(pc)] != 0;
                }
            };

            class LocalHistories
            {
            private:
                /** The array of histories */
                std::vector<unsigned int> localHistories;
                /** Size in bits of each history entry */
                const int localHistoryLength;

                /** Index function given the pc of the branch */
                unsigned int index(Addr pc) const
                {
                    return (pc >> 2) % localHistories.size();
                }

            public:
                LocalHistories(int nlocal_histories, int histo_len) : localHistories(nlocal_histories, 0), localHistoryLength(histo_len) {}

                /** Obtains the local history entry of a given branch */
                unsigned int operator[](Addr pc) const
                {
                    return localHistories[index(pc)];
                }

                /** Adds a history bit to the local history entry of a given branch */
                void update(Addr pc, bool value)
                {
                    assert(localHistories.size() > 0);
                    unsigned int &pos = localHistories[index(pc)];
                    pos <<= 1;
                    pos |= value;
                    pos &= ((1 << localHistoryLength) - 1);
                }

                /** Returns the number of bits of each local history entry */
                int getLocalHistoryLength() const
                {
                    return localHistoryLength;
                }

                /** Size in bits required by all history entries */
                int getSize() const
                {
                    return localHistoryLength * localHistories.size();
                }
            };

            class GlobalHistory
            {
            private:
                int block_size = 32;
                int history_length;
                std::vector<unsigned int> ghist_words;

            public:
                GlobalHistory(int histo_len) : history_length(histo_len), ghist_words((histo_len - 1) / block_size + 1, 0) {}

                void update(int branchOutcome)
                {
                    unsigned int total_blocks = (history_length - 1) / block_size + 1;

                    unsigned int next_bit = 0;
                    unsigned int new_bit = branchOutcome;

                    for (int block = 0; block < total_blocks; block++)
                    {
                        next_bit = (ghist_words[block] & (1 << (block_size - 1))) != 0;
                        ghist_words[block] <<= 1;
                        ghist_words[block] |= new_bit;
                        new_bit = next_bit;
                    }

                    unsigned int last_block = total_blocks - 1;
                    unsigned int mask = (1 << (((history_length - 1) % block_size) + 1)) - 1;

                    ghist_words[last_block] &= mask;
                }

                unsigned int hashSegment(const std::pair<int, int> &segment)
                {
                    int start_pos = segment.first;
                    int end_pos = segment.second;

                    int start_block = start_pos / block_size;
                    int end_block = end_pos / block_size + 1;

                    unsigned int x = 0;

                    x += ghist_words[start_block] >> (start_pos % block_size);

                    for (int block = start_block + 1; block < end_block - 1; block++)
                    {
                        x += ghist_words[block];
                    }

                    if (end_block > start_block + 1)
                    {
                        end_block--;

                        x += ghist_words[end_block] & ((1 << ((end_pos % block_size) + 1)) - 1);
                    }

                    return x;
                }
            };

            struct ThreadData
            {
                /* data */
                ThreadData(int table_size, int table_bits, int BTB_size,
                           int localhistories_size, int localhistories_len,
                           int globalhistory_len);
            };

            std::vector<Table> tables;
            BranchTargetBuffer BTB;
            LocalHistories localhistories;
            GlobalHistory globalhistory;

            static inline unsigned int hash1(unsigned int a)
            {
                a = (a ^ 0xdeadbeef) + (a << 4);
                a = a ^ (a >> 10);
                a = a + (a << 7);
                a = a ^ (a >> 13);
                return a;
            }

            static inline unsigned int hash2(unsigned int key)
            {
                int c2 = 0x27d4eb2d; // a prime or an odd constant
                key = (key ^ 61) ^ (key >> 16);
                key = key + (key << 3);
                key = key ^ (key >> 4);
                key = key * c2;
                key = key ^ (key >> 15);
                return key;
            }

            static inline unsigned int hash(unsigned int key, unsigned int i)
            {
                return hash1(key) ^ hash2(i);
            }
        };

    } // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_2BIT_LOCAL_PRED_HH__
