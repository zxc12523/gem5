/*
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

#include "cpu/pred/bit_level.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

namespace gem5
{
    namespace branch_prediction
    {
        BitlevelBP::BitlevelBP(const BitlevelBPParams &p) : BPredUnit(p), tables(8, Table(4096, 12)), BTB(4096),
                                                            localhistories(4096, 32), globalhistory(630), threshold(20)
        {
            segments.resize(8);
            segments[1] = {0, 13};
            segments[2] = {1, 33};
            segments[3] = {23, 49};
            segments[4] = {44, 85};
            segments[5] = {77, 159};
            segments[6] = {149, 270};
            segments[7] = {252, 630};
        }

        void BitlevelBP::btbUpdate(ThreadID tid, Addr branch_addr, void *&bp_history)
        {
        }

        // void BitlevelBP::squash(ThreadID tid, void *bp_history)
        // {
        // }

        void BitlevelBP::uncondBranch(ThreadID tid, Addr pc, void *&bp_history)
        {
            globalhistory.update(1);
            localhistories.update(pc, 1);
        }

        bool BitlevelBP::lookup(ThreadID tid, Addr branch_addr, void *&bp_history)
        {
            if (!BTB.seen(branch_addr))
            {
                return 0;
            }

            int sum = computeWeightedSum(branch_addr);

            if (abs(sum) < threshold)
            {
                return 0;
            }
            else
            {
                return sum >= 1;
            }

            return 0;
        }

        void BitlevelBP::update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history,
                                bool squashed, const StaticInstPtr &inst, Addr corrTarget)
        {
            for (int i = 0; i < tables.size(); i++)
            {
                unsigned int a;
                unsigned int key;

                if (i == 0)
                {
                    a = (unsigned int)branch_addr;
                    key = localhistories[branch_addr];
                }
                else
                {
                    a = (unsigned int)branch_addr;
                    key = globalhistory.hashSegment(segments[i]);
                }

                tables[i].update(hash(a, key), branch_addr);
            }

            BTB.update(branch_addr, corrTarget);
            globalhistory.update(taken);
            localhistories.update(branch_addr, taken);
        }

        int BitlevelBP::computeWeightedSum(Addr branch_addr)
        {
            int sum = 0;

            unsigned int a;
            unsigned int key;

            for (int i = 0; i < tables.size(); i++)
            {
                if (i == 0)
                {
                    a = (unsigned int)branch_addr;
                    key = localhistories[branch_addr];
                }
                else
                {
                    a = (unsigned int)branch_addr;
                    key = globalhistory.hashSegment(segments[i]);
                }

                sum += tables[i].computeWeight(hash(a, key), branch_addr);
            }

            return sum;
        }
    }
}