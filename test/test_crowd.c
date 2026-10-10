// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The neighbour grid of avoidance (mnav-0006): on the ground plane and in
// space, its neighbours are those a search of every agent finds, ties and
// shared spots included, and agents an agent ignores neither found nor
// counted against the limit.

#include "crowd.h"
#include "test_harness.h"

#include "maul-nav/base.h"

#include <stdint.h>

enum
{
    AGENTS = 600,
    LIMIT = 12
};

static uint64_t s_state = 0x2545F4914F6CDD1Dull;

// A draw in [0, 1), splitmix64's.
static double Random(void)
{
    s_state += 0x9E3779B97F4A7C15ull;
    uint64_t z = s_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return (double)(z >> 11) * 0x1p-53;
}

static mnavCrowdKey s_keys[AGENTS];
static mnavCrowdKey s_scratch[AGENTS];
static mnavCrowdRun s_table[2048];
static mnavCrowdNeighbor s_neighbors[LIMIT];
static mnavPos3 s_positions[AGENTS];
static uint64_t s_ids[AGENTS];
static uint32_t s_layers[AGENTS];
static uint32_t s_ignores[AGENTS];

static bool Before(const mnavCrowdNeighbor* a, const mnavCrowdNeighbor* b)
{
    if (a->distance != b->distance)
    {
        return a->distance < b->distance;
    }
    return a->id != b->id ? a->id < b->id : a->index < b->index;
}

// The neighbours of agent i by looking at every agent: the nearest within
// range, the lowest ids and indices first among equals.
static int32_t Brute(int32_t i, double range, bool space, mnavCrowdNeighbor* list)
{
    int32_t count = 0;
    for (int32_t j = 0; j < AGENTS; ++j)
    {
        double dx = s_positions[j].x - s_positions[i].x;
        double dy = s_positions[j].y - s_positions[i].y;
        double dz = space ? s_positions[j].z - s_positions[i].z : 0.0;
        double distance = dx * dx + dy * dy + dz * dz;
        if (j == i || distance >= range * range || (s_layers[j] & s_ignores[i]) != 0)
        {
            continue;
        }
        mnavCrowdNeighbor candidate = {distance, s_ids[j], j};
        if (count == LIMIT && !Before(&candidate, &list[count - 1]))
        {
            continue;
        }
        int32_t k = count < LIMIT ? count++ : count - 1;
        while (k > 0 && Before(&candidate, &list[k - 1]))
        {
            list[k] = list[k - 1];
            k -= 1;
        }
        list[k] = candidate;
    }
    return count;
}

// Agents in a box, on a coarse lattice so that many share a distance or a
// spot, and ids that repeat; with layers, each in one of three and
// ignoring any of the first two.
static void Scatter(bool space, double extent, bool layers)
{
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        double x = (double)(int32_t)(Random() * 16.0) * extent / 16.0;
        double y = (double)(int32_t)(Random() * 16.0) * extent / 16.0;
        double z = space ? (double)(int32_t)(Random() * 16.0) * extent / 16.0 : 0.0;
        s_positions[i] = (mnavPos3){x - extent / 2.0, y - extent / 2.0, z};
        s_ids[i] = (uint64_t)(Random() * 200.0);
        s_layers[i] = layers ? 1u << (uint32_t)(Random() * 3.0) : 0u;
        s_ignores[i] = layers ? (uint32_t)(Random() * 4.0) : 0u;
    }
}

// Compares the grid's neighbours with a search of every agent, the ground
// cells divisions to the range.
static void Compare(bool space, double extent, double range, bool layers, int32_t divisions)
{
    Scatter(space, extent, layers);
    mnavCrowd crowd = {s_keys, s_scratch, s_table, 0, s_neighbors, LIMIT, range,
                       space,  divisions, 0,       0};
    mnavBeginCrowd(&crowd, space);
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        s_keys[i] = mnavCrowdKeyOf(&crowd, s_positions[i], s_ids[i], i);
        s_keys[i].layers = s_layers[i];
    }
    mnavSortCrowd(&crowd, AGENTS);
    int32_t mismatches = 0;
    for (int32_t i = 0; i < AGENTS; ++i)
    {
        mnavCrowdNeighbor expected[LIMIT];
        int32_t want = Brute(i, range, space, expected);
        int32_t got = mnavCrowdNeighbors(&crowd, s_positions[i], i, s_ignores[i]);
        bool same = got == want;
        for (int32_t k = 0; same && k < got; ++k)
        {
            same = s_neighbors[k].index == expected[k].index &&
                   s_neighbors[k].distance == expected[k].distance;
        }
        mismatches += same ? 0 : 1;
    }
    CHECK(mismatches == 0, "the neighbours of a search of every agent");
}

// The next call's ground cells: kept while the probes held two to six
// agents each, otherwise sized for three, within two to six per range;
// space keeps two.
static void Held(mnavCrowd* crowd, bool space, int64_t probed, int64_t scanned)
{
    mnavBeginCrowd(crowd, space);
    crowd->probed = probed;
    crowd->scanned = scanned;
    mnavEndCrowd(crowd);
}

static void TestDivisions(void)
{
    mnavCrowd crowd = {0};
    mnavBeginCrowd(&crowd, false);
    CHECK(crowd.divisions == 4, "four cells per range at first");
    Held(&crowd, false, 1000, 700);
    CHECK(crowd.divisions == 2, "sparse agents, larger cells");
    Held(&crowd, false, 1000, 2700);
    CHECK(crowd.divisions == 2, "within the band, kept");
    Held(&crowd, false, 1000, 11700);
    CHECK(crowd.divisions == 3, "dense agents, smaller cells");
    Held(&crowd, false, 1000, 8800);
    CHECK(crowd.divisions == 5, "denser still");
    Held(&crowd, false, 1000, 3300);
    CHECK(crowd.divisions == 5, "settled");
    // Sized again, 2.2 a probe would make four; within the band, kept.
    Held(&crowd, false, 1000, 2200);
    CHECK(crowd.divisions == 5, "a little sparser, kept");
    Held(&crowd, false, 10, 100000);
    CHECK(crowd.divisions == 6, "at most six");
    Held(&crowd, false, 0, 0);
    CHECK(crowd.divisions == 6, "nothing probed, unchanged");
    Held(&crowd, true, 1000, 100);
    CHECK(crowd.divisions == 6, "space leaves the ground's cells");
}

int main(void)
{
    CHECK(mnavCrowdTableSize(AGENTS) <= 2048, "the table fits");
    // Every ground division the cells may take finds the same neighbours.
    for (int32_t d = 2; d <= 6; ++d)
    {
        Compare(false, 20.0, 3.0, false, d);
        Compare(false, 200.0, 10.0, false, d);
        Compare(false, 20.0, 3.0, true, d);
    }
    Compare(true, 20.0, 3.0, false, 0);
    Compare(true, 60.0, 10.0, false, 0);
    Compare(true, 6.0, 1.0, false, 0);
    Compare(true, 20.0, 3.0, true, 0);
    TestDivisions();
    return s_failures == 0 ? 0 : 1;
}
