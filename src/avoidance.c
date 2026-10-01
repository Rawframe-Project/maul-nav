// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Avoidance sets (mnav-0006): each agent's ORCA lines against its nearest
// neighbours (RVO2 src/Agent.cc, computeNewVelocity, in binary64), its
// share of each avoidance set by priority, and neighbours found through a
// grid sorted by cell and id, so that the agents' order never matters.

#include "maul-nav/avoidance.h"

#include "allocator.h"
#include "orca.h"

#include "maul-nav/base.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stdint.h>

// Marks a def built by mnavDefaultAvoidanceDef.
#define AVOIDANCE_DEF_COOKIE 0x4E415641u

// The largest coordinate an agent may have, in meters: grid cells stay
// well inside 64-bit integers.
#define MAX_COORDINATE 1.0e12

// How far right of its preferred velocity a held-back agent aims, as a
// fraction of its speed.
#define KEEP_RIGHT 0.01

// An agent's grid cell, id and index, sorted to find neighbours.
typedef struct Key
{
    int64_t x;
    int64_t y;
    uint64_t id;
    int32_t index;
} Key;

// A neighbour: its squared distance, id and index.
typedef struct Neighbor
{
    double distance;
    uint64_t id;
    int32_t index;
} Neighbor;

struct mnavAvoidance
{
    mnavMemory memory;
    mnavAvoidanceDef def;
    Key* keys;
    Key* scratch;
    Neighbor* neighbors;
    mnavLine* lines;
    mnavLine* projected;
};

mnavAvoidanceDef mnavDefaultAvoidanceDef(void)
{
    return (mnavAvoidanceDef){AVOIDANCE_DEF_COOKIE, {0}, {4096, 10}, 10.0, 2.0};
}

static bool Positive(double v)
{
    return isfinite(v) && v > 0.0;
}

mnavResult mnavCreateAvoidance(const mnavAvoidanceDef* def, mnavAvoidance** avoidanceOut)
{
    if (avoidanceOut == nullptr)
    {
        return mnav_errorInvalid;
    }
    *avoidanceOut = nullptr;
    if (def == nullptr || def->cookie != AVOIDANCE_DEF_COOKIE ||
        (def->allocator.alloc == nullptr) != (def->allocator.free == nullptr))
    {
        return mnav_errorInvalid;
    }
    const mnavAvoidanceLimits* limits = &def->limits;
    if (limits->agents < 1 || limits->agents > MNAV_MAX_AVOIDANCE_AGENTS || limits->neighbors < 1 ||
        limits->neighbors > MNAV_MAX_AVOIDANCE_NEIGHBORS || !Positive(def->neighborDistance) ||
        !Positive(def->timeHorizon))
    {
        return mnav_errorRange;
    }
    mnavMemory memory = mnavMakeMemory(def->allocator, UINT64_MAX);
    mnavAvoidance* a = nullptr;
    mnavResult result =
        mnavAllocate(&memory, 1, sizeof(mnavAvoidance), alignof(mnavAvoidance), (void**)&a);
    if (result != mnav_success)
    {
        return result;
    }
    *a = (mnavAvoidance){0};
    a->memory = memory;
    a->def = *def;
    size_t agents = (size_t)limits->agents;
    size_t neighbors = (size_t)limits->neighbors;
    result = mnavAllocate(&a->memory, agents, sizeof(Key), alignof(Key), (void**)&a->keys);
    if (result == mnav_success)
    {
        result = mnavAllocate(&a->memory, agents, sizeof(Key), alignof(Key), (void**)&a->scratch);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&a->memory, neighbors, sizeof(Neighbor), alignof(Neighbor),
                              (void**)&a->neighbors);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&a->memory, neighbors, sizeof(mnavLine), alignof(mnavLine),
                              (void**)&a->lines);
    }
    if (result == mnav_success)
    {
        result = mnavAllocate(&a->memory, neighbors, sizeof(mnavLine), alignof(mnavLine),
                              (void**)&a->projected);
    }
    if (result != mnav_success)
    {
        mnavDestroyAvoidance(a);
        return result;
    }
    *avoidanceOut = a;
    return mnav_success;
}

void mnavDestroyAvoidance(mnavAvoidance* avoidance)
{
    if (avoidance == nullptr)
    {
        return;
    }
    mnavMemory memory = avoidance->memory;
    size_t agents = (size_t)avoidance->def.limits.agents;
    size_t neighbors = (size_t)avoidance->def.limits.neighbors;
    mnavRelease(&memory, avoidance->keys, agents, sizeof(Key), alignof(Key));
    mnavRelease(&memory, avoidance->scratch, agents, sizeof(Key), alignof(Key));
    mnavRelease(&memory, avoidance->neighbors, neighbors, sizeof(Neighbor), alignof(Neighbor));
    mnavRelease(&memory, avoidance->lines, neighbors, sizeof(mnavLine), alignof(mnavLine));
    mnavRelease(&memory, avoidance->projected, neighbors, sizeof(mnavLine), alignof(mnavLine));
    mnavRelease(&memory, avoidance, 1, sizeof(mnavAvoidance), alignof(mnavAvoidance));
}

static bool FinitePos(mnavPos2 p)
{
    return isfinite(p.x) && isfinite(p.y);
}

static bool GoodAgent(const mnavAgent* agent)
{
    return FinitePos(agent->position) && FinitePos(agent->velocity) &&
           FinitePos(agent->preferred) && fabs(agent->position.x) <= MAX_COORDINATE &&
           fabs(agent->position.y) <= MAX_COORDINATE && Positive(agent->radius) &&
           isfinite(agent->maxSpeed) && agent->maxSpeed >= 0.0 && Positive(agent->priority);
}

static bool KeyBefore(const Key* a, const Key* b)
{
    if (a->x != b->x)
    {
        return a->x < b->x;
    }
    if (a->y != b->y)
    {
        return a->y < b->y;
    }
    return a->id != b->id ? a->id < b->id : a->index < b->index;
}

// Sorts the keys, a stable bottom-up merge through the scratch.
static void SortKeys(Key* keys, Key* scratch, int32_t count)
{
    Key* from = keys;
    Key* to = scratch;
    for (int32_t width = 1; width < count; width *= 2)
    {
        for (int32_t start = 0; start < count; start += 2 * width)
        {
            int32_t middle = start + width < count ? start + width : count;
            int32_t end = start + 2 * width < count ? start + 2 * width : count;
            int32_t i = start;
            int32_t j = middle;
            for (int32_t k = start; k < end; ++k)
            {
                bool left = i < middle && (j >= end || !KeyBefore(&from[j], &from[i]));
                to[k] = left ? from[i++] : from[j++];
            }
        }
        Key* swap = from;
        from = to;
        to = swap;
    }
    for (int32_t k = 0; from != keys && k < count; ++k)
    {
        keys[k] = from[k];
    }
}

// The first key at or after cell (x, y).
static int32_t FirstAt(const Key* keys, int32_t count, int64_t x, int64_t y)
{
    int32_t low = 0;
    int32_t high = count;
    while (low < high)
    {
        int32_t middle = low + (high - low) / 2;
        bool before = keys[middle].x < x || (keys[middle].x == x && keys[middle].y < y);
        low = before ? middle + 1 : low;
        high = before ? high : middle;
    }
    return low;
}

static bool NeighborBefore(const Neighbor* a, const Neighbor* b)
{
    if (a->distance != b->distance)
    {
        return a->distance < b->distance;
    }
    return a->id != b->id ? a->id < b->id : a->index < b->index;
}

// Keeps a candidate among the nearest, up to the limit; returns the count.
static int32_t Insert(Neighbor* list, int32_t count, int32_t limit, Neighbor candidate)
{
    if (count == limit && !NeighborBefore(&candidate, &list[count - 1]))
    {
        return count;
    }
    int32_t i = count < limit ? count++ : count - 1;
    while (i > 0 && NeighborBefore(&candidate, &list[i - 1]))
    {
        list[i] = list[i - 1];
        i -= 1;
    }
    list[i] = candidate;
    return count;
}

static int64_t CellOf(double v, double size)
{
    return (int64_t)floor(v / size);
}

// The neighbours of agent i, nearest first.
static int32_t Neighbors(mnavAvoidance* a, const mnavAgent* agents, int32_t count, int32_t i)
{
    double range = a->def.neighborDistance;
    const mnavAgent* self = &agents[i];
    int64_t cx = CellOf(self->position.x, range);
    int64_t cy = CellOf(self->position.y, range);
    int32_t found = 0;
    for (int64_t x = cx - 1; x <= cx + 1; ++x)
    {
        for (int32_t k = FirstAt(a->keys, count, x, cy - 1);
             k < count && a->keys[k].x == x && a->keys[k].y <= cy + 1; ++k)
        {
            int32_t j = a->keys[k].index;
            double dx = agents[j].position.x - self->position.x;
            double dy = agents[j].position.y - self->position.y;
            double distance = dx * dx + dy * dy;
            if (j != i && distance < range * range)
            {
                found = Insert(a->neighbors, found, a->def.limits.neighbors,
                               (Neighbor){distance, agents[j].id, j});
            }
        }
    }
    return found;
}

// The ORCA line of agent self against other: the half-plane of its
// velocities that, with other doing its share, avoid colliding within
// the horizon, or for agents already overlapping, parting within the
// step. self's share is other's priority over the sum of both.
static mnavLine AgentLine(const mnavAgent* self, const mnavAgent* other, double horizon,
                          double step)
{
    mnavPos2 position = {other->position.x - self->position.x,
                         other->position.y - self->position.y};
    mnavPos2 velocity = {self->velocity.x - other->velocity.x,
                         self->velocity.y - other->velocity.y};
    double distance = position.x * position.x + position.y * position.y;
    double combined = self->radius + other->radius;
    double combinedSq = combined * combined;
    double share = other->priority / (self->priority + other->priority);
    mnavLine line;
    mnavPos2 u;
    if (distance > combinedSq)
    {
        double inverse = 1.0 / horizon;
        mnavPos2 w = {velocity.x - inverse * position.x, velocity.y - inverse * position.y};
        double wSq = w.x * w.x + w.y * w.y;
        double dot = w.x * position.x + w.y * position.y;
        if (dot < 0.0 && dot * dot > combinedSq * wSq)
        {
            // Onto the cut-off circle.
            double length = sqrt(wSq);
            mnavPos2 unit = {w.x / length, w.y / length};
            line.direction = (mnavPos2){unit.y, -unit.x};
            double by = combined * inverse - length;
            u = (mnavPos2){by * unit.x, by * unit.y};
        }
        else
        {
            // Onto a leg.
            double leg = sqrt(distance - combinedSq);
            if (position.x * w.y - position.y * w.x > 0.0)
            {
                line.direction = (mnavPos2){(position.x * leg - position.y * combined) / distance,
                                            (position.x * combined + position.y * leg) / distance};
            }
            else
            {
                line.direction =
                    (mnavPos2){-(position.x * leg + position.y * combined) / distance,
                               -(-position.x * combined + position.y * leg) / distance};
            }
            double along = velocity.x * line.direction.x + velocity.y * line.direction.y;
            u = (mnavPos2){along * line.direction.x - velocity.x,
                           along * line.direction.y - velocity.y};
        }
    }
    else
    {
        // Overlapping: part within the step.
        double inverse = 1.0 / step;
        mnavPos2 w = {velocity.x - inverse * position.x, velocity.y - inverse * position.y};
        double length = sqrt(w.x * w.x + w.y * w.y);
        // Same place, same velocity: the lower id goes one way and the
        // other the opposite.
        mnavPos2 unit = length > 0.0 ? (mnavPos2){w.x / length, w.y / length}
                                     : (mnavPos2){self->id < other->id ? -1.0 : 1.0, 0.0};
        line.direction = (mnavPos2){unit.y, -unit.x};
        double by = combined * inverse - length;
        u = (mnavPos2){by * unit.x, by * unit.y};
    }
    line.point = (mnavPos2){self->velocity.x + share * u.x, self->velocity.y + share * u.y};
    return line;
}

mnavResult mnavAvoid(mnavAvoidance* avoidance, const mnavAgent* agents, int32_t agentCount,
                     double step, mnavPos2* velocitiesOut)
{
    if (avoidance == nullptr || agentCount < 0 ||
        (agentCount > 0 && (agents == nullptr || velocitiesOut == nullptr)) || !Positive(step))
    {
        return mnav_errorInvalid;
    }
    if (agentCount > avoidance->def.limits.agents)
    {
        return mnav_errorLimit;
    }
    double range = avoidance->def.neighborDistance;
    for (int32_t i = 0; i < agentCount; ++i)
    {
        if (!GoodAgent(&agents[i]))
        {
            return mnav_errorInvalid;
        }
        avoidance->keys[i] = (Key){CellOf(agents[i].position.x, range),
                                   CellOf(agents[i].position.y, range), agents[i].id, i};
    }
    SortKeys(avoidance->keys, avoidance->scratch, agentCount);
    for (int32_t i = 0; i < agentCount; ++i)
    {
        const mnavAgent* self = &agents[i];
        int32_t count = Neighbors(avoidance, agents, agentCount, i);
        for (int32_t n = 0; n < count; ++n)
        {
            avoidance->lines[n] = AgentLine(self, &agents[avoidance->neighbors[n].index],
                                            avoidance->def.timeHorizon, step);
        }
        mnavPos2 velocity = {0.0, 0.0};
        int32_t failed = mnavLinearProgram2(avoidance->lines, count, self->maxSpeed,
                                            self->preferred, false, &velocity);
        if (failed == count && (velocity.x != self->preferred.x || velocity.y != self->preferred.y))
        {
            // Held back: aim a little right of the preferred velocity, so
            // that agents meeting in perfect symmetry pass on their right
            // rather than stop face to face.
            mnavPos2 biased = {self->preferred.x + KEEP_RIGHT * self->preferred.y,
                               self->preferred.y - KEEP_RIGHT * self->preferred.x};
            failed = mnavLinearProgram2(avoidance->lines, count, self->maxSpeed, biased, false,
                                        &velocity);
        }
        if (failed < count)
        {
            mnavLinearProgram3(avoidance->lines, count, 0, failed, self->maxSpeed,
                               avoidance->projected, &velocity);
        }
        velocitiesOut[i] = velocity;
    }
    return mnav_success;
}
