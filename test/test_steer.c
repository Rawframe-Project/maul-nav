// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Steering on corners written by hand: straight at the next
// corner, swinging wide of a turn, slowing within the slowing distance
// of the target along the corners, stopping at the target or at a link's
// takeoff, passing a corner straight above the agent; the refusals.

#include "test_harness.h"

#include "maul-nav/base.h"
#include "maul-nav/query.h"

#include <math.h>
#include <stdint.h>

static mnavPos3 At(double x, double z)
{
    return (mnavPos3){x, 0.0, z};
}

static mnavCorners Corners(const mnavPos3* points, int32_t count)
{
    return (mnavCorners){points, count, nullptr, 0};
}

// Within binary32's rounding of a velocity of a few meters a second.
static bool Near(double a, double b)
{
    return fabs(a - b) < 1e-6;
}

static mnavSteering Steer(const mnavCorners* corners, const mnavSteerDef* def)
{
    mnavSteering s = {{-1.0, -1.0, -1.0}, 99, 99};
    CHECK(mnavSteer(corners, def, &s) == mnav_success, "steered");
    return s;
}

static void TestMoving(void)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    const mnavPos3 straight[2] = {At(0, 0), At(10, 0)};
    mnavCorners c = Corners(straight, 2);
    mnavSteering s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && s.link == -1 && Near((double)s.velocity.x, 3.5) &&
              (double)s.velocity.y == 0.0 && (double)s.velocity.z == 0.0,
          "straight at the next corner at full speed");
    // A turn ahead: toward (4, 0) less the direction to (4, 4) times 2.
    const mnavPos3 turn[3] = {At(0, 0), At(4, 0), At(4, 4)};
    c = Corners(turn, 3);
    s = Steer(&c, &def);
    double x = 4.0 - sqrt(0.5) * 2.0;
    double z = -sqrt(0.5) * 2.0;
    double length = hypot(x, z);
    CHECK(Near((double)s.velocity.x, 3.5 * x / length) &&
              Near((double)s.velocity.z, 3.5 * z / length),
          "swinging wide of the turn, away from it");
    def.enableTurnAnticipation = false;
    s = Steer(&c, &def);
    CHECK(Near((double)s.velocity.x, 3.5) && (double)s.velocity.z == 0.0,
          "without anticipation, straight");
    // A corner straight above the agent, at a step, is passed.
    const mnavPos3 step[3] = {At(0, 0), {0.0, 0.3, 0.0}, At(0, 5)};
    c = Corners(step, 3);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && (double)s.velocity.x == 0.0 &&
              Near((double)s.velocity.z, 3.5),
          "past a corner above the agent");
}

static void TestArriving(void)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    // 0.7 m left along the corners, inside the 1 m slowing distance.
    const mnavPos3 bend[3] = {At(0, 0), At(0.3, 0), At(0.3, 0.4)};
    mnavCorners c = Corners(bend, 3);
    mnavSteering s = Steer(&c, &def);
    double speed = hypot((double)s.velocity.x, (double)s.velocity.z);
    CHECK(s.state == mnav_steerMoving && Near(speed, 3.5 * 0.7), "slowing by the distance left");
    const mnavPos3 near[2] = {At(0, 0), At(0.1, 0)};
    c = Corners(near, 2);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerArrived && (double)s.velocity.x == 0.0 && s.link == -1,
          "within the arrival distance: arrived");
    c = Corners(near, 1);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerArrived, "at the target");
    def.arriveDistance = 0.25f;
    const mnavPos3 edge[2] = {At(0, 0), At(0.25, 0)};
    c = Corners(edge, 2);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerArrived, "exactly the arrival distance away: arrived");
    def.arriveDistance = 0.1f;
    def.slowDistance = 0.0f;
    const mnavPos3 short2[2] = {At(0, 0), At(0.2, 0)};
    c = Corners(short2, 2);
    s = Steer(&c, &def);
    CHECK(Near((double)s.velocity.x, 3.5), "no slowing: full speed to the end");
}

static void TestLinks(void)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    const mnavPos3 jump[4] = {At(0, 0), At(3, 0), At(5, 2), At(8, 2)};
    const mnavPathLink links[1] = {{{1, 1}, mnav_linkJump, 1}};
    mnavCorners c = {jump, 4, links, 1};
    mnavSteering s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && Near((double)s.velocity.x, 3.5) &&
              (double)s.velocity.z == 0.0,
          "straight at a takeoff, never swinging wide of it");
    const mnavPos3 there[4] = {At(2.95, 0), At(3, 0), At(5, 2), At(8, 2)};
    c.points = there;
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerAtLink && s.link == 0 && (double)s.velocity.x == 0.0,
          "within the arrival distance of the takeoff: at the link");
    // A link further on: steering as usual.
    const mnavPathLink later[1] = {{{1, 1}, mnav_linkJump, 2}};
    c = (mnavCorners){there, 4, later, 1};
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving, "a link two corners on");
}

static void TestRefusals(void)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    const mnavPos3 points[2] = {At(0, 0), At(1, 0)};
    mnavCorners c = Corners(points, 2);
    mnavSteering s;
    CHECK(mnavSteer(nullptr, &def, &s) == mnav_errorInvalid &&
              mnavSteer(&c, nullptr, &s) == mnav_errorInvalid &&
              mnavSteer(&c, &def, nullptr) == mnav_errorInvalid,
          "NULL arguments");
    mnavCorners none = Corners(points, 0);
    mnavCorners missing = Corners(nullptr, 2);
    mnavCorners unlinked = {points, 2, nullptr, 1};
    CHECK(mnavSteer(&none, &def, &s) == mnav_errorInvalid &&
              mnavSteer(&missing, &def, &s) == mnav_errorInvalid &&
              mnavSteer(&unlinked, &def, &s) == mnav_errorInvalid,
          "no points, points or links missing");
    // A link count below 0, with links or without.
    const mnavPathLink links[1] = {{{1, 1}, mnav_linkJump, 1}};
    mnavCorners negative = {points, 2, links, -1};
    mnavCorners negativeNull = {points, 2, nullptr, -1};
    CHECK(mnavSteer(&negative, &def, &s) == mnav_errorInvalid &&
              mnavSteer(&negativeNull, &def, &s) == mnav_errorInvalid,
          "a link count below 0");
    mnavSteerDef bad = def;
    bad.cookie += 1;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorInvalid, "a def not from the default");
    bad = def;
    bad.maxSpeed = -1.0f;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorInvalid, "a negative speed");
    bad.maxSpeed = MNAV_MAX_STEER_SPEED * 2.0f;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorLimit, "too fast");
    bad.maxSpeed = INFINITY;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorLimit, "an infinite speed");
    bad = def;
    bad.slowDistance = (float)NAN;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorInvalid, "a slowing distance of NaN");
    bad = def;
    bad.arriveDistance = MNAV_MAX_STEER_DISTANCE * 2.0f;
    CHECK(mnavSteer(&c, &bad, &s) == mnav_errorLimit, "an arrival distance too long");
    const mnavPos3 nan[2] = {At(0, 0), At((double)NAN, 0)};
    c = Corners(nan, 2);
    CHECK(mnavSteer(&c, &def, &s) == mnav_errorInvalid, "a corner not finite");
}

// Corners far apart but finite: the distance between them overflows no
// double, so the velocity is the def's speed along them, never NaN or
// zero.
static void TestFarCorners(void)
{
    mnavSteerDef def = mnavDefaultSteerDef();
    const mnavPos3 across[2] = {At(-1e308, 0), At(1e308, 0)};
    mnavCorners c = Corners(across, 2);
    mnavSteering s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && Near((double)s.velocity.x, 3.5) &&
              (double)s.velocity.z == 0.0,
          "corners 2e308 apart: full speed along them");
    const mnavPos3 far[2] = {At(0, 0), At(1e200, 0)};
    c = Corners(far, 2);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && Near((double)s.velocity.x, 3.5),
          "corners 1e200 apart: full speed, not zero");
    const mnavPos3 turn[3] = {At(1e300, 1e300), At(2e300, 1e300), At(2e300, 2e300)};
    c = Corners(turn, 3);
    s = Steer(&c, &def);
    double speed = sqrt((double)s.velocity.x * (double)s.velocity.x +
                        (double)s.velocity.z * (double)s.velocity.z);
    CHECK(s.state == mnav_steerMoving && isfinite(speed) && Near(speed, 3.5) &&
              (double)s.velocity.x > 0.0,
          "a turn far out: anticipated, finite, at full speed");
    // A near corner before a far one is measured as it is: no distance
    // between near corners vanishes because another lies far out.
    const mnavPos3 nearThenFar[3] = {At(0, 0), At(10, 0), At(10, 1e200)};
    c = Corners(nearThenFar, 3);
    mnavSteerDef straight = def;
    straight.enableTurnAnticipation = false;
    s = Steer(&c, &straight);
    CHECK(s.state == mnav_steerMoving && Near((double)s.velocity.x, 3.5) &&
              (double)s.velocity.z == 0.0,
          "a near corner before a far one: straight at it");
    // Anticipating, it swings wide as it does with the far corner nearer.
    const mnavPos3 nearThenNearer[3] = {At(0, 0), At(10, 0), At(10, 1e100)};
    mnavCorners nearer = Corners(nearThenNearer, 3);
    mnavSteering wide = Steer(&nearer, &def);
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && (double)s.velocity.x > 0.0 && (double)s.velocity.z < 0.0 &&
              Near((double)s.velocity.x, (double)wide.velocity.x) &&
              Near((double)s.velocity.z, (double)wide.velocity.z),
          "a near corner before a far one, anticipated: as with the far corner at 1e100");
    // A takeoff 10 m away is no arrival at the link.
    const mnavPathLink takeoff[1] = {{{1, 1}, mnav_linkJump, 1}};
    c = (mnavCorners){nearThenFar, 3, takeoff, 1};
    s = Steer(&c, &def);
    CHECK(s.state == mnav_steerMoving && s.link == -1 && Near((double)s.velocity.x, 3.5) &&
              (double)s.velocity.z == 0.0,
          "a takeoff 10 m away before a far corner: straight at it, not at the link");
}

int main(void)
{
    TestMoving();
    TestArriving();
    TestLinks();
    TestRefusals();
    TestFarCorners();
    return s_failures == 0 ? 0 : 1;
}
