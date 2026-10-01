# mnav-0009. Link generation

Status: Accepted

## Context

The requirements ask for automatic link generation. Unity, at bake,
connects ledges to the ground below (drop-down links landing
`2 * agentRadius + 4 * voxelSize` out, deeper than the step height and
no deeper than Drop Height) and gaps to ground across (jump-across
links from `2 * agentRadius` out to Jump Distance, within a voxel of
the start's height), checking that the way is unobstructed. Unreal
samples jump trajectories along navmesh border edges, validates them by
collision, and filters near-duplicates. Maul Nav's off-mesh links are
navmesh objects that snap, stream and stage; its runtime holds no
geometry.

## Decision

- **A pass over the navmesh:** `mnavGenerateLinks` reads the committed
  navmesh over a range of tile places and writes `mnavLinkDef`s into a
  caller's buffer, refusing a short one with the count; the host stages
  the links it wants. The order is tile place, polygon, edge, sample.
- **Edges and samples:** edges with no polygon across and no tile side
  are sampled every `spacing` meters, from a takeoff two cells inside
  each edge point, with the agent's radius, step height and cell size
  from the navmesh's settings.
- **Drops:** a landing `2 * radius + 4 * cellSize` out, on the highest
  surface from the step height to `dropMax` below; one way, or both
  ways when no deeper than `climbMax`.
- **Jumps:** a landing from `2 * radius` out to `jumpMax`, a cell at a
  time, at the first distance where a surface lies within the step
  height of the takeoff; one way, each side making its own.
- **What is kept:** a link the host's clearance callback passes, when
  given one; whose landing the navmesh does not already reach by a
  straight path within `detour` times the straight distance, which
  stands in for collision where the host gives none; and whose ends do
  not both lie within `filterDistance` of an earlier link's.
- Kinds (`mnav_linkDrop`, `mnav_linkJump` by default), costs and snap
  radius come from `mnavLinkGenDef`.

## Consequences

- The bake stays pure, and links follow streaming as the host's do.
- Without a clearance test, a jump over a wall that splits areas the
  navmesh does not join is made; the host's test is the remedy.
- The walking check uses the query context's limits; a way it cannot
  find within them keeps the link.
- With a short buffer, links past it are counted without the
  near-duplicate filter between them, so the count may exceed what a
  buffer large enough receives.
