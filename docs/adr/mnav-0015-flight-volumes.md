# mnav-0015. Flight volumes

Status: Accepted

## Context

Agents that fly or swim are not bound to the ground, so a navmesh does
not describe where they may go.
- **Stacked navmeshes:** layers at chosen heights leave the number of
  layers unknowable in open volume.
- **A regular grid:** one cell per voxel is too large for a sky.
- **What games ship:** a sparse voxel octree. Warframe's flight, after
  Brewer ("3D Flight Navigation Using Sparse Voxel Octrees", Game AI Pro
  3), stores its layers in Morton order with six neighbor links per node
  and 4 by 4 by 4 voxel leaves in 64 bits.

Maul Nav's bake already rasterizes its input into columns of solid
spans, a run-length voxelization of the same geometry.

## Decision

- **An optional part:** flight volumes are a tile kind of their own.
  A caller who never flies never links them.
- **The bake:**
  - **Voxels:** the bake takes the navmesh bake's input (meshes and
    terrains) and rasterizes it with the same rasterizer at a cell
    size and cell height of one voxel, a cube of a size the caller
    sets. A voxel any triangle touches is solid.
  - **Below the ground:** by default each column is also solid from
    the volume's floor up to its lowest solid voxel, so that ground
    holds no open space beneath it. Open space such as a debris field
    turns this off.
  - **Clearance:** the agent is a sphere of its radius. A voxel within
    R = ceil(radius / voxel) of a solid voxel, by integer offsets with
    dx² + dy² + dz² ≤ R², is blocked. Each tile is rasterized with a
    border of R voxels so that its blocking matches its neighbors'.
- **Shape:**
  - A tile's side is 4 × 2^k voxels, 16 to 512.
  - The volume's vertical range, a floor and a ceiling in meters, is
    split into whole cubes of the tile's side, each an octree.
- **Layout:** a compact pointerless octree.
  - **Nodes:** a block holding both solid and open space ("mixed") is
    a node with a mask of its mixed children, a mask of its solid
    children and the index of its first stored child. Children are
    numbered x + 2y + 4z within their parent; only mixed children are
    stored, in that order.
  - **Order:** a cube's nodes are stored level by level from the
    root, so a node's stored children are contiguous in the next
    level.
  - **Leaves:** a mixed 4 by 4 by 4 block is a 64-bit leaf, bit
    x + 4y + 16z solid.
  - **Uniform blocks** are a bit in their parent's masks, whatever
    their size.
  - Neighbors are found by descending from the root, with no stored
    links.
- **Determinism:** tile bytes are little-endian and the same on every
  platform and at any worker count, as navmesh tiles are (mnav-0003).
- **Measured:** per 32 m tile with a 0.5 m agent, against Brewer's
  layout:
  - 6.4 KB on a debris field of boxes floating up to 256 m at 1 m
    voxels, against 75.7 KB;
  - 0.8 KB on the benchmark's terrain, against 7.6 KB.

- **The API** (`maul-nav/flight.h`):
  - **The def:** `mnavFlightDef` holds the cookie, allocator, origin,
    voxel size, tile side, floor, ceiling, flier radius, the
    ground-below switch, and limits on input triangles, spans, nodes
    and leaves per tile, tiles and memory.
  - **The baker:** `mnavCreateFlightBaker`, `mnavBakeFlightTile` over
    the navmesh bake's `mnavBakeInput`, and `mnavCopyFlightTile`. Its
    fingerprint covers the generator, the settings, the place and the
    input that reaches the tile, as mnav-0003's does.
  - **The container:** `mnavFlightVolume` stages tiles and removals
    and commits them all or nothing (`mnavCommitFlight`).
    `mnavGetFlightTile` reads a committed tile's fingerprint.
  - **The first query:** `mnavIsFlightOpen` tells whether a point is
    open to the flier's center.
- **The format:** magic `MNVF`, its own format version, little-endian.
  - **The header:** 104 bytes. It holds the generator, flags, the
    fingerprint, the payload's hash and size, the place, and the
    settings' bits: side, voxel, radius, floor, ceiling, origin,
    floor voxel and cubes. It also holds the node and leaf counts.
  - **The payload:** a class byte per cube, a 16-bit mask per node
    and a 64-bit word per leaf.
  - **Not stored:** child and root indices. In level order they are
    running sums of the masks, and the loader rebuilds them.
- **Loading:** every byte is read as hostile. The loader checks:
  - the counts against the def's limits before allocating;
  - the payload's hash;
  - the settings against the volume's def;
  - the canonical form: classes up to mixed, masks with no child both
    mixed and solid and never all empty or all solid, leaves neither 0
    nor all ones, and every count used exactly.

  A malformed tile is `mnav_errorInvalid`, naming the section and
  element. Another format version is `mnav_errorVersion`.

- **The queries:**
  - **`mnavFindFlightPath`:** Lazy Theta* over the open blocks of the
    committed tiles, of any size, and the open voxels of mixed leaves.
    - **The graph:** blocks are linked through shared faces, found by
      descending the octrees across cubes and tiles.
    - **Points:** each block is entered at the point of the shared face
      nearest the point the path comes from. The point lies 1/1024
      voxel inside the block and 4/1024 in from the face's edges, so
      that it never sits on a voxel's face or edge.
    - **Line of sight:** a point is in sight of another when the
      segment crosses no blocked voxel. Where it passes an edge or a
      corner, every voxel it touches counts.
    - **Repair:** a node out of sight of its assumed parent takes the
      node that reached it. If even that is out of sight, the path bends
      through the center of that node's block, which the block holds
      either way. Another closed neighbor in sight that is cheaper wins
      instead.
    - **The search state:** it runs in an `mnavQuery` context: its node
      budget, path length limit, memory and `mnavPathEnd`. Short of the
      end, the path runs to the point nearest it.
    - **Slices (amended 2026-10-09):** `mnavBeginFlightPath`,
      `mnavContinueFlightPath` and `mnavFinishFlightPath` run the
      search a number of nodes at a time, as the navmesh's searches do
      (mnav-0005), for hosts that bound the work per frame; every engine
      surveyed runs flight searches off the frame. The context keeps
      the search in plain data between slices, and the path is the one
      `mnavFindFlightPath` gives, which itself now runs as one slice.
      The volume counts its commits: a search continued or finished
      after a commit, or on another volume, is refused as stale.
  - **`mnavFlightRaycast`:** the same walk, stopping at the first
    blocked voxel or place with no tile.
  - **`mnavCheckFlightPath` (amended 2026-10-09):** the same walk over a
    path's steps in order, with one cursor, naming the first step
    blocked or with no tile and its hit. Every point is checked before
    any step is cast, so that a refusal changes nothing. A host checks
    what is left of a path after a commit and searches again from where
    its agent is; smoothing, local octree updates and a 3D funnel are
    left out (research 50 in the docs repository).
  - **`mnavFindNearestFlightPoint`:** branch and bound over the
    committed octrees within a radius.
  - **Drawing (amended 2026-10-09):** `mnavDebugFlight` draws the open
    blocks inside a world box as wire boxes (`mnav_debugFlightBlock`,
    the value the block's side), each once, walking the box a row at a
    time and skipping each open block's width; a box of more than
    `MNAV_MAX_FLIGHT_DEBUG_VOXELS` voxels within the floor and ceiling
    is refused. Open blocks show where fliers may go and how the octree
    merged; solid space shows as gaps. `mnavDebugFlightPath` draws a
    path's steps as `mnav_debugPath`.
- **Measured:** on the Warframe 3D benchmark (research in the docs
  repository), the path searches averaged 0.925 to 0.944 of the
  26-connected grid's shortest. They expanded 4 to 7 times fewer nodes
  than A* through block centers on all maps but the densest, C1, where
  line of sight dominates.

Later slices add fuzzing, benchmarks and the guide.

## Consequences

- One bake input serves both tiers. A world with fliers bakes each tile
  twice, once at the navmesh's cells and once at its voxels.
- Every neighbor step costs a descent of a few levels. The search's
  research weighs that against storing links.
- The sphere stands for every flier; long, flat fliers take the
  sphere that holds them.
