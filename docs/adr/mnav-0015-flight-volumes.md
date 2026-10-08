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

Later slices add the volume's container, its streaming and its queries,
and amend this record.

## Consequences

- One bake input serves both tiers. A world with fliers bakes each tile
  twice, once at the navmesh's cells and once at its voxels.
- Every neighbor step costs a descent of a few levels. The search's
  research weighs that against storing links.
- The sphere stands for every flier; long, flat fliers take the
  sphere that holds them.
