# Flight volumes

Agents that fly or swim need the free space of a volume, not a surface. Stacked navmeshes at chosen heights leave the number of layers unknowable in open volume, and a regular grid of voxels is too large for a sky. What games ship is a sparse voxel octree; Warframe's flight, after Brewer ("3D Flight Navigation Using Sparse Voxel Octrees", Game AI Pro 3), stores its layers in Morton order with six neighbor links per node. Maul Nav's flight tiles are octrees too, laid out more compactly, and are an optional tile kind a program that never flies never links.

## Contents

- [The bake](#the-bake)
- [The layout](#the-layout)
- [The format and loading](#the-format-and-loading)
- [The search](#the-search)
- [Rays, checks and nearest points](#rays-checks-and-nearest-points)

## The bake

`mnavBakeFlightTile` (`src/flight_baker.c`, `src/flight.c`) takes the navmesh bake's input and rasterizes it with the same rasterizer at a cell size and height of one voxel. A voxel any triangle touches is solid. With `groundBelow`, each column is also solid from the volume's floor up to its lowest solid voxel, so ground holds no open space beneath it. The flier is a sphere: a voxel within R = ceil(radius / voxel) of a solid voxel, by integer offsets with dx² + dy² + dz² ≤ R², is blocked. Each tile is rasterized with a border of R voxels, so its blocking matches its neighbors'.

A tile's side is 4 × 2^k voxels, 16 to 512, and the volume's height range is split into whole cubes of that side, each an octree. Bytes are the same on every platform and at any worker count, and the fingerprint covers the generator, the settings, the place and the input that reaches the tile, as a navmesh tile's does.

## The layout

A compact octree with no pointers (`src/flight_tile.c`):

- A block holding both solid and open space ("mixed") is a node: a mask of its mixed children and a mask of its solid children. Children are numbered x + 2y + 4z within their parent, and only mixed children are stored, in that order.
- A cube's nodes are stored level by level from the root, so a node's stored children are contiguous in the next level and its first child's index is a running sum of the masks before it. Nothing else is stored.
- A mixed block of 4 by 4 by 4 voxels is a 64-bit leaf, bit x + 4y + 16z solid. Uniform blocks of any size are a bit in their parent's masks.
- Neighbors are found by descending from the root, with no stored links. Every neighbor step costs a descent of a few levels, against a layout several times smaller.

Per 32 m tile with a 0.5 m flier: 6.4 KB on a debris field of boxes floating up to 256 m at 1 m voxels, against 75.7 KB in Brewer's layout; 0.8 KB on the benchmark's terrain, against 7.6 KB.

## The format and loading

Magic `MNVF` and a format version of its own (`MNAV_FLIGHT_FORMAT`), little-endian. A 104-byte header holds the generator, flags, the fingerprint, the payload's hash and size, the place, the settings' bits and the node and leaf counts. The payload is a class byte per cube, a 16-bit mask per node and a 64-bit word per leaf; child and root indices are not stored, and the loader rebuilds them.

The loader reads every byte as hostile: counts against the def's limits before allocating, the payload's hash, the settings against the volume's def, and the canonical form (classes up to mixed, masks with no child both mixed and solid and never all empty or all solid, leaves neither empty nor full, every count used exactly). A tile has one canonical encoding, so two bakes of the same space are the same bytes, and a tile that is not canonical is refused, naming the section and element.

## The search

`mnavFindFlightPath` (`src/flight_search.c`) runs Lazy Theta* over the open blocks of the committed tiles, of any size, and the open voxels of mixed leaves.

- Blocks are linked through shared faces, found by descending the octrees across cubes and tiles.
- Each block is entered at the point of the shared face nearest the point the path comes from, 1/1024 voxel inside the block and 4/1024 in from the face's edges, so that it never sits on a voxel's face or edge, where a line of sight would be ambiguous.
- A point is in sight of another when the segment crosses no blocked voxel; where it passes an edge or a corner, every voxel it touches counts.
- A node out of sight of its assumed parent takes the node that reached it; if even that is out of sight, the path bends through the center of that node's block, which the block holds either way. Another closed neighbor in sight that is cheaper wins instead.
- The search runs in an `mnavQuery` context with its node budget and path length. Sliced, it keeps its state in plain data between slices, and `mnavFindFlightPath` itself runs as one slice, so slicing cannot change the path. The volume counts its commits, and a search across a commit is stale.

On the Warframe 3D benchmark maps, its paths averaged 0.925 to 0.944 of the 26-connected grid's shortest, and it expanded 4 to 7 times fewer nodes than A* through block centers on all maps but the densest, where line of sight dominates.

## Rays, checks and nearest points

- `mnavFlightRaycast` is the same walk as the line of sight, stopping at the first blocked voxel, the floor or ceiling, or a place with no tile.
- `mnavCheckFlightPath` walks a path's steps in order with one cursor and names the first step blocked or with no tile. Every point is checked before any step is cast, so a refusal changes nothing. A host checks what is left of a path after a commit and searches again; smoothing, local octree updates and a 3D funnel are left out.
- `mnavFindNearestFlightPoint` is a branch and bound over the committed octrees within a radius.
- `mnavDebugFlight` (`src/flight_debug.c`) draws the open blocks in a box, each once, walking the box a row at a time and skipping each open block's width.
