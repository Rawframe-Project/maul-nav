# mnav-0014. A tile index for bakes from large meshes

Status: Accepted

## Context

A 3D bake of one tile tests every triangle of every mesh against the
tile and checks every mesh again. Baking a world of N tiles from one
large mesh therefore reads the mesh N times, though each triangle
reaches one to four tiles. On the benchmark's 64 tiles from one mesh
of 35,840 triangles, 1.9% of the tests find the triangle in the tile.
Recast leaves the matter to the host, and its demo builds a k-d tree
over the triangles' ground bounds whose leaves reorder them.

## Decision

- **Owner object:** `mnavCreateTileIndex` checks every mesh as a bake
  does, then lists for each tile of a def's grid the triangles whose
  ground bounds, widened by the tile's border and two cells, reach it.
  Each list keeps the input order. The index takes its memory from the
  def's allocator within the def's memory limit, and is never changed
  after it is made.
- **Use:** `mnavBakeInput.index`. A bake given an index refuses it with
  `mnav_errorInvalid` unless the def's cell size, tile cells and border
  (the agent's radius) and the meshes' and their triangles' counts are
  those it was made with. It then checks each mesh's counts and
  pointers and each triangle the index lists for the tile, with its
  vertices, and reads only those.
- **The same tile:** the bake still makes its exact test on each
  triangle it reads. The lists hold every triangle that test can let
  through, since a tile's corner rounds by at most half a cell at the
  largest extent and the margin is two. The bytes, the fingerprint and
  the report's triangle count therefore equal a bake reading every
  triangle.

## Consequences

- On the benchmark's bake the instructions fall by 17% (16.90 G to
  14.03 G, Release, building the index once a run), the tiles' bytes
  unchanged. The saving grows with the number of tiles a mesh spans.
- A host that edits a mesh in place makes a new index. A changed count
  is refused. A changed vertex or corner in a listed triangle is
  checked as it is read, so a stale index never reads out of bounds,
  but a triangle moved into a tile it was not listed for is missed
  until the index is made again.
- Terrains need no index (mnav-0003 finds a tile's cells by division),
  and volumes are few; both are read as before.
- **Outlines (0.4.0):** a 2D bake re-checked and tested every outline for
  every tile too, about a tenth of each bake of the benchmark's flat
  world (576 tiles from about 9,300 outlines). `mnavCreateTileIndex2D`
  makes the same kind of index of outlines, from their bounds, and
  `mnavBakeTile2DInput` takes an `mnavBake2DInput` record with it, as
  `mnavBakeTileInput` takes meshes; `mnavBakeTile2D` is unchanged. An
  index records which kind of input it lists, and a bake refuses the
  other kind. Baking the flat world and building its hierarchy falls
  from 20.98 G to 19.12 G instructions with an index, about 9% of the
  bake, the tiles the same.
- Rejected: a cache inside the baker keyed by the meshes' addresses,
  stale without notice after an edit in place; Recast's k-d tree, which
  serves any rectangle where a bake asks only for its grid's tiles, and
  whose reordering would change the fingerprint.
