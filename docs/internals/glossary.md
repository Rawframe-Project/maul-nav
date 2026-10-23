# Glossary

Terms the code, the tests and the documentation use with one meaning.

- **Agent profile:** the radius, height, step height and steepest slope a navmesh is baked for. One profile per navmesh.
- **Area type:** a polygon's or cell's kind, 0 to 63. 0 is never walkable; 1 to 63 are kinds a filter costs and includes.
- **Bake:** turning host geometry into one tile's bytes. A **baker** holds a def, its memory and the last tile it baked.
- **Border:** the cells rasterized beyond a tile's sides, the agent's radius in cells and three more, so neighboring tiles agree on their shared side.
- **Cell:** the bake's grid unit, `cellSize` on the ground and `cellHeight` up. Settings in meters become cells.
- **Commit:** applying everything staged to a navmesh or flight volume at once, all or nothing. Queries see one commit at a time.
- **Corridor:** the polygons an agent's path runs through, kept in a buffer the host owns and moved as the agent moves.
- **Corners:** a corridor's straight path from the agent's position to its target.
- **Crossing:** one of the point links an edge link is staged as, spaced along its width.
- **Def:** the struct an object is made from, built by its `mnavDefault...Def` function, carrying a cookie and, for an owner object, an allocator and named limits. A root's def (a creation that takes no other library object, or `mnavSteer`) also carries the `MNAV_ABI_VERSION` stamp; the query filter and the link generation def carry only the cookie.
- **Detail mesh:** the small triangles laid over each polygon that follow the floor's heights.
- **Edge link:** an off-mesh link between two edges rather than two points.
- **Filter:** an agent's capabilities in a query: a cost per area type, the area types it may enter and the link kinds it may cross.
- **Fingerprint:** a tile's hash of everything that shaped it: the generator, the settings, its place and the input that reaches it.
- **Flight volume:** committed flight tiles, sparse voxel octrees of the space a flier's center may occupy.
- **Fragment:** what rasterization gives one cell for one triangle: the lowest and highest height of the triangle's part inside it.
- **Golden value:** a hash of a promised output, committed in a test.
- **Hierarchy:** clusters of tiles and the costs between their entrances, for long paths.
- **Layer region:** a region of open space that never lies over itself, so multi-floor buildings stay apart.
- **Limit:** a named bound in a def on work or memory, never on time. Reaching it is `mnav_errorLimit`.
- **Not loaded:** a place with no committed tile. Not a wall: nothing is known there.
- **Off-mesh link:** a way between two points the ground does not join (a jump, a ladder, a door), snapped to polygons at each commit.
- **Open space:** the walkable span tops of a tile's heightfield, linked to the spans an agent can step to.
- **Portal:** the part of an edge two polygons share, or a tile link's cell range, that a path crosses.
- **Query context:** the scratch memory of one thread's searches, sized by its limits.
- **Span:** a column's run of solid cells in a heightfield.
- **Stage:** to queue a change for the next commit. Staging changes nothing queries read.
- **Stale:** an id of a tile, polygon or link replaced or removed since it was handed out, or a search or field that outlived a commit.
- **Tier:** what a navmesh lets change once tiles are loaded: static, modifiers, dynamic.
- **Tile:** a square of `tileCells` cells baked on its own. Its place is its column and row from the def's origin.
- **Tile cache:** each tile's packed open space and volumes, kept to rebuild the tile with obstacles.
- **Tile index:** the triangles or outlines that may reach each tile, made once and shared by bakes.
- **Tile link:** a link between polygon edges on facing tile sides, made at a commit.
- **Transition:** a hierarchy's search node: the middle link of an entrance along a cluster border, or an off-mesh link between clusters.
