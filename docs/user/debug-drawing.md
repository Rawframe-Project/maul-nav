# Debug drawing

The library never draws. It fills an `mnavDebugBuffer` the host owns with vertices and two index lists, triangles and lines, which any renderer can draw.

- **Vertices** are three floats relative to the buffer's `origin`, a world position in doubles, so the precision is kept where the camera is. Set the origin near what is being looked at.
- **Kinds, not colors.** Each vertex carries what it shows (`mnavDebugKind`) and a value: the area type of a polygon's fill, whether tile links join a tile side, a link's kind with 256 added when it is detached or disabled, a polygon's place in a corridor, an open flight block's side. The host picks the colors.
- **Appending.** Every debug call appends, so one buffer can hold a frame's worth of drawing. A full buffer returns `mnav_errorCapacity` with the counts the whole would need; grow it to those counts and draw again.

| Function | Draws |
|---|---|
| `mnavDebugNavmesh` | Polygons by area, inner edges, walls, tile sides and tile bounds, over an `mnavTileRange` of tile places, or every tile for NULL. |
| `mnavDebugLinks` | Every off-mesh link as an arc, at its snapped ends when attached. |
| `mnavDebugPath`, `mnavDebugCorridor` | A path's straight line; a corridor's polygons. |
| `mnavDebugFlowField`, `mnavDebugNavFlow` | An arrow for each cell or polygon with a way on. |
| `mnavDebugAvoidance` | Each agent's outline and a line to each neighbor `mnavAvoid` would give it. |
| `mnavDebugAvoidance3D` | Each flier as three rings, its neighbors, and the spheres it avoids. |
| `mnavDebugFlight` | A flight volume's open blocks inside a box, as wire boxes. Solid space shows as gaps. |
| `mnavDebugFlightPath` | A flight path's steps. |

The avoidance drawings check their input as `mnavAvoid` and `mnavAvoid3D` do, so drawing what a step would refuse is refused too. A bake's own debug output is its [report](baking.md#the-report).
