# Documentation

## Using the library

- [Getting started](user/getting-started.md): requirements, building, installing, the samples, and the rules every part of the API follows.
- [Baking](user/baking.md): defs, input, many tiles, the report, obstacles through the tile cache.
- [The navmesh](user/navmesh.md): loading, streaming, ids, tiers, off-mesh links and link generation.
- [Paths and queries](user/paths.md): nearest points, path searches, filters, sliced and hierarchical searches, spatial queries, grids, corridors and steering.
- [Crowds and avoidance](user/crowds.md): avoidance alone, a crowd on a navmesh, fliers.
- [Flow fields](user/flow-fields.md): over grids and over the navmesh.
- [Flight volumes](user/flight.md): baking, loading and querying space for fliers.
- [Debug drawing](user/debug-drawing.md).
- [Troubleshooting](user/troubleshooting.md): what each result means, and common surprises.
- [API reference](https://rawframe-project.github.io/maul-nav/api.html), generated from the headers.

## How it works

- [Overview](internals/overview.md): the library profile and how the parts fit.
- [Glossary](internals/glossary.md).
- [Determinism](internals/determinism.md).
- [Bake](internals/bake.md), [Navmesh](internals/navmesh.md), [Queries](internals/queries.md), [Hierarchy](internals/hierarchy.md), [Flow fields](internals/flow-fields.md), [Avoidance](internals/avoidance.md), [Flight volumes](internals/flight.md).
- [Maul conventions](internals/maul-conventions.md): the rules every Maul library follows.

## Maintaining it

- [Development](operations/development.md): toolchain, build options, running tests, fuzzing, the differential harness, benchmarks.
- [Checks](operations/checks.md): what verifies a change, locally and in CI.
- [Release](operations/release.md).
