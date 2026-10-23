# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning; before 1.0.0, any release may change the API, the ABI and the tile formats. Tags are `vX.Y.Z`.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul-nav/base.h`, where CMake reads it, updates `.release-please-manifest.json`, and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles.
2. Rewrite that entry in user language before merging: what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security. Leave out refactors, tests, CI and documentation. Say what a caller must change for a breaking change, and name every change of results (tile bytes, fingerprints, paths, velocities), since hosts store tiles and compare runs.
3. Check that the `required` CI job passed on the release pull request.
4. Mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the changelog entry as its notes. The Pages workflow publishes the documentation from `main`.
5. The `assets` job of `.github/workflows/release.yml` then attaches the source archive `maul-nav-X.Y.Z.tar.gz`, its `SHA256SUMS`, an SPDX SBOM and a build provenance attestation. Check that they are on the release; a user verifies a download with `sha256sum -c SHA256SUMS` and `gh attestation verify maul-nav-X.Y.Z.tar.gz -R Rawframe-Project/maul-nav`.

## Notes

- A change to the layout of tile or flight tile bytes bumps `MNAV_TILE_FORMAT` or `MNAV_FLIGHT_FORMAT`, so an older tile is refused with `mnav_errorVersion` instead of misread.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- A library that pins this one moves its pin in a pull request of its own.
