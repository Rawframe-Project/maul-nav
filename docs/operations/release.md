# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning from 1.0.0: a feature bumps the minor version and a fix the patch. There is no 2.0.0: a change that would break 1.x is designed to be additive or does not ship. A minor release may grow a def, so the ABI holds within a minor version: the shared library's SOVERSION is major.minor, and the CMake package accepts any later 1.z (SameMajorVersion). Tags are `vX.Y.Z`.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul-nav/base.h`, where CMake reads it, updates `.release-please-manifest.json`, and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles and the `BREAKING CHANGE:` footers.
2. Rewrite that entry in user language, in `CHANGELOG.md` on the release pull request's branch (`release-please--branches--main--components--maul-nav`): what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security.
   - A breaking change comes first under Changed, prefixed **Breaking:**, and says what a caller must change.
   - Name every change of results (tile bytes, fingerprints, paths, velocities), since hosts store tiles and compare runs, and every new tile or flight tile field, which earlier releases refuse.
   - User documentation (the README and `docs/user/`) gets one short line under Changed when it changes what a user can find or do.
   - Leave out refactors, tests, CI, and the internals and operations documentation.
3. Put the same entry in the pull request's body, in place of the notes between release-please's `---` lines: release-please publishes the GitHub release notes from the body, not from `CHANGELOG.md`. release-please rewrites both whenever anything else lands on `main`, so the curation is the last change before the merge, and nothing else merges to `main` until the release is published. Curate only after the `release` run for `main`'s latest push has finished, and do not dispatch `release.yml` afterwards: a run in flight force-updates the release branch and writes the body again over the curation.
4. Check that the `required` CI job passed on the release pull request, that the last nightly run of `ci.yml` on `main` passed (the long fuzzing and the large scenes), and that the tag `vX.Y.Z` does not exist yet (`git ls-remote --tags origin vX.Y.Z` prints nothing).
5. Mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the body's notes. The Pages workflow publishes the documentation from `main`.
6. The `assets` job of `.github/workflows/release.yml` then attaches the source archive `maul-nav-X.Y.Z.tar.gz` (the tagged tree), its `SHA256SUMS`, an SPDX SBOM and a build provenance attestation. Check that they are on the release; a user verifies a download with `sha256sum -c SHA256SUMS` and `gh attestation verify maul-nav-X.Y.Z.tar.gz -R Rawframe-Project/maul-nav`.

## A release made by hand

The Release app cannot create a release on a commit whose workflows differ from the head of `main`. When release-please fails to publish for that reason, a maintainer finishes the release by hand:

1. Create the tag `vX.Y.Z` on the release pull request's merge commit, and the GitHub release on it with the curated notes.
2. Move the release pull request's label from `autorelease: pending` to `autorelease: tagged`, so that release-please does not try again.
3. Run the `release` workflow from `main` with the input `tag` set to `vX.Y.Z` (`gh workflow run release.yml --ref main -f tag=vX.Y.Z`). Its `assets` job builds, attests and attaches the files as after an ordinary release. Check them as in step 6.

The `assets` job refuses, before it builds anything:
- a tag that is not `vX.Y.Z`;
- a run that release-please gave a release other than the tag;
- a tag on a commit that `main` does not hold, since the attestation names this workflow on `main`;
- a release that already has files.

It builds from the commit it resolved the tag to and checks that the checkout is that commit, so a tag moved during the run changes nothing it attaches. It never replaces a published file: a run that failed after attaching some files is re-run only after a maintainer has deleted those files from the release by hand.

## Notes

- A change of results is never a patch release. A pull request that gives existing inputs a different result carries `feat` and states the change in its body, whatever else it does; nothing breaks the API, and only a minor release grows a def.
- `MNAV_TILE_FORMAT` and `MNAV_FLIGHT_FORMAT` no longer change. New tile or flight tile data is a new field with a new tag, never required: a 1.z library reads every tile a 1.y wrote, giving an absent field its default, and an earlier release refuses the new tag with `mnav_errorVersion`. A field whose meaning changes gets a new tag. `test/stored_tiles.h` keeps its tiles in the formats 1.0.0 writes, so the tests show that a later release still loads them.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- A library that pins this one moves its pin in a pull request of its own.
