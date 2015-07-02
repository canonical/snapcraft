# Part Flow

Each part has 3 major user-visible stages: build, stage, and snap.  Put respectively into parts/XXX, stage/, and snap/ directories.

## Build

There are actually several steps rolled into this one conceptual piece.

### Pull (parts/XXX/src)

This gets the source.

### Build (parts/XXX/build and parts/XXX/install)

This builds the source in build/ and does 'make install' or equivalent into install/.  Other parts that depend upon this one will be able to build against this part's install/ folder.

## Stage

This copies all files from parts/XXX/install into stage/ (or hardlinks them).  It complains if there would be any conflicts.

## Snap

This copies all files from stage/ to snap/ while filtering some files that shouldn't be delivered to end user.

## Assemble

This is a final last step, which basically runs "snappy build snap/"
