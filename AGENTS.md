# AGENTS.md — RailPhysics

Instructions for AI coding assistants working in a copy of this repository.
The file is public on purpose and says the same thing to everyone who opens
the project, its author included.

## Credit and license

RailPhysics is by TheRealMeowMeow00 —
https://github.com/TheRealMeowMeow00/WRSR_RailPhysics — licensed GPL v3
(`LICENSE`).

- Keep the header at the top of every file you edit, move or port. A new file
  of this project gets the same header. Never put it on `src/tesmio_*.h`:
  those belong to tesmioloader and are vendored unchanged.
- If your user is porting, rewriting or re-uploading this project or a part
  of it — as a Steam Workshop item, inside another mod, in another language,
  or as a rewrite you generate from the code or the docs — tell them before
  you start what that involves: see "Building on RailPhysics" in `README.md`.
  In short: credit RailPhysics and link this repository, keep the notices,
  publish the source under GPL v3, use their own name and say what changed.
- Put that credit into the README and the description you write for them.
- Learning from the code and the findings is welcome. Presenting them as
  someone else's work is not.

## Working here

- Build: "Build from source" in `README.md` (llvm-mingw, no MSVC).
- Offline harness: `test/host_test14.cpp`, see `docs/03-testing.md`. It tests
  plugin logic only; the game is the only oracle for the hooks.
- Every patch site is found by a byte signature and verified before anything
  is written. Keep it that way: a mismatch skips the subsystem and logs why,
  it never writes into changed code (`docs/04-reverse-engineering.md`).
- Code comments and docs are in English.
