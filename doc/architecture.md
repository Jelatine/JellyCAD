# JellyCAD architecture

JellyCAD uses one process and one script execution at a time. Lua APIs remain compatible; C++ geometry APIs use typed options instead of Lua tables.

## Build boundaries

| Target | Responsibility | Dependencies |
| --- | --- | --- |
| `jelly_geometry` | Shapes, transformations, geometry queries and CAD file I/O | OpenCASCADE, C++17 |
| `jelly_robotics` | Link/Joint model, validation, URDF/MJCF generation and transactional export | geometry |
| `jelly_lua_runtime` | Lua state, binding adapters and synchronous execution | robotics, geometry, Lua/sol2 |
| `jelly_application` | Execution lifecycle and preview scheduling | runtime, Qt Core |
| `jelly_services` | Git process queue and LLM transport/SSE decoding | Qt Core/Network |
| `jelly_desktop` | Qt widgets, editor and AIS visualization | application, services, Qt Widgets |
| `JellyCAD_cli` | Command-line execution without a GUI application | application |

The application and tests link the same libraries. Geometry and robotics headers must not include sol2 or Qt Widgets. Only the desktop owns AIS display objects. Lua bindings live in `src/runtime/jy_bindings.cpp`.

## Execution and cancellation

`RunRequest` carries an ID, source, source kind, execution directory and trigger. The GUI controller assigns monotonically increasing IDs. `RunResult` reports success, failure or cancellation and elapsed time, once per accepted request.

The synchronous runtime constructs and destroys its Lua state on the calling thread. The desktop controller runs it in a worker and moves through Idle, Running and Stopping states. Busy explicit requests are rejected without clearing the scene. Cancellation is per execution, checked by the Lua instruction hook and by queue producers. Native OCCT calls must return before cancellation completes; threads are never forcibly terminated.

Worker events go through a queue capped at 256 entries, drained in batches of at most 64 on a 16 ms GUI timer. Individual log entries are capped at 16 KiB and the terminal retains at most 5,000 text blocks. Shape snapshots deep-copy OCCT geometry and existing mesh data before crossing threads. AIS operations remain on the GUI thread; a batch triggers one refresh, and completion triggers one fit-to-view. Elapsed time, queue peak and slow display batches are logged locally.

Window close requests cancellation while continuing to process events, and closes after the worker has actually finished. The controller destructor also cancels and joins as a fallback; its producer cannot wait on GUI signal delivery.

Relative Lua file I/O remains supported by a scoped process working-directory change. A process-wide execution mutex prevents concurrent runtimes, and the original directory is restored on success and failure. Application file paths and Git command directories are absolute. Scripts remain trusted local programs, not a sandbox.

## Documents and generated code

F5 performs an atomic save if needed and explicitly submits exactly one run. Failed saves retain the dirty editor buffer. The status-bar Auto preview option defaults to enabled and is persisted in settings. File fingerprints suppress duplicate notifications and the app's own saves; a 200 ms debounce coalesces changes. While busy, only the newest pending preview survives. Changing documents drops the previous pending preview and cancels its active execution.

LLM chunks appear in a separate draft area. Only a successful, complete stream updates the editor, in one undo transaction. Cancellation, network failure, malformed/incomplete SSE and idle timeout leave the original document intact. SSE lines/events are buffered as bytes before UTF-8 JSON decoding. Replies are identified individually so an obsolete reply cannot complete a newer request. The default idle timeout is 60 seconds.

Git commands execute serially in the service. Each command captures its directory and workspace generation. Switching workspaces clears queued commands and ignores the active command's old result; an already-running Git operation is allowed to finish. Commands have a 120-second timeout.

## Robot exports

Export first validates names, joint limits, unique names and tree structure. A writer generates all candidate files under a unique sibling staging directory. Stream write errors are surfaced before installation.

A `.jellycad-files` manifest tracks generated files. Commit backs up affected files, installs new files and removes only obsolete manifest-owned files. A failure rolls back installed files and restores backups; if rollback itself fails, the backup directory is retained and reported. Files absent from both the old manifest and new output are preserved, including unrelated mesh files. Symlinks and directory/file conflicts in affected paths are rejected.

The multi-file commit recovers from reported I/O errors but is not crash-atomic across power loss. No claim of OS-level sandboxing or hard interruption of native computation is made.

## Verification

```sh
cmake --preset release --fresh -DBUILD_TESTS=ON
cmake --build --preset release
ctest --test-dir build/release -C Release --output-on-failure --no-tests=error
```

Tests cover geometry, Lua API compatibility using bundled examples, cancellation and full queues, working-directory restoration, preview scheduling, atomic saves, AI undo/failure behavior, SSE boundaries and transport failures, Git workspace isolation, and export rollback. GUI workflow tests use Qt's offscreen plugin and do not exercise the native OpenGL viewport. CMake requires GTest when tests are enabled; CI fails when no tests are discovered.
