---
name: monorepoClaude
description: E:\Projetos\Claude access policy - shared/ readable and replicable; third-party/ free to use without asking; other folders only with explicit consent; nothing there is ever modified
metadata:
  type: reference
---

`E:\Projetos\Claude` is a parallel project, never modified (not a single line).

- `shared\` (include, src, cmake, tools\ppCheck incl. ppCompile.exe / ppCheck.exe): read freely and REPLICATE into E:\Projetos\lidar as convenient. Replicated so far into `lidar\shared`: winTypes, alloc, mem, thread, fault, libDiscipline, macros, testCache, sha256, arg, idle (.h) + thread, fault, abNew, abPool, sha256 (.cpp).
- `third-party\` (android-ndk-r27c, androidSdk, jdk17, gradle-8.7, iosSdk, zig, stb, miniz...): use consented WITHOUT confirmation (user, 2026-09-26) - read/execute only, outputs always in `lidar\build`.
- Every other folder - `abCamera\`, `memory\`... - ONLY with the user's explicit consent, asked per use.
- Already known (read before the rule): abCamera's TMobile port (common app + thin TAndroid/TiOS), its bash+ppCompile mobile builds, Gradle only for packaging.

Related: regraOuro 3-4, [[missao]].
