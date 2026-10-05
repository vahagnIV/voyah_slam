# third_party

## g2o

- Upstream: https://github.com/RainerKuemmerle/g2o
- Tag: `20241228_git`, commit `eec325a1da1273e87bc97887d49e70570f28570c`
- Vendored pristine in `third_party/g2o/src/` (no local edits; keep it that way).
- Conan package: `g2o/20241228@voyah/stable` (recipe: `third_party/g2o/conanfile.py`, which sets `user`/`channel`, so a plain `conan create` yields this reference).

### License

The core libraries we build (`stuff`, `core`, `types_*`, `solver_eigen`) are BSD-2-Clause.
Upstream's LGPL parts (CSparse/CHOLMOD solvers, `G2O_USE_LGPL_LIBS`, the OpenGL/GUI viewer)
and the `freeglut` bits are NOT built: the recipe disables CHOLMOD, CSparse, OpenGL, apps and
examples. The full upstream `LICENSE` is in `g2o/src/` and is packaged under `licenses/`.

### Recipe options

`shared` (default False), `fPIC` (default True). Eigen comes from Conan (`eigen/3.4.0`,
declared with `transitive_headers=True` because g2o headers include Eigen).

Upstream CMake variables set by the recipe (verified against the tag's CMakeLists.txt):
`BUILD_SHARED_LIBS` (from `shared`), `BUILD_UNITTESTS=OFF`, `G2O_USE_CHOLMOD=OFF`,
`G2O_USE_CSPARSE=OFF`, `G2O_USE_OPENGL=OFF`, `G2O_USE_LOGGING=OFF`, `G2O_BUILD_APPS=OFF`,
`G2O_BUILD_EXAMPLES=OFF`, `G2O_BUILD_BENCHMARKS=OFF`.

Deviation from the spec: upstream has no `BUILD_TESTING` switch; the equivalent is
`BUILD_UNITTESTS` (default OFF, set explicitly).

### Consumed targets

CMake config name `g2o` (`find_package(g2o REQUIRED)`). Upstream's own installed CMake config
is not used; the recipe declares components instead. All names match the plan; library files
are `libg2o_<name>.a` with no build-type suffix.

| Target                | Library             | Requires                                  |
|-----------------------|---------------------|-------------------------------------------|
| `g2o::stuff`          | `g2o_stuff`         | `eigen::eigen` (+ `rt` on Linux)          |
| `g2o::core`           | `g2o_core`          | `stuff`, `eigen::eigen`                   |
| `g2o::types_slam3d`   | `g2o_types_slam3d`  | `core`                                    |
| `g2o::types_sba`      | `g2o_types_sba`     | `core`, `types_slam3d`                    |
| `g2o::solver_eigen`   | `g2o_solver_eigen`  | `core`                                    |

Headers are included as `g2o/core/...`, `g2o/types/sba/...`, `g2o/solvers/eigen/...`.
Other upstream libraries get built and packaged but are not exposed as components; add one to
`package_info` if needed.

### Build

```
conan create third_party/g2o --build=missing -s build_type=Debug
conan create third_party/g2o --build=missing -s build_type=Release
```

`test_package` builds a Levenberg-Marquardt optimizer with `BlockSolver_6_3` +
`LinearSolverEigen` and runs `initializeOptimization()` on a one-edge graph.

### Updating

Replace `third_party/g2o/src/` with the new upstream tag (no `.git`), bump `version` in
`conanfile.py`, update the tag/commit above (and `conanfile.txt` of the project), re-check the
option names and library names, then re-run both `conan create` commands.
