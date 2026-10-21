# Generated kernels: sympy → Rust → WebAssembly

**Status:** implemented for the square Newton path. `otkd emit --format wasm` writes the
kernel; `tests/test_wasm.py` builds it and checks it against the Python engine.

The short version: the package already derives the residual and its Jacobian symbolically. This
document's pipeline takes those *expressions*, factors them into a straight-line program, prints
that program as Rust, compiles it to WebAssembly, and hands it to a page. The page can then
solve the mechanism with no server, no Python and no network — and it is solving the same
model, because the kernel was generated from the same derivation.

```
  topology.yaml ─┐
  geometry.yaml ─┼─► Problem ─► Φ(q,g,t), J = ∂Φ/∂q ─► CSE ─► Rust ─► wasm ─► kernel.js
  study.yaml    ─┤   (sympy)        expressions        circuit   source   module    (base64)
  constraints ──┘                                           │
                                                            └── one f64 buffer, no allocator
```

---

## 1. What a "kernel" is, in mathematics

The compiled problem is a map and its derivative:

```
Φ : Rⁿ × R^G × R^A → R^m           Φ(q, g, t) = 0
J = ∂Φ/∂q : Rⁿ × R^G × R^A → R^{m×n}
```

For the wishbone `n = m = 21` (three moving bodies × 7 unknowns), `G = 67` geometry numbers,
`A = 1` drive. Apart from the distance row's `sqrt` and the driver row's `atan2`, both maps are
**polynomial**.

A kernel is what you get when you stop treating those as *expressions* and start treating them
as an **arithmetic circuit**. Common-subexpression elimination (CSE) rewrites the expanded
polynomials as a sequence of scalar assignments:

```
w₀ = x·y
w₁ = w₀·y            // x·y²
w₂ = √w₀
r₁ = w₀ + w₁ + w₂
r₂ = w₁ − 3
```

Two rows, five nodes, no repetition. Mathematically nothing changed — `r` is the same map — but
the *representation* is now a straight line: no allocation, no interpreter, no branches. That is
the whole idea.

Measured on the wishbone model in this repository:

| | as expressions | after CSE |
|---|---|---|
| residual `Φ`, 21 rows | 11 313 operations | **1 337 nodes, ~7 000 operations** |
| Jacobian `J`, 441 entries | 29 653 operations | included above |
| non-zero Jacobian entries | — | **161 of 441** |

and the compiled result:

| | value |
|---|---|
| generated Rust | ~1 400 statements, 180 KB of source |
| `kernel.wasm` | 74 KB, 34 KB gzipped |
| **one Newton frame** | **0.09 ms** (Python: 7 ms — about 75×) |
| solving the geometry (`pack`) | 0.005 ms |
| Jacobian agreement with Python | exact (0 difference) |

The speed comes from removing overhead, not from doing less arithmetic: 21-element `numpy` calls
cost microseconds each in dispatch and allocation, while the same flops in a compiled loop are
free. Sparsity helps too — the kernel only touches the 161 non-zero Jacobian entries.

---

## 2. sympy → Rust

sympy has a Rust printer, and it handles everything this package needs:

```python
sp.rust_code(sp.sqrt(x**2 + y**2) + sp.atan2(y, x) + x**3)
# "(x.powi(2) + y.powi(2)).sqrt() + y.atan2(x) + x.powi(3)"
```

`sqrt` becomes a method, `atan2` becomes a method on the numerator, integer powers become
`.powi(n)`, and `cos`/`sin` are available for free. The printers for C, C++, Fortran, Julia and
Octave exist as well; `sympy.utilities.codegen` and `autowrap` exist but are aimed at f2py and
Cython, not at WebAssembly, so this package does its own statement emission (about 60 lines).

**The one trap: integer literals.** sympy prints `2*x`, and Rust refuses to multiply an integer
literal by an `f64`:

```
error[E0277]: cannot multiply `{integer}` by `f64`
```

So the printer output is post-processed to promote literals — `2*x` becomes `2.0*x` — while
skipping the two places where an integer is required: the argument of `.powi(...)` and array
indices. A regex cannot do that safely; a small character walk can, and it is in
`emitters/wasm.py`.

CSE itself is `sympy.simplify.cse_main.cse`: it returns `(replacements, reduced)`, where
`replacements` is already in evaluation order. Printing it is a `for` loop.

---

## 3. Rust → WebAssembly

One command, no Cargo project, no external crates:

```console
$ rustc --target wasm32-unknown-unknown --crate-type cdylib --edition 2021 \
        -O -C panic=abort -C strip=symbols -C lto=fat kernel.rs -o kernel.wasm
```

Two design choices make the module small and the glue trivial:

**No `wasm-bindgen`, no Emscripten, no allocator.** The kernel has fixed sizes known at
generation time, so it needs no memory management at all. `rustc` exports linear memory and
the static buffer; JavaScript writes into it directly.

**`std`, but only its arithmetic.** `wasm32-unknown-unknown` ships `std` (minus I/O), which is
where `f64::sqrt`, `atan2` and `powi` come from. That is why this route needs no libm and no
wasi-sdk, and why the module is 74 KB rather than the 1–2 MB an Emscripten build would be.

### The ABI: one buffer, one offset table

```
MEM, an f64 array, exported as a global holding its address

   0   q      21   pose unknowns (p, quaternion per body)      in, and solved in place
  21   g      67   resolved geometry                          written by pack()
  88   t       1   drive value(s)
  89   out    21   residual Φ
 110   jac   441   ∂Φ/∂q, row-major
 551   met    10   camber, toe, track, contact, instant centre, roll centre
 551   world  30   world positions of every geometry point
 581   pts    30   raw input points
 611   axs     6   raw input axes
 617   num     1   lengths / gaps / compliances (NaN means "default")
```

```rust
#[no_mangle] pub static mut MEM: [f64; TOTAL] = [0.0; TOTAL];
#[no_mangle] pub extern "C" fn design();      // reset q to the design pose
#[no_mangle] pub extern "C" fn pack() -> i32; // raw geometry -> the g vector
#[no_mangle] pub extern "C" fn compute();     // Φ and J into `out` and `jac`
#[no_mangle] pub extern "C" fn solve(iters: i32) -> f64;  // damped Newton from the current q
#[no_mangle] pub extern "C" fn metrics();     // the drawn quantities into `met`
#[no_mangle] pub extern "C" fn world();       // point positions into `world`
```

**The one gotcha that costs an afternoon:** the offsets are indices into `MEM`, not into linear
memory. JavaScript must start from `instance.exports.MEM.value / 8`:

```js
base = instance.exports.MEM.value / 8;
q = new Float64Array(instance.exports.memory.buffer, base + 0, 21);
```

Everything else is copying numbers. `pack()` is what makes the module *topology-scoped rather
than geometry-scoped*: it completes each site's axes into an orthonormal frame, defaults a
missing tie-rod length to the design separation, and reads compliances — the same rules as
`Model.resolve()`, generated from the same symbol table.

---

## 4. What the emitter writes

```console
$ otkd emit examples/double-wishbone assets/data/wishbone --format wasm
```

```
assets/data/wishbone/
  kernel.js       105 KB  the only file a page imports: wasm inlined as base64 + the API
  kernel.json       2.5 KB  the ABI: offsets, point/axis orders, row names, units, metric names
  kernel.rs       180 KB  generated Rust (provenance; not deployed)
  kernel.wasm      74 KB  built by rustc (provenance; inlined into kernel.js anyway)
  provenance.json    1 KB  model id, otkd and rustc versions, sha256 of each input and of the wasm
  source/           12 KB  the four config files that produced it
```

The last two exist so the artefact can be audited without this repository: which model, which
generator, which toolchain, and the exact inputs. That is what makes a vendored copy reviewable
in a diff. The emitter refuses a model whose problem its single solver cannot solve (a redundant
or compliant one) rather than emitting a kernel that solves the wrong equations.

`kernel.js` is an ES module with no imports and no fetch — the module is inlined — so it drops
into any static build and works offline:

```js
import kernel from "./wishbone/kernel.js";

const { status, residual, q, points } = await kernel.frame(geometry, rideHeight);
// status 0 = ok; residual is the infinity norm of Φ; points are world positions
```

The lower-level calls (`design`, `setGeometry`, `setDrive`, `solve`, `evaluate`, `world`) exist
for the interactive case: set the geometry once, then solve on every slider tick. `solve`
returns the residual and leaves the pose in the buffer, so a page can show convergence.

---

## 5. Conformance

The kernel is an artefact, so it obeys the same rules as `out.js`:

- **Python is the reference.** `tests/test_wasm.py` solves the same drive values with the
  Python engine, runs the wasm module under node, and requires agreement to `1e-9` on poses and
  world points, and `1e-12` on the Jacobian. The Jacobian currently agrees exactly.
- **The source is reproducible.** `kernel.rs` and `kernel.json` are deterministic text; a test
  asserts that two emissions are byte-identical.
- **The binary is a toolchain artefact.** `.wasm` bytes depend on the rustc version, so they are
  tested behaviourally rather than by hash. Do not put the `.wasm` in a byte-diff check.
- **The emitter never re-derives.** If the Rust and Python disagree, the Python derivation is
  right and the artefact is stale; regenerate.

---

## 6. Getting it into `maths-site`

Two repositories, no registry: **this one publishes a zip, the site's CI downloads it and
unpacks it into its assets.** The zip is the transport for vendoring — not a runtime
dependency, and not a second build.

### This repository: one zip per tag

```console
$ uv run python scripts/build_release.py --version 0.11.0 --out dist
  double-wishbone              payload out.js              kernel kernel.wasm
  double-wishbone-compliant    payload out.js              kernel kernel.wasm
  double-wishbone-two-leg      payload out.js              kernel kernel.wasm
  double-wishbone-ubj          payload out.js              kernel kernel.wasm
  quarter-car                  payload none (dynamics)     kernel kernel.wasm
  ride-height                  payload out.js              kernel kernel.wasm

dist/otkd-0.11.0.zip  900 KB
sha256 40d6f315…
```

Five pose solvers and one integrator: the compliant and the redundant models compile now, and a
dynamics model has no payload because a trajectory is not a sweep.

`.github/workflows/release.yml` runs that on every push to `main` that warrants a release: the
version is decided from the Conventional Commits, applied as a tag only once the artefacts exist,
and served to the package by `hatch-vcs` ([`releasing.md`](releasing.md)). It rebuilds the
archive once to prove it is byte-reproducible, then attaches the zip and its `.sha256` to the
release. There is no `v` prefix. Reproducibility is
not an assertion: the workflow compares the two builds.

```
otkd-0.1.0/
  MANIFEST.json   generator, git commit, python and rustc versions, per-model status, and each
                  model's four config digests (`bundle: otkd-bundle-2`)
  SHA256SUMS      every file's digest
  package/        otkd-0.1.0-py3-none-any.whl and otkd-0.1.0.tar.gz
  models/double-wishbone/out.js                 the static payload
  models/double-wishbone/kernel/kernel.js       the compiled solver (self-contained)
  models/double-wishbone/kernel/kernel.json     the ABI
  models/double-wishbone/kernel/kernel.rs       generated source (provenance)
  models/double-wishbone/kernel/kernel.wasm     the module (provenance; inlined in kernel.js)
  models/double-wishbone/kernel/provenance.json input hashes, toolchain
  models/double-wishbone/source/*.yaml          the four config files, and their digests in the
                                                manifest — the only provenance a dynamics model has
  models/double-wishbone/kernel/source/*.yaml   the same four, beside the artefact they made
  models/ride-height/…
  models/double-wishbone-two-leg/…              every model the emitter supports, with a kernel
  models/quarter-car/kernel/                    an integrator, and no `out.js`: a trajectory is
                                                not a sweep, so there is no payload to publish
```

The manifest says *why* a kernel is missing rather than leaving a hole, so a consumer sees the
current limit instead of a broken path.

**A load's direction is the model's; its magnitude is a slot.** A dynamics kernel has a load buffer
— three numbers per load in the study — and `setLoads([...])` writes it. `setGeometry` fills it with
the model's own loads first, so a page that changes nothing gets the model and `setLoads` is the
override. The manifest lists the loads with their sites and defaults. Measured against Python with
the corner's load at 1× and 2×: the peak heave agrees to `1e-8` m both times.

**The manifest carries the model's own default geometry.** `default_geometry` is the points, the
axes and the numbers a page would otherwise have to reconstruct — including the points only a
dynamics model has, like a coil's seat. A page sets it once and then changes one number, which is
what a parameter control is: `setGeometry({ ...manifest.default_geometry, numbers })`.

**A dynamics study compiles an integrator.** `M` and `C` per body, the saddle-point acceleration,
RK4 with a projection after each step, and a trajectory buffer beside the pose: `integrate(steps)`
fills it and `trajectory()`/`velocities()` read it. The buffers are *additive* — `v`, `acc`, `traj`,
`trajv` — so the ABI string stays `otkd-kernel-2` and the four kinematic modules are byte-identical;
a page checks `solver: "dynamics"` before calling the functions only a dynamics kernel has. Measured
against Python on the quarter car: acceleration to `2.6e-8` on a scale of `21.8`, poses to `1.8e-8`
at step 100, energy drift `5.4e-8` ([`decisions.md`](decisions.md) round 28).

**Compliance is the KKT system, not a projection.** The equilibrium kernel solves
`[∇Π − Φᵀλ ; Φ] = 0` in `(q, λ)`, the same system the Python engine solves, so a joint with three
compliant components compiles and agrees with Python to `1e-9`. The first version projected the
potential onto a one-dimensional null space, which was right for the tie-rod corner and refused a
bushed ball joint by name ([`decisions.md`](decisions.md) round 25).

### `maths-site`: a step before the build

```yaml
      - name: Vendor the otkd kernel
        env:
          OTKD_VERSION: "0.1.0"
          OTKD_SHA256: "<the digest from the release>"
        run: |
          curl -sSL -o /tmp/otkd.zip \
            "https://github.com/<owner>/otkd/releases/download/v${OTKD_VERSION}/otkd-${OTKD_VERSION}.zip"
          echo "${OTKD_SHA256}  /tmp/otkd.zip" | sha256sum -c -
          unzip -q /tmp/otkd.zip -d /tmp/otkd
          install -d books/maths-applied/docs/assets/data/wishbone
          cp -r /tmp/otkd/models/double-wishbone/kernel/. \
                books/maths-applied/docs/assets/data/wishbone/
```

Both kinds of artefact come from the same zip, so a figure can import `out.js` and
`wishbone/kernel.js` from one pinned version. Cache the download on `OTKD_VERSION` (the digest
makes the cache safe): a build that needs no new version needs no network.

**The one thing to decide in that repository:** its `assets/` tree is committed and drift-checked,
so a build that writes into it dirties the tree. Either

- **ignore the vendored folder** (`books/maths-applied/docs/assets/data/wishbone/`) and let the
  build produce it, as above — the deployed site has it, the repository does not, and a local
  build runs the same vendor step first; or
- **commit it** and treat the download as a refresh a human runs when bumping `OTKD_VERSION`;
  then CI is fully offline and the drift check covers it.

The first is what "download it in the pipeline" means and is fine; the second is what keeps the
"a clone builds offline" rule exactly true. Whichever is chosen, the folder must be in exactly
one of the two states — ignored or committed — never both.

**Pin the digest, not just the version.** A GitHub release asset can be deleted and re-uploaded
under the same tag; the version number is a label, the `sha256` is the content.

## 7. Limits

- **The topology is fixed at build time.** Numbers are free — every geometry, every drive,
  every compliance. Changing a joint or a body is a re-emit, which is a seconds-long step.
- **No allocator, no threads, no I/O.** The module has no imports, which is why it works
  anywhere a `WebAssembly.Module` does.
- **74 KB.** Fine to inline; if that ever matters, `-C opt-level=z` and dropping `std` for
  `libm` are the levers.
- **Not a second implementation.** Nothing here is maintained by hand: `kernel.rs` is emitted,
  and if it disagrees with the derivation the test says so.

---

## References

- sympy, `cse`: <https://docs.sympy.org/latest/modules/simplify/simplify.html#sympy.simplify.cse_main.cse>
- sympy, code printers (`rust_code`, `ccode`): <https://docs.sympy.org/latest/modules/printing.html>
- sympy, `codegen` and `autowrap` (not used here): <https://docs.sympy.org/latest/modules/utilities/codegen.html>
- rustc, `wasm32-unknown-unknown`: <https://doc.rust-lang.org/rustc/platform-support/wasm32-unknown-unknown.html>
- The Rust and WebAssembly book: <https://rustwasm.github.io/docs/book/>
- MDN, WebAssembly JavaScript interface: <https://developer.mozilla.org/en-US/docs/WebAssembly/JavaScript_interface>
- MDN, `WebAssembly.Memory`: <https://developer.mozilla.org/en-US/docs/WebAssembly/JavaScript_interface/Memory>
- `wasm-bindgen` (not used — fixed-size buffers need no glue): <https://rustwasm.github.io/wasm-bindgen/>
- Emscripten (not used — bigger module, libc included): <https://emscripten.org/>
- Pyodide (not used — sympy in the browser is the wrong side of the boundary): <https://pyodide.org/>
