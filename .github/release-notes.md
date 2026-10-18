Compiled artefacts for this version.

The zip carries the package as well: `package/otkd-<version>-py3-none-any.whl` and the sdist.
`models/<name>/out.js` is the static payload a page imports;
`models/<name>/kernel/` is the generated WebAssembly kernel, present where a compiled solver
exists today (the square Newton path). `MANIFEST.json` records which models have a kernel and
why any do not; `provenance.json` inside each kernel records the input hashes and the toolchain.

```console
$ V=<version>
$ curl -sSLO "https://github.com/$GITHUB_REPOSITORY/releases/download/$V/otkd-$V.zip"
$ curl -sSLO "https://github.com/$GITHUB_REPOSITORY/releases/download/$V/otkd-$V.zip.sha256"
$ sha256sum -c "otkd-$V.zip.sha256"
$ unzip -q "otkd-$V.zip" -d otkd
```

Pin the digest, not just the version: a release asset can be replaced, a `sha256` cannot.

The archive is byte-reproducible for this version, this compiler and these inputs — the release
workflow builds it twice and compares.
