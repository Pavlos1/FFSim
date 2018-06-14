# FFSim

TODO: What is this?

## Building
**You must use a 64-bit compiler**

```bash
cargo build
cp target/debug/libffsim.so $XPLANE_DIR/Resources/plugins/ffsim.xpl
```

The name of the DLL (`libffsim.so`) will differ slightly on Windows and
macOS. The destination file should still have the extension `.xpl`.

Make sure to delete `libffsim.so` after you have finished testing, as this
plugin will disable user inputs.

## Optimized Build

```bash
cargo build --release
cp target/release/libffsim.so $XPLANE_DIR/Resources/plugins/ffsim.xpl
```

