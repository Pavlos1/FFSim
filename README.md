# FFSim

This repository contains an X-Plane plugin that enables Hardware in the Loop
(HIL) testing of a flight controller.

> It has been tested on X-Plane's default plane (Cessna Skyhawk), but it should
work fine on any single-engine airplane with "standard" control surfaces.

**The plugin will override user control of the aircraft while enabled.** If
manual control of the aircraft is required, the plugin can be enabled and
disabled mid-flight via the menu under Plugins-\>Plugin Admin-\>Enable/Disable.

> The parking brake can be controlled by the user even when the plugin is
enabled. It's usually under the 'B' key.

Communication is done via USB-UART (FTDI), so you will need to make sure the
serial port has the right permissions set. Also you may need to hardcode the
serial port used if it is not `/dev/ttyUSB0` or `COM5`&mdash;ideally this would
be configurable via the X-Plane GUI instead.

> The plugin will print debug information if launched from the command line
with `./X-Plane-x86 |grep FFSim` or similar. It will print a message when
the serial connection is established or lost.

## Windows
Windows has a few prerequisites. The following worked for an RSCS Windows 10
Education machine, YMMV:

1. Install Cygwin. Make sure the standard development packages are installed
&mdash;in particular a 64-bit GNU linker is essential.
1. Install Rust via the [EXE](https://win.rustup.rs/x86_64). When prompted,
enter your host triple as `x86_64-pc-windows-gnu`.
1. Install LLVM via the latest 64-bit prebuild Windows binary available from
the LLVM [releases page](https://releases.llvm.org/download.html).

## Building
**You must use a 64-bit compiler**

```bash
cargo build
cp target/debug/libffsim.so $XPLANE_DIR/Resources/plugins/ffsim.xpl
```

The name of the DLL (`libffsim.so`) will differ slightly on Windows and
macOS. The destination file should still have the extension `.xpl`.

## Optimized Build

```bash
cargo build --release
cp target/release/libffsim.so $XPLANE_DIR/Resources/plugins/ffsim.xpl
```

