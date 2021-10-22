#!/bin/bash

# gtk-examples
sudo rm -R build
arch-meson -Dgtk-examples=true $_pkgdirname build
ninja -C build
sudo ./build/demo/gtk-libfprint-test