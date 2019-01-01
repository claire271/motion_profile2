#!/bin/sh

# Create build directory
if [ ! -e build ]; then
	mkdir build
fi

# Build
cd src
javac -g `find . -name "*.java"` -d ../build
