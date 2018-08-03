#!/bin/bash
mkdir _build-test
cd _build-test
cp ../build/berlin-latest.osm.pbf .
echo
echo "start osrm-extract"
./osrm-extract -p /usr/local/share/osrm/profiles/car.lua berlin-latest.osm.pbf
echo
echo "start osrm-partition"
./osrm-partition berlin-latest.osrm
echo
echo "start osrm-customize"
./osrm-customize berlin-latest.osrm

echo
echo "pb files"
ll *.pb