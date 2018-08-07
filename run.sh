#!/bin/bash
mkdir _build-test
cd _build-test
#cp ../build/berlin-latest.osm.pbf .
echo
echo "start osrm-extract"
./osrm-extract -p /usr/local/share/osrm/profiles/car.lua testdata.pbf
echo
echo "start osrm-partition"
./osrm-partition testdata.osrm
echo
echo "start osrm-customize"
./osrm-customize testdata.osrm

echo
echo "pb files"
ls *.pb
