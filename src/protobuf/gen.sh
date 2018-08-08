#!/bin/bash

protoc --cpp_out=. node-based-graph.proto
protoc --cpp_out=. edge-based-graph.proto
protoc --cpp_out=. rtree.proto
