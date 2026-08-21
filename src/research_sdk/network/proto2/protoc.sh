#! /usr/bin/env bash

if ! command -v protoc &> /dev/null ; then
    # brew install protobuf
    sudo apt install protobuf-compiler
fi


touch __init__.py

for proto in `ls *.proto`; do
    protoc --python_out=. $proto
done
