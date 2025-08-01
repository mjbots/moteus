#!/bin/sh

set -ev

./tools/bazel test --test_size_filters= --config=host //:host
./tools/bazel build --config=target //:target
