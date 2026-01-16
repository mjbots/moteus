#!/bin/sh

set -ev

./tools/bazel test --test_size_filters= --test_tag_filters=-manual --config=host //:host
./tools/bazel build --config=target //:target
