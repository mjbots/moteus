#!/bin/sh

set -ev

./tools/bazel test --config=host //:host
./tools/bazel build --config=target //:target
