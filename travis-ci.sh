#!/bin/sh

set -ev

./tools/bazel test --cpu=k8 //:host
./tools/bazel build //:target
