#!/bin/sh

set -ev

./tools/bazel test //:host
./tools/bazel test --cpu=stm32f4 -c opt //:target
