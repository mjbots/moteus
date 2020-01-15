#!/bin/bash

GIT_REV=$(git rev-parse HEAD)
if [[ $? != 0 ]];
then
    exit 1
fi

echo "BUILD_SCM_REVISION ${GIT_REV}"

# Check whether there are any uncommitted changes.
git diff-index --quiet HEAD --
if [[ $? == 0 ]];
then
    TREE_STATUS=0
else
    TREE_STATUS=1
fi

echo "BUILD_SCM_STATUS ${TREE_STATUS}"
