#!/bin/bash
# Script to download all bazel dependencies for offline builds
# Run this script to populate the bazel cache before building

set -e

DISTDIR="${BAZEL_DISTDIR:-/tmp/bazel_cache}"
REPO_CACHE="${BAZEL_REPO_CACHE:-/tmp/repo_cache}"

mkdir -p "$DISTDIR"
mkdir -p "$REPO_CACHE"

download_and_cache() {
    local url="$1"
    local expected_hash="$2"
    local filename=$(basename "$url")

    if [ -f "$DISTDIR/$filename" ]; then
        echo "Already exists: $filename"
        return
    fi

    echo "Downloading: $url"
    curl -L -o "$DISTDIR/$filename" "$url"

    # Compute hash and add to repo cache
    local hash=$(sha256sum "$DISTDIR/$filename" | cut -d' ' -f1)
    mkdir -p "$REPO_CACHE/content_addressable/sha256/$hash"
    cp "$DISTDIR/$filename" "$REPO_CACHE/content_addressable/sha256/$hash/file"
    echo "  Cached with hash: $hash"

    if [ -n "$expected_hash" ] && [ "$hash" != "$expected_hash" ]; then
        echo "  WARNING: Hash mismatch! Expected: $expected_hash"
    fi
}

echo "Downloading moteus bazel dependencies..."
echo "DISTDIR: $DISTDIR"
echo "REPO_CACHE: $REPO_CACHE"
echo ""

# mjbots repositories
download_and_cache "https://github.com/mjbots/mjlib/archive/ec386ea442ad6dc4d597f4ca80d2317a6d0ec283.zip" \
    "b3ef858b7a0b7971fa1a2229a0481bfc8ddf0708a980c83f3885613e9e6ba7f6"

download_and_cache "https://github.com/mjbots/bazel-toolchain/archive/47efd434976067fec4689a3e5cca9859b82b08ec.zip" \
    "86428e992ad64e74bca6e388e4dc2846dee8fd8060b1b5cd8dfdc33631ef9103"

download_and_cache "https://github.com/mjbots/rules_mbed/archive/47c061f57e4ed7589ff6e851f2e77f045ce7811c.zip" \
    "ae6f71b4981031e5537507b690d48701ff17bc01b203bb8ab3127ca344d631f4"

download_and_cache "https://github.com/mjbots/bazel_deps/archive/18eeb7e59ab3535ef93c45716313b9a037c972a1.zip" \
    "7b5891127d884743d23779f42a2d80dfcea93fe97efb16372495497e19ff8468"

# Bazel ecosystem
download_and_cache "https://github.com/bazelbuild/rules_cc/archive/726dd8157557f1456b3656e26ab21a1646653405.tar.gz" \
    "b6f34b3261ec02f85dbc5a8bdc9414ce548e1f5f67e000d7069571799cb88b25"

download_and_cache "https://github.com/bazelbuild/bazel-skylib/archive/d2cf1cc2bcd1e879743faf5216c4887b994705af.zip" \
    "4f5657797eb215b4281a05eee830b22289d2ef7571ad8ace6c8da4db76f47b7e"

download_and_cache "https://github.com/bazelbuild/rules_python/releases/download/0.24.0/rules_python-0.24.0.tar.gz" \
    "0a8003b044294d7840ac7d9d73eef05d6ceb682d7516781a4ec62eeb34702578"

# LLVM toolchain (large file ~414MB)
download_and_cache "https://github.com/llvm/llvm-project/releases/download/llvmorg-10.0.0/clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz" \
    "b25f592a0c00686f03e3b7db68ca6dc87418f681f4ead4df4745a01d9be63843"

# Boost (large file ~124MB)
download_and_cache "https://archives.boost.io/release/1.74.0/source/boost_1_74_0.tar.gz" \
    "afff36d392885120bcac079148c177d1f6f7730ec3d47233aa51b0afa4db94a5"

# mbed-os (ARM microcontroller SDK, ~88MB)
download_and_cache "https://github.com/ARMmbed/mbed-os/archive/mbed-os-5.13.4.tar.gz" \
    "5c291c1df779834224437043e2274bd1b54216b01e145c1754ac6cd8890cb12e"

# Third-party libraries (from mjlib dependencies)
download_and_cache "https://github.com/fmtlib/fmt/archive/5.0.0.tar.gz" \
    "fc33d64d5aa2739ad2ca1b128628a7fc1b7dca1ad077314f09affc57d59cf88a"

download_and_cache "https://github.com/Naios/function2/archive/7cd95374b0f1c941892bfc40f0ebb6564d33fdb9.zip" \
    "f2127da1c83d1c3dea8a416355a7cbb4e81fe55cf27ac9ef712d0554a3b745b6"

download_and_cache "https://github.com/muellan/clipp/archive/2c32b2f1f7cc530b1ec1f62c92f698643bb368db.zip"

download_and_cache "https://github.com/google/snappy/archive/1.1.7.tar.gz"

# System dependencies (Ubuntu 24.04 needs libtinfo5 from older package)
if ! ldconfig -p | grep -q libtinfo.so.5; then
    echo "Installing libtinfo5..."
    curl -L -o /tmp/libtinfo5.deb http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb
    dpkg -i /tmp/libtinfo5.deb
    ldconfig
fi

# Python dependencies
echo "Installing Python dependencies..."
apt-get install -y python3-can python3-serial python3-setuptools python3-pyelftools \
    python3-qtpy python3-wheel python3-importlib-metadata python3-scipy python3-usb mypy nodejs

echo ""
echo "All dependencies downloaded!"
echo ""
echo "To build with these dependencies, use:"
echo "  tools/bazel test --config=host //:host --repository_cache=$REPO_CACHE --distdir=$DISTDIR --spawn_strategy=local"
