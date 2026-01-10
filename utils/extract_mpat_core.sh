#!/bin/bash

# Extract the core computation logic from mpat.html for use in CLI tools.
# This script reads the content between MPAT_CORE_START and MPAT_CORE_END markers
# and outputs a standalone JavaScript module.
#
# Usage:
#   ./extract_mpat_core.sh                      # Use default paths
#   ./extract_mpat_core.sh INPUT_HTML OUTPUT_JS # Use specified paths (for Bazel)

set -e

if [[ $# -eq 2 ]]; then
    # Bazel mode: paths provided as arguments
    MPAT_HTML="$1"
    OUTPUT_FILE="$2"
elif [[ $# -eq 0 ]]; then
    # Standalone mode: use default paths relative to script
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    MPAT_HTML="${SCRIPT_DIR}/../docs/mpat.html"
    OUTPUT_FILE="${SCRIPT_DIR}/mpat_core.mjs"
else
    echo "Usage: $0 [INPUT_HTML OUTPUT_JS]" >&2
    exit 1
fi

if [[ ! -f "$MPAT_HTML" ]]; then
    echo "Error: mpat.html not found at $MPAT_HTML" >&2
    exit 1
fi

# Extract content between markers
extract_core() {
    sed -n '/MPAT_CORE_START/,/MPAT_CORE_END/p' "$MPAT_HTML" | \
        sed '1d;$d'  # Remove the marker lines themselves
}

# Generate the module
cat > "$OUTPUT_FILE" << 'HEADER'
// Auto-generated from docs/mpat.html - DO NOT EDIT DIRECTLY
// Regenerate with: utils/extract_mpat_core.sh
//
// This module contains the core computation logic for the moteus Performance
// Analysis Tool. It can be used standalone for CLI tools or imported by other
// JavaScript/Node.js applications.

// Stub for html template tag (used only for UI rendering, not needed for computation)
const html = () => null;

HEADER

# Append the extracted core
extract_core >> "$OUTPUT_FILE"

# Add exports
cat >> "$OUTPUT_FILE" << 'EXPORTS'

// Module exports
export {
    // Field definitions (source of truth for motors, controllers, etc.)
    fieldDefinitions,

    // Helper classes
    LimitedValue,
    Interpolator2D,

    // Data structures
    OperatingPoint,

    // Base classes
    Controller,
    Motor,
    MoteusController,

    // Controller implementations
    MoteusC1,
    MoteusR4,
    MoteusN1,
    MoteusX1,

    // Odrive controllers
    OdriveController,
    OdriveMicro,
    OdriveS1,
    OdrivePro,

    // Motor implementations
    MJ5208,
    MAD8318,
    GBM5208,
    BE8108,
    Hoverboard350,
    HT1105,
    DCWS500Spindle,

    // Evaluation functions
    bisectMaxTrue,
    evaluateMaxCurrent,
    evaluateOperatingPoint,
    evaluateMaxTorque,
    evaluateMaxVelocity,
    evaluateMinMovementTime,

    // Analysis classes
    AnalysisMaxCurrent,
    AnalysisOperatingPoint,
    AnalysisMaxTorque,
    AnalysisMaxVelocity,
    AnalysisMinMovementTime,
};
EXPORTS

echo "Extracted mpat core to: $OUTPUT_FILE"
