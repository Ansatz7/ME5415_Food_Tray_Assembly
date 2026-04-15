#!/bin/bash
# ME5415 Food Tray Assembly - SOFA Environment Setup
# Source this file before running any simulation:
#   source setup_env.sh

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# v23.06 = Python 3.8, compatible with Ubuntu 20.04 GLIBC 2.31
# v24.12 = Python 3.12, requires GLIBC 2.32+ (Ubuntu 22.04+)
SOFA_DIR="$REPO_DIR/sofa/SOFA_v23.06.00_Linux"

# --- SOFA binary ---
export PATH="$SOFA_DIR/bin:$PATH"

# --- Shared libraries ---
export LD_LIBRARY_PATH="\
$SOFA_DIR/lib:\
$SOFA_DIR/plugins/SofaPython3/lib:\
$SOFA_DIR/plugins/SoftRobots/lib:\
$SOFA_DIR/plugins/STLIB/lib:\
$SOFA_DIR/plugins/SoftRobots.Inverse/lib:\
$SOFA_DIR/plugins/ModelOrderReduction/lib:\
${LD_LIBRARY_PATH}"

# --- Python bindings (all plugin site-packages, Python 3.8) ---
export PYTHONPATH="\
$SOFA_DIR/plugins/SofaPython3/lib/python3/site-packages:\
$SOFA_DIR/plugins/SoftRobots/lib/python3/site-packages:\
$SOFA_DIR/plugins/STLIB/lib/python3/site-packages:\
$SOFA_DIR/plugins/SoftRobots.Inverse/lib/python3/site-packages:\
$SOFA_DIR/plugins/ModelOrderReduction/lib/python3/site-packages:\
$SOFA_DIR/plugins/CosseratPlugin/lib/python3/site-packages:\
$SOFA_DIR/plugins/SofaGLFW/lib/python3/site-packages:\
${PYTHONPATH}"

# --- SOFA plugin load path (for runSofa) ---
# Fix Qt WebEngine sandbox crash on Ubuntu 20.04
export QTWEBENGINE_DISABLE_SANDBOX=1

export SOFA_ROOT="$SOFA_DIR"
export SOFA_PLUGIN_PATH="\
$SOFA_DIR/plugins/SofaPython3:\
$SOFA_DIR/plugins/SoftRobots:\
$SOFA_DIR/plugins/STLIB:\
$SOFA_DIR/plugins/SoftRobots.Inverse:\
$SOFA_DIR/plugins/ModelOrderReduction"

echo "SOFA environment configured."
echo "  SOFA_ROOT:  $SOFA_ROOT"
echo "  runSofa:    $(which runSofa 2>/dev/null || echo 'not in PATH yet')"
echo ""
echo "Usage:"
echo "  runSofa simulation/Main.py           # GUI mode"
echo "  python3 simulation/Main.py           # headless / scripted"
