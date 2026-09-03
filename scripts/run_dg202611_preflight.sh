#!/bin/bash
# DG202611 pre-hardware preflight checks
# Runs all tests and basic validation before experiments

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "===DG202611 PREFLIGHT CHECKS==="
echo "Repository: $REPO_ROOT"
echo ""

# Track results
PASS_COUNT=0
FAIL_COUNT=0

run_check() {
    local name="$1"
    shift
    echo "[$name] Running..."
    if "$@"; then
        echo "[$name] PASS"
        ((PASS_COUNT++))
        return 0
    else
        echo "[$name] FAIL"
        ((FAIL_COUNT++))
        return 1
    fi
}

# Step 1: ROS2 build
echo ""
echo "===STEP 1: ROS2 BUILD==="
cd "$REPO_ROOT"
if run_check "ROS2_BUILD" colcon build --packages-select ylhb_base --cmake-args -DBUILD_TESTING=OFF; then
    source install/setup.bash
else
    echo "Build failed, cannot proceed with tests"
    exit 1
fi

# Step 2: Python import checks
echo ""
echo "===STEP 2: PYTHON IMPORT CHECKS==="
cd "$REPO_ROOT/src/ylhb_base/scripts"

run_check "IMPORT_MODEL1" python3 -c "import multisource_fusion_core"
run_check "IMPORT_MODEL2" python3 -c "import lidar_robust_features"
run_check "IMPORT_MODEL3" python3 -c "import active_relocalization_core"
run_check "IMPORT_HEALTH" python3 -c "import navigation_health_core"
run_check "IMPORT_UWB_MODEL" python3 -c "import uwb_data_model"
run_check "IMPORT_UWB_ESTIMATOR" python3 -c "import uwb_2d_estimator"
run_check "IMPORT_UWB_QUALITY" python3 -c "import uwb_quality_gate"

# Step 3: Unit tests
echo ""
echo "===STEP 3: UNIT TESTS==="
cd "$REPO_ROOT"

run_check "MODEL1_TESTS" python3 -m pytest src/ylhb_base/test/test_multisource_fusion_core.py -v --tb=no
run_check "MODEL2_TESTS" python3 -m pytest src/ylhb_base/test/test_lidar_robust_features.py -v --tb=no
run_check "MODEL3_TESTS" python3 -m pytest src/ylhb_base/test/test_active_relocalization_core.py -v --tb=no
run_check "HEALTH_TESTS" python3 -m pytest src/ylhb_base/test/test_navigation_health_core.py -v --tb=no
run_check "UWB_TESTS" python3 -m pytest src/ylhb_base/test/test_uwb_model1_integration.py src/ylhb_base/test/test_uwb_model3_compatibility.py -v --tb=no

# Step 4: System scenarios
echo ""
echo "===STEP 4: SYSTEM SCENARIOS==="
run_check "SYSTEM_SCENARIOS" python3 scripts/sys_scenarios.py

# Step 5: Evaluator tests
echo ""
echo "===STEP 5: EVALUATOR TESTS==="
run_check "POSITION_EVALUATOR_TEST" python3 -m pytest src/ylhb_base/test/test_position_error_evaluator.py -v --tb=no

# Step 6: Evaluator import checks
echo ""
echo "===STEP 6: EVALUATOR IMPORTS==="
cd "$REPO_ROOT/scripts"
run_check "IMPORT_POS_EVAL" python3 -c "import position_error_evaluator"
run_check "IMPORT_FEAT_EVAL" python3 -c "import feature_repeatability_evaluator"
run_check "IMPORT_RELOC_EVAL" python3 -c "import relocalization_evaluator"
run_check "IMPORT_EXP_LOGGER" python3 -c "import experiment_logger"
run_check "IMPORT_SESSION_TOOL" python3 -c "import experiment_session_tool"

# Final summary
echo ""
echo "===PREFLIGHT SUMMARY==="
echo "PASSED: $PASS_COUNT"
echo "FAILED: $FAIL_COUNT"

if [ $FAIL_COUNT -eq 0 ]; then
    echo ""
    echo "✅ PREFLIGHT: PASS"
    exit 0
else
    echo ""
    echo "❌ PREFLIGHT: FAIL"
    exit 1
fi
