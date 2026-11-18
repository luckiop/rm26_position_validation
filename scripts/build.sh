#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$root/rm_tech_core_trajectory_solve"
if [[ ! -f "$source_dir/src/tech_core_planner/package.xml" ]]; then
  echo '请先运行 git submodule update --init --recursive' >&2
  exit 1
fi
cd "$root"
colcon build --symlink-install --build-base build-integrated --install-base install-integrated \
  --base-paths "$source_dir/src/tech_core_planner" "$source_dir/src/rm26_engineering_arm_demo" "$source_dir/thrid_party/mujoco" "$root/src/rm27_heavy_arm_demo" "$root/src/rm26_level3_validation" \
  --packages-up-to rm26_level3_validation "$@"
