#!/usr/bin/env bash
# Provision the SONIC navigation package on pc2 (.223). Run from <task>/package after
# copying the assembled payload there. It writes only under the task directory and
# creates one new image. It starts no controller, container service, DDS writer or
# robot command, and installs nothing on the host.
set -euo pipefail

verify_root=/home/unitree/dimos-navigation-verify-20261009
task="${SONIC_NAV_TASK_ROOT:-$verify_root/sonic-navigation-20261013}"
package="$task/package"
original=/home/unitree/GR00T-WholeBodyControl
other_stage=/home/unitree/dimos-sonic-guard-pc2-223-20261014
image=dimos-g1-sonic-navigation:20261013
base_image=dimos-g1-navigation-verify:20261012-pointlio
base_image_id=sha256:01980cefa285106c334c71d9dafb8db6b8ff24ded4d6539f6d91baaab56178b0
log="$task/provision"

cd "$package"
sha256sum --check --quiet manifest.sha256  # every payload file, from assemble.py
[[ -e "$task/native" || -e "$log" ]] && { echo "Task already provisioned: $task" >&2; exit 1; }
mkdir -p "$log"

snapshot() {  # $1: before|after. Fingerprints and shared controllers must not change.
    sha256sum --check --quiet "$verify_root/environment-before.sha256" > "$log/fingerprints-$1.log" 2>&1
    (cd "$original" && sha256sum --check --quiet "$package/native-baseline.sha256") > "$log/original-source-$1.log" 2>&1
    sha256sum --check --quiet "$package/shared-controllers.sha256" > "$log/shared-controllers-$1.log" 2>&1
    ps -eo pid,user,comm,args --sort=pid > "$log/processes-$1.txt"
}
snapshot before

# Native: the ported source tree, built with pc2's own JetPack toolchain and SDK libraries.
mkdir "$task/native"
tar -xf native-source.tar -C "$task/native"
(cd "$task/native" && sha256sum --check --quiet "$package/native-source.sha256")
ln -s "$original/gear_sonic_deploy/thirdparty" "$task/native/gear_sonic_deploy/thirdparty"
source_dir="$task/native/gear_sonic_deploy"
cmake -S "$source_dir" -B "$task/native-build" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_DEPLOY_TESTS=ON -DBUILD_EXAMPLES=OFF -DBUILD_SRCS=ON > "$log/native-configure.log" 2>&1
nice -n 19 cmake --build "$task/native-build" -j 2 > "$log/native-build.log" 2>&1
sdk_lib="$source_dir/thirdparty/unitree_sdk2/thirdparty/lib/aarch64"
native_env=(LD_LIBRARY_PATH="$sdk_lib:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/opt/onnxruntime/lib"
    SONIC_PLANNER_LEASE_CASES="$package/sonic_planner_lease_cases.json")
(cd "$source_dir" && env "${native_env[@]}" ./target/release/run_tests \
    --gtest_output="json:$log/native-tests.json" > "$log/native-tests.log" 2>&1) || true
python3 - "$log/native-tests.json" <<'EOF'
import json, sys
report = json.load(open(sys.argv[1]))
failed = sorted(f"{s['name']}.{t['name']}" for s in report["testsuites"] for t in s["testsuite"] if t.get("failures"))
# The exact FK fixture is absent on every host; any other failure blocks the package.
assert failed == ["FK.TestFKAndGlobalVelocities"], failed
print(f"native tests: {report['tests'] - len(failed)}/{report['tests']} passed; only the FK fixture test fails")
EOF
sha256sum "$source_dir/target/release/g1_deploy_onnx_ref" | tee "$log/controller.sha256"

# Models and TensorRT caches: copied from pc2's own installation, never from another device.
while read -r digest relative; do
    install -D -m 0644 "$original/gear_sonic_deploy/$relative" "$source_dir/$relative"
done < models.sha256
(cd "$source_dir" && sha256sum --check --quiet "$package/models.sha256")

# Python graph image, on the verified existing base image; build-only host networking.
[[ "$(docker image inspect --format '{{.Id}}' "$base_image")" == "$base_image_id" ]] \
    || { echo "Base image $base_image is not $base_image_id" >&2; exit 1; }
if docker image inspect "$image" > /dev/null 2>&1; then
    echo "Refusing to replace existing image: $image" >&2; exit 1
fi
docker build --network host --build-arg "BASE_IMAGE=$base_image" --tag "$image" \
    --file Dockerfile . > "$log/image-build.log" 2>&1
docker run --rm --network none --cpus 2 --memory 4g --user 1000:1000 --cap-drop ALL \
    --security-opt no-new-privileges "$image" > "$log/smoke.json"
docker image inspect "$image" > "$log/image-inspect.json"

snapshot after
diff -u "$log/processes-before.txt" "$log/processes-after.txt" > "$log/processes.diff" || true
echo "Provisioned $task: fingerprints, original source and shared controllers unchanged."
echo "No controller, graph container or robot command was started."
