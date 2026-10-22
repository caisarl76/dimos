"""Two-G1 arm sync acceptance on two MuJoCo stand-ins (no robot).

Starts the frozen stand-in snapshot (standin/, see SHA256SUMS) twice on the dual blueprint's ports,
runs unitree-g1-sonic-arm-dual-agentic, drives the checks over MCP (and one sentence through the
agent when GENON_API_KEY is available), then stops everything and writes results.json. Never run it
beside another MuJoCo sim: a stand-in falls when it cannot keep real time.

Usage (from the worktree root):
    /home/jihun/work/dimos_dev/.venv/bin/python \
        docs/artifacts/g1_dual_arm_sim_20261021/demo_run_dual_sim.py <out-dir>
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PYTHON = "/home/jihun/work/dimos_dev/.venv/bin/python"
DIMOS = [PYTHON, "-m", "dimos.cli.dimos"]
ENCODER = (
    "/home/jihun/work/GR00T-WholeBodyControl/gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx"
)
SCENE = "/home/jihun/work/GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
MODEL_DIR = "/home/jihun/.cache/huggingface/hub/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2"
GENON = Path("/home/jihun/work/dimos_dev/.env.genon")
PORTS = {"222": (15556, 15557), "223": (16556, 16557)}
LEFT_ARM = slice(15, 19)
RESULT_TIMEOUT = 150.0


def env() -> dict[str, str]:
    e = dict(os.environ, PYTHONPATH=str(ROOT))
    if GENON.exists():
        for line in GENON.read_text().splitlines():
            if line.startswith("GENON_API_KEY="):
                e["OPENAI_API_KEY"] = line.split("=", 1)[1].strip().strip("\"'")
                e["OPENAI_BASE_URL"] = "https://api.genon.ai/v1"
    return e


def dimos(*args: str, check: bool = True, timeout: float = 60) -> str:
    done = subprocess.run(
        [*DIMOS, *args], cwd=ROOT, env=env(), capture_output=True, text=True, timeout=timeout
    )
    if check and done.returncode:
        raise RuntimeError(f"dimos {' '.join(args)} failed:\n{done.stdout}\n{done.stderr}")
    return done.stdout


def wait_for(predicate, timeout: float, what: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.5)
    raise TimeoutError(what)


def log_records() -> list[dict]:
    records = []
    for line in dimos("log", "--json", "-n", "5000", check=False).splitlines():
        try:
            records.append(json.loads(line))
        except ValueError:
            pass
    return records


def find(obj, key):
    """First value of `key` anywhere in a nested JSON value."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        obj = list(obj.values())
    if isinstance(obj, list):
        for v in obj:
            if (found := find(v, key)) is not None:
                return found
    if isinstance(obj, str) and obj[:1] in "{[":
        try:
            return find(json.loads(obj), key)
        except ValueError:
            pass
    return None


def results_of(tool: str) -> list[str]:
    return [
        str(r.get("result"))
        for r in log_records()
        if r.get("event") == "Arm skill result" and r.get("tool") == tool
    ]


def call(tool: str, **args) -> dict:
    """Call a background skill over MCP and wait for its result in the run log."""
    before = len(results_of(tool))
    started = time.monotonic()
    reply = dimos("mcp", "call", tool, "--json-args", json.dumps(args), check=False)
    out: dict = {"tool": tool, "args": args, "reply": reply.strip(), "t0": started}
    if "started" not in reply:
        out["result"] = None
        out["t1"] = time.monotonic()
        return out
    wait_for(lambda: len(results_of(tool)) > before, RESULT_TIMEOUT, f"{tool} result")
    out["t1"] = time.monotonic()
    out["result"] = results_of(tool)[before]
    for key in ("start_skew_s", "submit_skew_s"):
        out[key] = find(out["result"], key)
    return out


def standin(name: str, out: Path) -> subprocess.Popen:
    """STANDIN=repo runs the in-repo dimos.robot.unitree.g1.sonic_sim_peer; otherwise standin/."""
    reference, feedback = PORTS[name]
    repo = os.environ.get("STANDIN") == "repo"
    program = (
        ["-m", "dimos.robot.unitree.g1.sonic_sim_peer", "--model-dir", MODEL_DIR, "--scene", SCENE]
        if repo
        else ["sonic_sim_peer.py"]
    )
    return subprocess.Popen(
        [
            PYTHON,
            *program,
            "--name",
            f"g1_{name}",
            "--feedback",
            f"tcp://127.0.0.1:{feedback}",
            "--reference",
            f"tcp://127.0.0.1:{reference}",
            "--log",
            str(out / f"sim_{name}.npz"),
        ],
        cwd=ROOT if repo else HERE / "standin",
        env=env() if repo else None,
        stdout=open(out / f"standin_{name}.log", "w"),
        stderr=subprocess.STDOUT,
    )


def arm_motion(npz: Path, t0: float, t1: float) -> float:
    """Largest left-arm excursion (rad) from the window's first sample."""
    d = np.load(npz)
    sel = (d["t"] >= t0) & (d["t"] <= t1)
    if not sel.any():
        return float("nan")
    q = d["q"][sel][:, LEFT_ARM]
    return float(np.abs(q - q[0]).max())


def summary(npz: Path) -> dict:
    d = np.load(npz)
    return {
        "samples": len(d["t"]),
        "peak_left_arm_dq": float(np.abs(d["dq"][:, LEFT_ARM]).max()),
        "max_tilt_deg": float(d["tilt"].max()),
        "min_contacts": int(d["contacts"].min()),
        "tick_ms_p99": float(np.percentile(d["tick_ms"], 99)),
        "tick_ms_max": float(d["tick_ms"].max()),
    }


def main() -> None:
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (
        os.environ.get("STANDIN") != "repo"
        and "def startup_on_floor" not in (HERE / "standin/sonic_sim_peer.py").read_text()
    ):
        raise SystemExit("standin/sonic_sim_peer.py is not the full simulator")
    if "No running DimOS instance" not in dimos("status", check=False):
        raise SystemExit("another DimOS instance is running; not touching it")
    sims = {name: standin(name, out) for name in PORTS}
    checks: list[dict] = []
    try:
        for name in PORTS:
            log = out / f"standin_{name}.log"
            wait_for(lambda log=log: "standing" in log.read_text(), 180, f"{name} standing")
        agent = "OPENAI_API_KEY" in env()
        blueprint = "unitree-g1-sonic-arm-dual-agentic" if agent else "unitree-g1-sonic-arm-dual"
        model = ["--model", "openai:openai/gpt-5.6-luna"] if agent else []
        # Not through dimos(): the daemon inherits a captured stdout pipe and keeps it open.
        launch = subprocess.run(
            [
                *DIMOS,
                "--viewer",
                "none",
                "run",
                blueprint,
                *model,
                f"--dualsonicarmskillcontainer.encoder-path={ENCODER}",
                f"--dualsonicarmskillcontainer.collision-model-path={SCENE}",
                "--dualsonicarmskillcontainer.max-dq=2.0",
                "--daemon",
            ],
            cwd=ROOT,
            env=env(),
            stdin=subprocess.DEVNULL,
            stdout=open(out / "dimos_run.log", "w"),
            stderr=subprocess.STDOUT,
            timeout=180,
        )
        if launch.returncode:
            raise SystemExit(f"dimos run failed; see {out / 'dimos_run.log'}")
        wait_for(
            lambda: "move_upper_body" in dimos("mcp", "list-tools", check=False), 180, "mcp up"
        )
        time.sleep(2.0)

        checks.append(call("move_left_arm_pose", robot="both", pose="left_arm_side_horizontal"))
        checks.append(call("perform_gesture", robot="both", name="wave"))
        checks.append(call("move_upper_body", robot="223", motion="left_arm_forward", angle_deg=20))

        # Stop mid-wave: both must retreat smoothly. Stop 6 s after both stand-ins arm, not after the
        # call: encoding both waves takes longer than that, and a stop then sends nothing.
        armed = {n: (out / f"standin_{n}.log").read_text().count("armed epoch") for n in PORTS}
        before = len(results_of("perform_gesture"))
        t0 = time.monotonic()
        reply = dimos(
            "mcp",
            "call",
            "perform_gesture",
            "--json-args",
            json.dumps({"robot": "both", "name": "wave"}),
            check=False,
        )
        wait_for(
            lambda: all(
                (out / f"standin_{n}.log").read_text().count("armed epoch") > armed[n]
                for n in PORTS
            ),
            RESULT_TIMEOUT,
            "both armed",
        )
        time.sleep(6.0)
        stop_reply = dimos("mcp", "call", "stop_arm", check=False)
        wait_for(
            lambda: len(results_of("perform_gesture")) > before, RESULT_TIMEOUT, "stopped wave"
        )
        checks.append(
            {
                "tool": "perform_gesture+stop_arm",
                "reply": reply.strip(),
                "stop_reply": stop_reply.strip(),
                "t0": t0,
                "t1": time.monotonic(),
                "result": results_of("perform_gesture")[before],
            }
        )

        if agent:
            before = len(results_of("perform_gesture"))
            t0 = time.monotonic()
            dimos("agent-send", "both robots wave", check=False)
            try:
                wait_for(lambda: len(results_of("perform_gesture")) > before, 240, "agent wave")
                result = results_of("perform_gesture")[before]
            except TimeoutError:
                result = None
            checks.append(
                {
                    "tool": "agent-send: both robots wave",
                    "t0": t0,
                    "t1": time.monotonic(),
                    "result": result,
                    "start_skew_s": find(result, "start_skew_s"),
                }
            )

        # All-or-nothing: 223 gone, 222 must not move.
        sims["223"].send_signal(signal.SIGTERM)
        sims["223"].wait(30)
        time.sleep(1.0)
        checks.append(call("move_left_arm_pose", robot="both", pose="left_arm_side_horizontal"))
        time.sleep(3.0)
        checks[-1]["t1"] = time.monotonic()
    finally:
        dimos("stop", check=False, timeout=60)
        for p in sims.values():
            if p.poll() is None:
                p.send_signal(signal.SIGTERM)
                p.wait(30)

    for c in checks:
        c["arm_motion_rad"] = {
            name: arm_motion(out / f"sim_{name}.npz", c["t0"], c["t1"]) for name in PORTS
        }
    report = {
        "checks": checks,
        "standins": {name: summary(out / f"sim_{name}.npz") for name in PORTS},
    }
    (out / "results.json").write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps(report, indent=2, default=str))


if __name__ == "__main__":
    main()
