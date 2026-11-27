# SONIC service coordinator

The user confirmed the next step after native execution-module validation:
package the existing observation/inference/supervision flow as a reusable
DimOS service coordinator. SONIC remains the motor controller. The action
provider remains LeRobot; this work does not implement a new grasp policy.

## Interface and scope

- One `SonicServiceModule`, injected `SonicExecutionSpec`, paired
  `In[SonicObservation]` and service-status output.
- Explicit policy-name/endpoint configuration. Operators must select policies
  compatible with the execution module's sonic78 profile; server metadata only
  negotiates image/history shape, not model/control semantics.
- Request an instruction, policy and bounded streaming duration. Reserve a
  service ID before asynchronous preflight so concurrent requests cannot
  overlap. Require compatible policy metadata and a fresh observation before
  admitting an execution task.
- Preserve original observation timestamps and reject replayed snapshots.
  One inference request runs outside the supervisor at a time. Replan every
  350 ms by default; execution retains its own freshness and reference guards.
- Duration begins with accepted running execution. Progress measures that
  duration, not manipulation success. Cancel acknowledges intent immediately;
  completion requires observing execution handback or reporting uncertainty.
- Every service records its request, native task binding, terminal result and
  externally reported outcome in a JSONL journal. Successful streaming leaves
  object success unknown. An explicit outcome requires evidence and the same
  service ID; successful object reports require finished, confirmed execution.

## Concurrency and failure handling

Module-loop state changes are serialized. Execution RPCs and policy inference
run off the loop. Configure injected RPC method timeouts explicitly; the
framework's default 120 seconds is unsuitable here. Check cancel/terminal
state after awaited work before issuing further commands. Never retry an
ambiguous start or infer controller recovery from a lost reply. Latch transport
uncertainty so another service cannot silently take ownership.

Shutdown requests cancellation, observes bounded handback, discards outstanding
inference and releases owned resources. The execution module's independent
supervision and native receiver expiry remain necessary if this coordinator
disappears. No automatic native process launch, robot mode change or hardware
connection is added.

## Implementation and verification

1. Add focused behavioral tests around the real execution module and existing
   scripted local ZMQ peer. Mock only GPU policy inference when needed.
2. Implement coordinator and typed dependency. Verify admission, timestamps,
   progress/finish, overlap, in-flight cancellation, frozen observations,
   inference/RPC errors, late results, outcome reporting and shutdown.
3. Exercise actual blueprint dependency wiring and worker RPC transport.
4. Regenerate the built-in registry, run focused SONIC tests, lint and typing,
   and obtain independent source review. Document configuration and limitations.

This change validates service orchestration with local protocol peers. Native
MuJoCo validation of this additional layer and real-robot commissioning remain
separate milestones. The prior execution-module native validation is retained.

## Completion

Completed on 2026-10-13. `SonicServiceModule`, its typed execution dependency,
generated module registry entry and usage guide are present. The focused suite
passes 262 tests (24 coordinator cases, including two worker integration tests),
and the five registry comparison checks pass. Focused Ruff and production mypy
checks pass. Independent review findings were fixed and re-reviewed.
Evidence: `docs/artifacts/g1_sonic_service_20261012/verification.json`.
Native validation of the coordinator is the next milestone.
