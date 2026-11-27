# G1 SONIC service implementation log

DimOS manages the service, while the existing external C++ SONIC controller owns body and hand motor commands. Integration validation precedes the stationary object-to-tray service. The current checkpoint establishes feedback and guarded planner handover, but arm tracking does not yet support precise manipulation execution.

## Checkpoint records

| Checkpoint | Scope | Result |
|---|---|---|
| [2026-10-07 integration checkpoint](/docs/platforms/humanoid/g1/implementation-log-2026-10-07.md) | Hardware observations from 2026-10-06 and local integration trials from 2026-10-06–07 | Feedback implemented; expiry and handover simulation-tested; incremental elbow tracking failed; latest integration not deployed on hardware. |

## Status at the 2026-10-07 checkpoint

| Area | Current status | Evidence category |
|---|---|---|
| Hardware inventory and D435i acquisition | Checked on `pc2_222`; camera-to-robot calibration and time synchronization remain. | Hardware-observed; calibration and timing still unverified. |
| SONIC feedback into DimOS | Implemented locally; real telemetry previously compared against independently subscribed DDS measurements. | Implemented and hardware-observed for passive feedback. |
| Reference expiry and planner handover | Local receiver prototype validated in isolated MuJoCo simulation. | Implemented as a local prototype and simulation-tested. |
| Small arm reference through SONIC | Tested in simulation; a requested +0.100 rad change produced +0.031 rad of measured movement. Tracking accuracy did not meet the criterion. | Simulation-tested with a failed tracking result; hardware tracking still unverified. |
| Dex3 grasping and object-to-tray service | Still to implement and validate. | Still unverified. |
| Latest integration deployed on hardware | No. The guarded receiver and moving-reference integration remain local. | Hardware deployment still unverified. |

The measured elbow change was +0.031413 rad, with an incremental error of −0.068587 rad against the +0.100 rad request. Successful reference delivery and planner handover do not establish a precise manipulation executor. The cause remains unresolved.

## Evidence categories

- **Implemented** means code exists in the local integration checkout or saved controller prototype. It does not imply upstream merge or hardware deployment.
- **Simulation-tested** means exercised with the isolated SONIC encoder, C++ controller and MuJoCo stack. Each result retains its own pass or fail outcome.
- **Hardware-observed** means measured in the earlier `pc2_222` inventory, camera acquisition or passive feedback checks. It does not establish command integration or manipulation readiness.
- **Still unverified** identifies work without the required validation, including code still to be implemented.

## Next milestone

Verify identical encoder inputs and compare outputs across Python and C++, then demonstrate a repeatably accurate unloaded reach through SONIC with measured completion and cancellation. Dex3 grasping follows that milestone. Camera/tool calibration, timestamp alignment and commissioning of the exact PC2 build remain prerequisites for the object-to-tray service.

The [latest arm-reference walkthrough](/docs/platforms/humanoid/g1/sonic-arm-reference-validation.md) is the technical source for this checkpoint. The [service development plan](/docs/platforms/humanoid/g1/service-development-plan.md) records the broader sequence. Implementation is held at this documentation checkpoint.
