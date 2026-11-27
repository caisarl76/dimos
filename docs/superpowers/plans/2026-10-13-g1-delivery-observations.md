# G1 supervised delivery-freshness prototype

The user selected the delivery-freshness option in the hardware timing audit.
DimOS supervises the task, LeRobot supplies predictions and native SONIC owns
motor control. Acquisition age and skew remain unknown. This implementation
does not establish bottle-to-tray success or commission hardware motion.

1. Add a declared observation timing basis and immutable per-input receipt
   provenance. Preserve existing acquisition-mode callers and reject missing
   or inconsistent delivery metadata.
2. Add a separate bounded delivery pairer/module for independently received
   camera, body, left-hand and right-hand samples. Preserve original receipt
   times and source markers, detect source changes/rollback, and require a
   new module/service session after a fault or receiver restart.
3. Require observation, service and execution-profile timing agreement. Log
   the basis and receipts. Check delivery expiry throughout startup,
   inference waiting and execution, then use the existing bounded recovery.
4. Add a bounded, subscriber-only DDS/native-camera diagnostic that can
   exercise the adapter without a service, policy client or command writer.
   Keep receiver code separate from pairing logic. Real source names/hand
   ordering are explicit inputs; initial timing budgets remain provisional.
5. Test rejection, ownership, independent input dropout, source restart and
   service recovery with synthetic input and local transports. Run focused
   SONIC regression, registry generation and production lint/type checks.
6. Document the source/queue limitations, receiver deployment and remaining
   hardware validation. Inspect current ownership before any live probe.

The oldest original receiver time bounds age *since delivery*. It does not
bound earlier sensor, publisher or network/reader queue delays. Duplicate
source markers cannot refresh a cached sample. A publisher that restamps old
sensor values can evade this detector. Adapter faults stop publication;
already delivered observations expire within the configured service deadline.

Status on 2026-10-13: local implementation, regression tests, registry generation,
type/lint checks and independent source review are complete. The SONIC suite
passed 366 tests; the final registry/budget check passed six tests. PC2 preflight
found an active camera owner, so the native-camera/DDS pairing probe was not
run. Hardware commissioning remains pending. See
`docs/platforms/humanoid/g1/sonic-delivery-observations.md` and
`docs/artifacts/g1_delivery_observation_20261013/verification.json`.
