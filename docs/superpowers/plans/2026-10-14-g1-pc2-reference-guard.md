# PC2 reference-guard preparation

User authorization: prepare the simulation-tested SONIC receiver guard for
pc2_222 in an isolated build. The user owns real deployment/start commands.
Preserve the current checkout, local changes and running controller. Do not
edit implementation-log.md, launch real control or install system packages.

The contract is the existing stationary v4 receiver guard documented in
sonic-reference-expiry.md and its saved design/patch. Port existing behavior;
do not expand its navigation, grasp or timing guarantees.

1. Capture current PC2 source, local changes, build configuration and dependency
   paths. Pin source hashes and prove the active checkout stays unchanged.
2. Prepare isolated sources locally and on PC2. Merge the existing guard into
   the exact captured source, preserving telemetry and other local additions.
3. Verify existing guard tests against the port, compile the ARM controller and
   relevant native tests in separate output directories. Constrain build load.
   Add regression tests for any changed integration behavior, if necessary.
4. Review the complete port with a fresh reviewer. Compare final source/binary
   provenance, runtime dependencies and available command-line checks without
   connecting the new executable to real DDS/control.
5. Package an explicit operator launch procedure and report validation scope.
   A compiled/tested build does not establish hardware recovery. The operator
   launches it after the report; guarded live commissioning remains separate.

Evidence and progress ledger: docs/artifacts/g1_pc2_guard_20261014/.
Ruling: preserve evidence rather than automatic commits or scratch deletion;
the user has not requested commits, and the isolated snapshot must be auditable.
