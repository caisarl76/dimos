# Stage A review and current gates

Independent read-only review: `/root/stage_a_review`, gpt-6-astra, high reasoning.
The reviewer inspected the implementation and raw evidence; it changed no files
and launched no controller.

Four blocking code/provenance findings are resolved and independently checked:

- A newer correlated negative settling result revokes prior pose permission.
- Finite unsupported/over-limit velocity commands emit IDLE with rejected-input
  status; newer valid source commands recover without ending the request.
- Publisher binding begins a fail-closed 250 ms quiet interval before rearm.
- The other session's observation changes are imported unchanged in their own
  labelled baseline commit `9fa98b269`, with updated source hashes and tests.

Verification: 399 SONIC tests and four shared Python/native lease-oracle tests
pass. Mypy passes for four production integration files; Ruff passes changed
Python files. All 20 original SONIC baseline and 51 original native source hashes
match their recorded snapshots. No package installation or original workspace
source/model changes were performed.

The fifth finding remains open: the translating noise proof cannot meet the
approved total native-replan limit. Run06 recorded 26 successful checks, but its
noise trace exercised IDLE and does not satisfy this finding. Run07 lost readiness
when the first PController.advance lazily imported SciPy; initialization is now
done before live leases. Run08 failed the unchanged forward stopping gate. Run09
stayed active and translated, with held facing and literal heartbeats, but
recorded 19 native state replans. Raw review attributes the 18 subsequent updates
to movement changes exceeding 0.02 rad during actual body-yaw variation.

Reviewer recommendation: keep the body-frame semantic, retain the ≤1 bound for a
deterministic constant-heading fixture, and separately attribute every live
direction/native-replan change. This alters the approved test interpretation;
the proposal is recorded in [stage-a-test-amendment.md](stage-a-test-amendment.md)
for user review. Run09 remains failed. This review does not authorize Stage B or
waive missing proof.

Task 5 and Stage A simulation verdict: **OPEN**. Stage B: **not started**.
Real robot commissioning: **blocked**, also by run02's unexplained reverse
displacement, run08's stopping failure and the missing exact FK fixture.
