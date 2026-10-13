# Stage A review and current gates

Independent read-only reviewer: `/root/stage_a_review`, gpt-6-astra, high
reasoning. The reviewer changed no files and launched no controller.

The five original findings are resolved: correlated negative settling evidence
revokes pose permission; finite unsupported/over-limit velocity becomes recoverable
IDLE; publisher startup enforces the 250 ms quiet interval; upstream observation
changes have their own baseline import; and the user-approved separate fixed-heading
and live-heading proofs now cover translation, not just IDLE.

The amendment review found two additional analyzer gaps: duplicate state-triggered
replans could match unchanged intent, and a packet sent after inference began could
explain a replan. Both regressions failed before the fixes. Eight artifact tests
now pass; the analyzer compares successive native states including periodic rows
and requires a pre-start send plus a real applied acknowledgment and valid deadline.

Reanalysis passes the saved evidence without another controller launch:

- Run10: constant-heading fixture, one state replan plus five periodic replans,
  identical admitted motion, 48 advancing source sequences, measured stop in 2.27 s.
- Run11: production live body-heading conversion, 19 attributed state replans plus
  five periodic replans, 0.814 m forward displacement, held facing/literal heartbeats,
  measured stop in 2.55 s.
- Run12: full 26-case matrix passes, including four movement signs/stops, source
  expiry/outage, feedback/subscription/publisher loss, pose lease suspension,
  pose expiry and handback. All 16 live-noise state replans are attributed; there
  are no unexplained conversions or heartbeat-only state replans.

The diagnostic runs remain incomplete full matrices. The default analyzer rejects
those; `--proof-only` explicitly validates the labelled noise proof. Run09 retains
its failed verdict under the previous aggregate criterion. All earlier raw
failures remain recorded and hashed.

The tested production baseline has 399 SONIC and four shared lease-oracle tests
passing. Production control semantics and its tested source bytes are unchanged.
The original native source still matches all 51 hashes. The task-boundary audit
found four newer upstream Python files (observation, session, service and service
tests); their separate import and compatibility tests must finish before Stage B.

Host-only Stage A proof: **PASS after the reviewed analyzer fixes**. Host-only
Stage B: **awaiting upstream compatibility update**. Real robot commissioning:
**blocked** by run02's unexplained reverse displacement, run08's stopping failure
and the missing exact FK fixture. No original workspace, environment or model was
modified and no real robot controller was launched.

See [task5-amendment-evidence.json](task5-amendment-evidence.json) for source, test
and per-run hashes; [stage-a-test-amendment.md](stage-a-test-amendment.md) records
the user's approved change in test interpretation.
