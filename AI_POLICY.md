# AI Use Policy

This policy applies to all contributions to this repository (code, documentation,
issues, and pull request discussions). It exists to keep contributions high-quality
and maintainer time well spent — not to police your tooling.

## Position

AI-assisted development is a reality, and this project does not fight it. The
maintainer uses AI tooling in developing this project. AI is welcome here as a
tool — the same way a compiler, an IDE, or a code formatter is a tool. What is
not welcome is AI as a *substitute for understanding*.

The distinction this policy draws is simple: **AI may help you produce the work,
but it may not replace you as the author of it.**

## What is expected of AI-assisted contributions

If you used AI assistance in any part of a contribution, all of the following
must hold:

1. **You have the foundational knowledge to direct the tool.** You knew what you
   were asking for before the AI produced it.
2. **You understand the output.** Every line in your diff is a line you can
   explain — what it does, why it is there, and what would break without it.
3. **You know how it was tested.** You can describe how the change was verified
   and what its failure modes are.
4. **The contribution is not bloat.** AI makes it cheap to generate large diffs.
   A contribution is judged by the problem it solves, not its size. Changes that
   exist primarily to pad activity metrics will be closed.
5. **AI was used to increase productivity, not to skip the work.** Generating
   boilerplate you already know how to write and would rather oversee than type
   is a good use. Generating code you could not have written or reviewed
   yourself is not.

## A human is the interface

A human must be the main interface for every contribution, even when AI
assistance is involved:

- **Autonomous agents may not contribute.** Pull requests or issues that were
  authored and submitted by an AI agent acting on its own will be closed without
  detailed review.
- **You answer review questions yourself, in your own words.** If a maintainer
  asks about your change, the answer comes from you. Pasting the question into
  an AI and pasting its answer back is grounds for closing the PR. The review
  conversation is how this project verifies a human is behind the work — treat
  it as such.
- **Accountability is human.** Commits are attributed to you. Do not add AI
  co-author trailers (e.g. `Co-Authored-By: <AI tool>`). Your sign-off means
  *you* take responsibility for the change, including its correctness and your
  right to submit it under this project's license.

Using AI to improve the grammar or clarity of text you wrote yourself is fine
and, for non-native English speakers, encouraged.

## Disclosure

State in the pull request description whether the change is:

- **No AI** — written entirely by hand.
- **AI-assisted** — you directed and reviewed AI-generated portions.
- **AI-generated** — substantial portions produced by AI under your review.

Disclosure carries no penalty. It exists so review effort can be calibrated
honestly. Non-disclosure that later becomes evident does carry a penalty: loss
of the benefit of the doubt on future contributions.

To keep review load manageable, please keep **at most one AI-assisted or
AI-generated pull request open at a time**.

## Verification

Code that has only been reasoned about is not verified. Every PR must describe
how the change was verified, at a level a reviewer can reproduce.

**If the change affects behavior on hardware** (e.g. board support code, drivers,
peripheral configuration), you must explain the test you ran:

- What hardware (chip, board revision, relevant peripherals or feature flags).
- What you actually did — the procedure, not just "tested on RP2040". For
  example: "flashed the blinky example on a Megalops rev B, confirmed LED
  toggling and USB enumeration after reset."
- What you observed, including anything unexpected.

If you could not test on hardware, say so plainly and describe what you did
verify instead (see below). An untested contribution is not automatically
rejected — the maintainer may be able to verify it on hardware — but an
*undisclosed* lack of testing is treated as a policy violation. AI cannot flash
a board on your desk; this requirement is how the project keeps hardware truth
in the loop.

**If the change does not involve hardware** (e.g. code generation logic, host
tooling, CI, documentation), an equivalent standard applies. State which of the
following you did:

- Ran the test suite, and added or updated tests covering the change.
- For code-generation changes: included or updated snapshot tests so the diff
  of the *generated output* is visible in review, and inspected that output
  yourself.
- Confirmed the crate builds for all supported targets / feature combinations
  the change touches.
- For documentation-only changes: rendered and read the result.

"It compiles" alone is not verification. If AI produced the code, you are
attesting that the verification above was performed by you, not assumed.

## Enforcement

Maintainers may close, without extended justification, contributions that show
signs of unreviewed AI output: APIs that do not exist, descriptions that do not
match the diff, boilerplate responses to review comments, or an inability to
discuss the change. If you believe your contribution was closed in error, reply
as yourself — a genuine human conversation resolves this quickly.
