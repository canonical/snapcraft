---
name: documentation-style
description: "Enforces project documentation style guide compliance for tone, voice, terminology, punctuation, and formatting. Use when checking documentation style or validating MyST/reST syntax. Cites specific style guide violations."
---

# Documentation Style Review

## Scope

Style conformance only: tone, voice, terminology, punctuation,
Oxford comma, active versus passive voice, prohibited phrases,
and formatting conventions.

## Inputs

- Documentation file(s) under review.
- Normative style asset: `references/doc-style-guide.md`.
- Syntax-specific style guides (fetched at runtime):
  - MyST: `https://raw.githubusercontent.com/canonical/sphinx-docs-starter-pack/refs/heads/main/docs/reference/myst-syntax-reference.md`
  - reST: `https://raw.githubusercontent.com/canonical/sphinx-docs-starter-pack/refs/heads/main/docs/reference/rst-syntax-reference.rst`

## Actions

1. **Load style guides**: Read `references/doc-style-guide.md`.
   Fetch the syntax-specific guide matching the file type
   (MyST for `.md`, reST for `.rst`).

2. **Syntax compliance**: Check headings, lists, code blocks,
   inline literals, and directives against the applicable syntax guide.
   Treat every instruction in the guide as mandatory;
   do not rely on a subset of rules.

3. **Full style guide compliance**: Read and apply all rules defined
   in `references/doc-style-guide.md`.
   Treat every instruction in the guide as mandatory;
   do not rely on a subset of rules.

4. **Style guide citation**: For every violation found,
   locate and quote the specific passage
   in `references/doc-style-guide.md` or the syntax-specific guide
   that supports the finding.

5. **Fallback behaviour**: If syntax guides cannot be fetched
   (offline or network blocked), continue the review
   using `references/doc-style-guide.md`
   and the syntax patterns already present in the documentation set.
   
   If `references/doc-style-guide.md` is unavailable,
   STOP the review and report that the style guide cannot be accessed.

6. **Verification**:

   Confirm the style review completed:

   - Style guides loaded (doc-style-guide.md and syntax-specific guide if available)
   - Syntax compliance checked against applicable guide
   - Style guide compliance checked against all rules in doc-style-guide.md
   - All violations cited with quoted passages from style guides

   State the completion status:
   - `✓ Style review complete: [N] violations found`
   - OR `✓ Style review complete: No violations found`

## Constraints

- Quote style guides when making style suggestions.
- Do not suggest style or markup changes without quoting the style guides.

## Output

A list of style violations, each accompanied by:

- The specific passage quoted from the style guide or syntax reference.
- The observation or suggested change.
