---
name: documentation-structure
description: "Validates documentation structural integrity including heading hierarchy, metadata, file naming, navigation, and cross-references. Use when checking documentation organization or validating toctree structure."
---

# Documentation Structure Audit

## Scope

Document structure only: heading hierarchy, section ordering,
presence of required sections (introduction, prerequisites, steps, reference),
file naming, metadata blocks, navigation, and cross-references.

## Inputs

- Documentation file(s) under review.
- Documentation directory structure.
- Diataxis classification output
  (from the `documentation-diataxis` skill, when run as part of the orchestrated review).

## Actions

1. **File Naming**:

   Verify files use lowercase with dashes
   and the correct extension for their syntax
   (for example, `connect-vscode.rst` for reST, `connect-vscode.md` for MyST).

2. **Metadata**:

   Ensure every page has required metadata near the top
   when the repository's docs conventions require it:
   `.. meta::` after the anchor label for reST,
   or the MyST equivalent (front matter or `meta` directive) for Markdown sources.

3. **Directory Placement**:

   Confirm the file is located in the directory matching its intended
   Diataxis category
   (for example, tutorials in `tutorial/`, how-to guides in `how-to/`).

4. **Navigation**:

   Ensure new pages are added to the `toctree`.

5. **Cross-References**:

   - Prefer stable reference roles
     (`:ref:` for reST, `{ref}`/`{numref}` for MyST) over page-level links.
   - Flag uses of `:doc:`/`{doc}` (or equivalents)
     except for index-like pages that are unlikely to be moved or renamed.
   - Suggest adding links to improve documentation discoverability.
   - Verify cross-references resolve correctly.

6. **Verification**:

   Confirm the structural audit completed:

   - File naming checked for all files
   - Metadata presence verified
   - Directory placement validated (using Diataxis classification if available)
   - Navigation structure checked (toctree entries)
   - Cross-references validated

   State the completion status:
   - `✓ Structure audit complete: [N] violations found`
   - OR `✓ Structure audit complete: No violations found`

## Constraints

- Do not modify files during the audit; this is a read-only review.
- Do not invent metadata requirements; check against repository's documented conventions.
- Focus on structural issues only; do not flag style preferences.
- When Diataxis classification is available (from orchestrated review), use it to validate directory placement. Otherwise, infer from directory name.

## Output

A list of structural or metadata violations covering file naming,
metadata, directory placement, navigation, and cross-reference issues.
