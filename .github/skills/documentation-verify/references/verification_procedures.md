# Documentation Verification Procedures

This reference provides comprehensive procedures for verifying documentation accuracy against source code.

---

## Table of Contents

1. [Discovery Scan Procedures](#discovery-scan-procedures)
2. [Verification Pass Checklist](#verification-pass-checklist)
3. [Evidence Documentation](#evidence-documentation)
4. [Classification Rules](#classification-rules)
5. [False-Positive Prevention](#false-positive-prevention)

---

## Discovery Scan Procedures

### Step 1: Identify Changed Documentation Claims

Run `git diff` to list changed documentation files. For each changed file, categorize changes into:

**Claim Categories:**

- **Behaviour claims**: Assertions about how the project, commands, or features behave.
- **Options/defaults/constraints**: Documented flags, configuration keys, default values, allowed values, validation rules.
- **Examples**: Code samples, command invocations, YAML/JSON configurations, expected outputs.
- **CLI surface**: Command names, subcommands, flags, help text, output formats.
- **API surface**: REST endpoints, request/response formats, client method signatures, schemas.
- **Error messages**: Documented error text, exit codes, diagnostic output.
- **Terminology/renames/deprecations**: Changed names, deprecated features, migration paths.
- **Interface/component behaviour**: Connection types, interaction mechanics, isolation rules.

### Step 2: Form Initial Hypotheses

For each claim, assign a preliminary classification:

- **Supported**: Claim appears to match code structure (preliminary).
- **Unsupported**: Claim appears inconsistent with code (preliminary).
- **Speculative**: Claim describes future or intended behaviour without code backing.
- **Ambiguous**: Unclear whether claim matches code (needs deeper investigation).
- **Outdated**: Claim may describe previous code behaviour.

---

## Verification Pass Checklist

**CRITICAL**: Code is the source of truth. Complete verification before reporting any claims.

### Search Strategies

Use **at least two distinct search strategies** per claim:

1. **Direct identifier search**: Search for exact names, keys, constants, struct fields using `grep -r`, `git grep`, ripgrep, or language-specific tools.
2. **Entrypoint tracing**: Follow from CLI, config, or API entrypoint to implementation.
3. **Test evidence search**: Locate tests that exercise the claimed behaviour in test files.
4. **Schema/validation search**: Find parsers, validators, schema generators, struct tags, validation functions.

### Verification Checklist by Claim Type

#### Behaviour Claims

- [ ] Locate implementation code path
- [ ] Verify behaviour matches documented description
- [ ] Check for conditional behaviour (flags, modes, edge cases)
- [ ] Confirm error handling matches docs

#### Options/Defaults/Constraints

- [ ] Find struct field or config key definition
- [ ] Extract actual default value from code
- [ ] Find allowed values (enums, validation statements, regex patterns)
- [ ] Verify constraint enforcement

#### Examples

- [ ] Confirm example syntax matches actual parser expectations
- [ ] If example shows command output, verify against golden test files or actual execution
- [ ] Confirm field names, indentation, and structure match code expectations
- [ ] Check that referenced flags and options exist in code

#### CLI Surface

- [ ] Locate command definition in the CLI framework location
- [ ] Verify command name, aliases, subcommands match
- [ ] Check flag definitions (name, shorthand, type, default, help text)
- [ ] Confirm help text matches command definition
- [ ] Verify output formatting (column headers, sorting)

#### API Surface

- [ ] Find route definition
- [ ] Verify HTTP method, path, versioning
- [ ] Check request/response struct definitions
- [ ] Confirm client method signature
- [ ] Verify backward compatibility

#### Error Messages

- [ ] Search codebase for exact error text or pattern
- [ ] Verify error is returned in documented scenario
- [ ] Check error message format follows style guide

#### Terminology/Renames/Deprecations

- [ ] Search for old name to confirm it is truly deprecated or removed
- [ ] Find deprecation markers, aliases, or migration helpers
- [ ] Check changelog, release notes, or version gating logic
- [ ] Verify new name exists and is used consistently

---

## Evidence Documentation

### For Claims Supported by Code

Document:
- File path, function or struct, and line range
- Assessment: `Supported (verified at [file:line])`

### For Claims Not Supported by Code

Document:
- Searches performed (at least two strategies with specific search terms)
- What was expected versus what was found
- Assessment: `Unsupported (expected [X], found [Y] at [file:line])`

### For Inconclusive Claims

Document:
- Search attempts performed
- What evidence is missing or ambiguous
- Assessment: `Inconclusive (needs human review: [specific check])`

---

## Classification Rules

Based on verification evidence, reclassify hypotheses using this decision matrix:

| Original Hypothesis | Verification Outcome | Final Classification |
|---------------------|----------------------|----------------------|
| Unsupported | Found matching code | Retract claim (docs are correct) |
| Unsupported | Found code with different default | Docs outdated (needs value update) |
| Unsupported | No code evidence after thorough search | Confirmed unsupported (docs ahead of code) |
| Supported | Code contradicts doc claim | Docs incorrect (needs correction) |
| Ambiguous | Tests confirm behaviour | Supported (test-backed) |
| Ambiguous | Cannot locate relevant code | Inconclusive (flag for human review) |

---

## False-Positive Prevention

Apply these rules to avoid false positives:

1. **Do not claim "unsupported" without documented code search evidence**
   - Must have at least two strategies with explicit search terms

2. **Prefer conservative classifications**:
   - Prefer "inconclusive" over "unsupported" when code is complex or evidence is indirect
   - Prefer "outdated" over "unsupported" when code exists but with different behaviour or values
   - Prefer "imprecise" over "incorrect" when docs are vague but not technically wrong
   - Retract claim entirely if verification confirms docs are accurate

3. **Cross-check documentation coverage**:
   - Before claiming "unsupported", verify the entity is not documented elsewhere
   - Search `docs/` for related terms, alternative phrasings, synonyms
   - If claim is supported elsewhere, classify as "present but undiscoverable" instead

---

## Content Completeness Check

Before finalizing the report, check that all relevant topics are covered, especially for reference documentation:

- **CLI**: Verify command-line interface changes are reflected in CLI reference documentation
- **Configuration**: Check that new configuration options are documented in the reference section
- **APIs and schemas**: Validate that API and schema modifications are properly documented
