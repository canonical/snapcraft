# Documentation Verification Report Format

This reference defines how to structure and format the final verification report.

---

## Report Structure

### 1. Group Findings by Classification

Organize findings into these categories:

- **Confirmed unsupported**: Docs describe behaviour or options not present in code
- **Docs outdated**: Code exists but with different values or behaviour than documented
- **Docs incorrect**: Code contradicts doc claim
- **Docs imprecise**: Code behaviour more nuanced than docs suggest
- **Docs speculative**: Describes intended future behaviour (not yet implemented)
- **Inconclusive**: Cannot verify (requires human review)
- **No issues found**: All doc claims backed by code (state this explicitly)

---

## Finding Format Template

For each verified finding, use this structure:

````markdown
**Doc Claim**: [File path:line] "[Quoted claim from docs]"

**Verification Checklist**:
- [ ] Search strategies used: [list at least two strategies]
- [ ] Code location(s) checked: [file paths]
- [ ] Test evidence: [test file/function or "Not found"]
- [ ] Schema/validation: [struct/parser location or "Not found"]

**Code Evidence**:
- **Expected**: [What docs claim should exist]
- **Found**: [What code actually shows, with file:line references]
- **Assessment**: [Supported | Unsupported | Outdated | Incorrect | Imprecise | Inconclusive]

**Issue**: [Classification from list above]
- [Brief description of mismatch]

**Recommended Action**:
- [File path]: [Specific minimal edit to restore correctness]
- Rationale: [Why this edit aligns docs with code]
- Alternative: [If docs are ahead of code, suggest opening an issue or reverting speculative claim]
````

---

## Conservative Change Suggestions

When recommending actions, follow these guidelines:

### For "Docs Outdated"
Update specific values or behaviour descriptions to match current code.

**Example**:
```
Recommended Action:
- docs/config.md:42: Change default value from `timeout: 30s` to `timeout: 60s`
- Rationale: Code shows default in config/defaults.go:15 is time.Duration(60 * time.Second)
```

### For "Docs Incorrect"
Correct the claim with precise wording from code.

**Example**:
```
Recommended Action:
- docs/api.md:78: Change "returns HTTP 200 on success" to "returns HTTP 201 on success"
- Rationale: Handler at handlers/create.go:45 returns http.StatusCreated (201)
```

### For "Docs Imprecise"
Add qualifiers, conditions, or edge-case notes.

**Example**:
```
Recommended Action:
- docs/cli.md:120: Add note "Only available when --experimental flag is enabled"
- Rationale: Feature gated behind ExperimentalMode check at cmd/feature.go:33
```

### For "Confirmed Unsupported"
Revert or remove unsupported claim. If claim represents intended behaviour, change to future tense and add note.

**Example**:
```
Recommended Action:
- docs/features.md:89: Remove claim about auto-retry functionality
- Alternative: If feature is planned, change to "Auto-retry functionality is planned for future release"
```

### For "Docs Speculative"
Mark as future or intended, not current behaviour.

**Example**:
```
Recommended Action:
- docs/roadmap.md:15: Change "The system supports" to "The system will support (planned)"
- Rationale: No implementation found; appears to be planned feature
```

### For "Inconclusive"
Provide specific human review action.

**Example**:
```
Recommended Action:
- Request human review: Cannot locate implementation for documented retry logic
- Check: Is this implemented via external library? Search plugin system
```

---

## Linking to Code Artifacts

Always reference:
- Specific files with line numbers (e.g., `auth/handler.go:145-167`)
- Function or struct names (e.g., `parseConfig()`, `ServerConfig struct`)
- Test files that exercise the behaviour (e.g., `auth_test.go:TestLoginSuccess`)
- Search commands used (e.g., `grep -r "timeout" config/`)

---

## Report Integration

- Prioritize code backing findings appropriately (blocking issues for incorrect or unsupported claims)
- Cross-reference with Diataxis compliance findings to identify if inaccuracies stem from category misalignment
- Prepare evidence-based recommendations with code references

---

## Final Output Rule

**Only verified findings with evidence make it to the final report.**

Do NOT include:
- Intermediate hypotheses
- Reasoning process
- Unverified claims
- Speculation without code backing
