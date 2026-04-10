---
description: 'Guidelines for GitHub Copilot to write comments to achieve self-explanatory code with fewer comments. Examples are in Python but it should work on any language that has comments.'
applyTo: '**'
---

# Self-explanatory Code Commenting Instructions

## Core Principle
**Write code that speaks for itself. Comment only when necessary to explain WHY, not WHAT.**
We do not need comments most of the time.

## Commenting Guidelines

### ❌ AVOID These Comment Types

**Obvious Comments**
```python
# Bad: States the obvious
counter = 0  # Initialize counter to zero
counter += 1  # Increment counter by one
```

**Redundant Comments**
```python
# Bad: Comment repeats the code
def get_user_name():
    return user.name  # Return the user's name
```

**Outdated Comments**
```python
# Bad: Comment doesn't match the code
# Calculate tax at 5% rate
tax = price * 0.08  # Actually 8%
```

### ✅ WRITE These Comment Types

**Complex Business Logic**
```python
# Good: Explains WHY this specific calculation
# Apply progressive tax brackets: 10% up to 10k, 20% above
tax = calculate_progressive_tax(income, [0.10, 0.20], [10_000])
```

**Non-obvious Algorithms**
```python
# Good: Explains the algorithm choice
# Using Floyd–Warshall for all-pairs shortest paths
# because we need distances between all nodes
for k in range(vertices):
    for i in range(vertices):
        for j in range(vertices):
            # ... implementation
            pass
```

**Regex Patterns**
```python
# Good: Explains what the regex matches
# Match email format: username@domain.extension
import re
email_pattern = re.compile(r'^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$')
```

**API Constraints or Gotchas**
```python
# Good: Explains external constraint
# GitHub API rate limit: 5000 requests/hour for authenticated users
await rate_limiter.wait()
response = await http_client.get(github_api_url)
```

## Decision Framework

Before writing a comment, ask:
1. **Is the code self-explanatory?** → No comment needed  
2. **Would a better variable/function name eliminate the need?** → Refactor instead  
3. **Does this explain WHY, not WHAT?** → Good comment  
4. **Will this help future maintainers?** → Good comment

## Special Cases for Comments

### Public APIs
```python
def calculate_compound_interest(principal: float, rate: float, time: float, compound_frequency: int = 1) -> float:
    """
    Calculate compound interest using the standard formula.

    Args:
        principal (float): Initial amount invested.
        rate (float): Annual interest rate (as decimal, e.g., 0.05 for 5%).
        time (float): Time period in years.
        compound_frequency (int, optional): How many times per year interest compounds (default: 1).

    Returns:
        float: Final amount after compound interest.
    """
    # ... implementation
    pass
```

### Configuration and Constants
```python
# Good: Explains the source or reasoning
MAX_RETRIES = 3  # Based on network reliability studies
API_TIMEOUT = 5.0  # AWS Lambda timeout is 15s, leaving buffer (seconds)
```

### Annotations
```python
# TODO: Replace with proper user authentication after security review
# FIXME: Memory leak in production - investigate connection pooling
# HACK: Workaround for bug in library v2.1.0 - remove after upgrade
# NOTE: This implementation assumes UTC timezone for all calculations
# WARNING: This function mutates the input list instead of returning a copy
# PERF: Consider caching this result if called frequently in a hot path
# SECURITY: Validate input to prevent SQL injection before using in query
# BUG: Edge case failure when list is empty - needs investigation
# REFACTOR: Extract this logic into separate utility function for reusability
# DEPRECATED: Use new_api_function() instead - this will be removed in v3.0
```

## Anti-Patterns to Avoid

### Dead Code Comments
```python
# Bad: Don't comment out code
# def old_function():
#     ...
def new_function():
    ...
```

### Changelog Comments
```python
# Bad: Don't maintain history in comments
# Modified by John on 2023-01-15
# Fixed bug reported by Sarah on 2023-02-03
def process_data():
    # ... implementation
    pass
```

### Divider Comments
```python
# Bad: Don't use decorative comments
# =====================================
# UTILITY FUNCTIONS
# =====================================
```

## Quality Checklist

Before committing, ensure your comments:
- [ ] Explain WHY, not WHAT
- [ ] Are grammatically correct and clear
- [ ] Will remain accurate as code evolves
- [ ] Add genuine value to code understanding
- [ ] Are placed appropriately (above the code they describe)
- [ ] Use proper spelling and professional language

## Summary

Remember: **The best comment is the one you don't need to write because the code is self-documenting.**
