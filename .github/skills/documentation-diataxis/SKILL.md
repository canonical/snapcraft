---
name: documentation-diataxis
description: "Analyzes documentation against Diataxis framework (Tutorial, How-to, Reference, Explanation). Use when reviewing documentation structure or classifying content type. Identifies misalignments between declared category and actual content."
---

# Diataxis Classification Review

## Scope

Diataxis classification only: identify whether each page is a tutorial,
how-to guide, explanation, or reference; note structural mismatches
between content type and declared category, and suggest improvements
where real issues exist.
Ground classification in the Diataxis foundations: the two axes of craft
(action vs cognition, acquisition vs application) define four user needs
(learning, goals, information, understanding).

This skill identifies flaws and provides actionable recommendations;
it does not enforce compliance or apply pass/fail criteria.

## Inputs

- Documentation file(s) under review.
- Diataxis framework principles (embedded in classification criteria below).

## Actions

1. **Identify intended category**: Determine the declared category
  based on directory location
  (`tutorial/`, `how-to/`, `explanation/`, `reference/`)
  and file metadata (front matter keys such as `category`, `type`,
  `diataxis`, or reST `.. meta::` entries).

2. **Infer actual category**: Analyse the text's structure, tone,
   and progression to determine which quadrant it actually resembles.
   Use these classification criteria:

   - **Tutorial indicators**:
     - Step-by-step progression building a complete project
     - Imperative mood ("Create a file", "Run this command")
     - Learning-focused language ("you will learn", "by the end")
     - Safe, controlled environment (specific versions, no branching)
     - Frequent reassurance and checkpoints
     - Teaches by doing, not explaining

   - **How-to guide indicators**:
     - Problem-solution format with clear goal
     - Assumes existing knowledge and competence
     - Clearly states prerequisites and applicability scope
     - Action-oriented ("To achieve X, do Y")
     - Flexible, allows for variation
     - Focuses on results, not learning
     - Omits explanations unless critical to success

   - **Reference indicators**:
     - Descriptive, declarative statements
     - Comprehensive coverage of subject
     - Neutral, technical tone
     - Structured for lookup (tables, lists, alphabetical)
     - Parameters, options, API signatures
     - Accuracy over narrative

   - **Explanation indicators**:
     - Conceptual focus ("why" and "how it works")
     - Discursive, exploratory tone
     - Comparative analysis
     - Context, background, relationships
     - Illuminates understanding, not action
     - May include history, design decisions, alternatives
     - May contain subjective opinions and personal perspectives

3. **Check user need alignment**:

   Map the page to the user need implied by the action/cognition and
   acquisition/application axes (learning, goals, information,
   understanding).

   - **Tutorials**: Is it a learning-oriented lesson?
     Does it build confidence through doing? Is it linear and safe?
   - **How-to guides**: Is it a task-oriented recipe?
     Does it help a competent user solve a specific problem?
     Is it goal-focused?
   - **Reference**: Is it information-oriented?
     Does it describe things accurately and completely?
     Is it structured for lookup?
   - **Explanation**: Is it understanding-oriented?
     Does it clarify concepts, context, and relationships?
     Is it discursive?

4. **Note hard-to-fit genres**: Some documentation types do not align
   cleanly with a single quadrant (for example, release notes or
   contributing guides). Flag these cases explicitly, reference the
   Diataxis guidance on complex hierarchies
   (https://diataxis.fr/complex-hierarchies/), and choose the closest
   fit category for reporting.

5. **Evaluate quality**:

   - **Functional quality**: Is the content accurate, complete,
     consistent, useful, and precise?
     - Missing prerequisites or dependencies
     - Incomplete steps or procedures
     - Inconsistent terminology or naming
     - Outdated information (version mismatches, deprecated features)

   - **Deep quality**: Does the content have good flow?
     Does it anticipate user questions?
     Is the cognitive load appropriate? Is the experience clear?
     - Paragraph length (>4 sentences may suggest need for breaking)
     - Sentence complexity (nested clauses, dense jargon)
     - Transition quality (abrupt topic changes, missing connectives)
     - Progressive disclosure (introducing too much too soon)
     - User journey mapping (gaps in expected flow)

6. **Document misalignments**: Explicitly identify where the document
   fails to meet the needs of its category, jumps between categories,
   or where quality breaks down. For each issue, provide:
   - Specific location (section, paragraph)
   - Nature of the problem
   - Impact on user experience
   - Concrete suggestion for improvement

7. **Verify completion**: Confirm the analysis completed:

   - Category classification completed (declared vs inferred)
   - User need alignment analyzed
   - Quality assessment performed (functional and deep quality)
   - Misalignments documented with recommendations

   State the completion status:
   - `✓ Diataxis analysis complete: [declared category] → [inferred category], [N] issues found`
   - OR `✓ Diataxis analysis complete: Content aligns well with [category]`

## Constraints

- Do not ignore the Diataxis framework.
- Assign each page to exactly one quadrant.
- Do not introduce categories beyond the four quadrants.

## Output

A Diataxis Analysis Report detailing:

- Declared category (from metadata/directory structure).
- Inferred category (from content analysis).
- User need alignment analysis (which quadrant best serves the user).
- Functional quality findings (with specific examples).
- Deep quality findings (with specific examples).
- Identified issues and actionable recommendations for improvement.

If no significant issues are found, state that the documentation
aligns well with its intended category.
