```{eval-rst}
:orphan:

.. meta::
   :description: Documentation style guide covering file naming,
                 structure, semantic line breaks, reStructuredText and Markdown
                 conventions, terminology, and project-specific patterns.
```

<!--
  ~ Copyright 2026 Canonical Ltd.
  ~ See LICENSE file for licensing details.
-->

# Documentation style guide

This style guide documents the established conventions used in the project documentation. It captures actual patterns observed across the documentation set and serves as a reference for maintaining consistency in new contributions.

This guide is subordinate to your organization's documentation standards but records project-specific decisions and patterns that extend or clarify those standards.

---

## File naming and organization

**Directory structure**

The documentation follows the [Diátaxis](https://diataxis.fr/) framework with four main sections:

```
docs/
├── tutorial/          # Step-by-step learning paths
├── how-to/            # Task-oriented guides
├── explanation/       # Conceptual information
└── reference/         # Technical specifications
```

**File naming convention**

All filenames use lowercase letters and dashes for word separation.

Examples:

- Good: `part-1-get-started.rst`
- Good: `connect-editor.rst`
- Good: `network-interface.rst`
- Good: `container-vs-dockerfile.rst`
- Avoid: `ConnectEditor.rst` (uppercase)
- Avoid: `network_interface.rst` (underscore)

Optional: Tutorial files may use a sequential numbering pattern:

```
part-1-get-started.rst
part-2-work-with-features.rst
part-3-advanced-concepts.rst
part-4-production-deployment.rst
```

How-to files: Use verb-first naming pattern:

```
add-configuration.rst
connect-editor.rst
forward-ports.rst
debug-issues.rst
resolve-conflicts.rst
```

Explanation files use noun-based naming:

```
concepts.rst
interface-concepts.rst
best-practices.rst
runtime-behavior.rst
```

Reference files match command structure:

```
command-launch.rst
command-connect.rst
build-tool.md
```

Filenames and directory names in the documentation repo should be in lowercase,
with dashes instead of spaces; the directory tree must be built in a way that
provides for readable, meaningful URLs: `/docs/howto/change-tyres`.

---

## Page structure and metadata

**Standard page structure**

Every documentation page follows a consistent structure:
anchor label, metadata block, page title, opening paragraph, and section hierarchy.
Use the syntax references for exact markup and placement details.

**Metadata block**

Every page must have a metadata block immediately after the anchor label.
Use a brief, clear description (typically 1-2 lines),
wrapping at natural phrase boundaries.

**Anchor labels**

Use lowercase with underscores or dashes.

Optional: Prefix labels with section type.
Use prefixes consistently across the docs.

Prefixes:

- `tut_` - Tutorial sections
- `how_` - How-to guides
- `exp_` - Explanation articles
- `ref_` - Reference documentation

Examples: `tut_get_started`, `how_add_actions`, `exp_interface_concepts`, `ref_command_launch`.

---

## Writing style and tone

**Voice and audience**

Target audience is developers and technical professionals seeking to:

* Achieve specific goals without much overhead and roundabout musings
* Perform and conceive complex ad-hoc tasks and workflows that require precision and depth
* Attain understanding of the project's key capabilities beneficial for their scenarios

Content follows the Diátaxis framework, providing:

* Concise tutorials for common, starter-level actions and scenarios, eliminating the need to invent custom steps and allowing novice users to journey along the hot path effortlessly
* Elaborate explanations of the thinking behind the project's design, including design decisions, related concepts, and how it should be used
* Detailed how-to guides that address specific needs of advanced users and cover topics beyond basic entry-level operations
* Comprehensive reference of all options, settings, and details available to customize the project's operation in any desirable manner

The tone is authoritative but relaxed, confident but approachable. Think water cooler conversation, not classroom session.

Example:

```text
<PROJECT_NAME> is a tool for defining and handling development environments.

List your dependencies and components in YAML to define an environment. The key pieces of a definition are components, independent but connectable units of functionality. The project simplifies experiments with your environment layout.
```

**Active and passive voice**

Use active voice for user actions and cause-effect relationships. Use passive voice when the agent is unknown or the effect is the focus.

Active voice examples (preferred for actions):
- "The server hosts all files"
- "You install apps with the App Center"
- "The validate library checks the form for errors"

Passive voice examples (appropriate for effects):
- "Before upload, the form is checked for errors"
- "The files are deleted every time the script runs"

Pattern recognition: Passive voice uses `is/are/was/were/been/being + past participle` (e.g., "is installed", "are checked").

**Direct instructions**

Use imperative mood for instructions. Avoid "you can" or "you may" for required actions.

Preferred:

```
Install the application using the `--classic` option:
```

Avoid:

```
You can install the application with:
```

**Paragraph length**

Keep paragraphs focused and relatively short (2-5 sentences typically). Complex topics should be broken into multiple paragraphs.

Example:

```restructuredtext
Install the project,
upgrading the prerequisites if needed,
then ensure it runs.

Authenticate to the package manager and install
using the required options:
```

**Paragraph structure (optional)**

Where suitable, structure paragraphs using the Topic-Development-Example-Summary (TDES) pattern:

1. **Topic**: Open with a clear statement of what the paragraph addresses
2. **Development**: Explain the concept or provide necessary context
3. **Example**: Illustrate with a concrete example
4. **Summary**: Close with the key takeaway or implication

This pattern is particularly effective for explanatory content but should be applied flexibly; not every paragraph requires all four elements.

Example applying TDES:

```text
Interfaces enable communication between components and the host system.
Each interface defines a specific capability, such as network access or GPU usage.
For instance, the `network` interface allows a component to access external services,
while the `gpu` interface provides access to hardware acceleration.
Using interfaces, you can precisely control what resources each component can access.
```

In practice, simpler paragraphs may use just Topic-Example or Topic-Development, depending on the content's purpose and complexity.

**Clarity over cleverness**

- State prerequisites explicitly
- Define terms at first use
- Avoid assumptions about reader knowledge
- Use precise, unambiguous language

**Words and phrases to avoid**

Avoid clichés, violent metaphors, and jargon. Replace them with simpler alternatives:

- **Clichés**
  - `the ability to`, `is able to` → `can`
  - `in order to` → `to`
  - Avoid: `allow`, `going forward`, `not only...but also`

- **Violent metaphors**
  - `kill`, `terminate` → `stop`
  - `execute` → `run`
  - `eliminate` → `remove`

- **Jargon**
  - `leverage` → `use`
  - `end user` → `user`
  - `use case` → `example` or `scenario`
  - Avoid: `ecosystem`, `form factor`, `harness`, `next level`

This is not an exhaustive list; use your best judgment.

**Latin words and phrases**

Replace Latin phrases with English equivalents:

- `e.g.` → `for example`, `such as`
- `i.e.` → `that is`, `in other words`
- `etc.` → `and so on`
- `via` → `through`, `with`, `using`
- `ad hoc` → `unscheduled`, `temporary`, `bespoke`
- `per se` → `necessarily`, `intrinsically`
- `versus`, `vs.` → `compared to`, `opposed to`
- `vice versa` → `the reverse`, `the other way around`
- `circa` → `around`, `near`
- `cf.` → `refer to`

This is not an exhaustive list; use your best judgment.

**Demonstrative pronouns**

Avoid orphan "this", "these", "those", "that" when ambiguous. Pair with the noun for clarity.

Good: "The `yaml` object is sourced from the `yamllib` library. This object is only available if..."

Avoid: "The `yaml` object is sourced from the `yamllib` library. This is only available if..." (unclear if referring to object or library)

**Language and spelling**

Convention: Use US English spelling, grammar, and formatting conventions throughout the documentation.

Common US/UK differences:
- Patterns: `-ize` (not `-ise`), `-or` (not `-our`), `-able` (not `-eable`)
- US: `license` (noun and verb), `defense`, `program`, `percent`, `skeptical`, `catalog`, `traveling`, `labeled`
- UK: `licence` (noun), `defence`, `programme` (non-IT), `per cent`, `sceptical`, `catalogue`, `travelling`, `labelled`

Common technology terms:
- `email`, `online`, `website`, `internet`
- `setup` (noun), `set up` (verb)
- `backup` (noun), `back up` (verb)
- `login` (noun), `log in` (verb)
- `space-separated`, `comma-delimited`
- `open source` (noun), `open-source` (adjective)

Examples:
- Good: `color`, `center`, `analyze`, `behavior`
- Avoid: `colour`, `centre`, `analyse`, `behaviour`
- Good: Use serial comma: "components, interfaces, and environments"
- Good: Double quotes for quotations: "The project is a tool"

**Contractions**

Acceptable: `aren't`, `can't`, `couldn't`, `didn't`, `doesn't`, `don't`, `hadn't`, `hasn't`, `haven't`, `isn't`, `it's`, `mustn't`, `wasn't`, `won't`, `wouldn't`, `you're`, `you've`, `you'll`, `we're`, `we've`

Forbidden: `ain't` (colloquial), `gonna`, `gotta`, `something's` (confusion with possessive), `I'd`, `I'll` (avoid first person)

**Dates and numbers**

Date format:
- Single day: `1 January 2013`
- Range within month: `1-2 January 2013`
- Range across months: `1 January - 2 February 2013`

Numbers:
- Spell out below 10: `seven servers`
- Use digits from 10 onwards: `15 containers`
- Exception: Always use digits for units of measurement: `5 GB`, `3 seconds`
- Use commas for thousands: `7,000` not `7000`

---

## Semantic line breaks (optional)

**Pattern**

The documentation consistently uses semantic line breaks (one line per clause or significant phrase) in reStructuredText files. This improves version control diffs and editing precision.

Rationale: Semantic breaks make git diffs more readable and help reviewers identify exactly what changed in a sentence or paragraph.

**Implementation**

Break lines at natural semantic boundaries:
- After each complete clause
- Before coordinating conjunctions (and, but, or)
- Before relative clauses (which, that, who)
- After introductory phrases

Example:

```restructuredtext
This is the first section of the :ref:`four-part series <tut_index>`;
a practical introduction
that takes you on a tour
of the essential |project_markup| activities.
```

MyST equivalent:

````markdown
This is the first section of the {ref}`four-part series <tut_index>`;
a practical introduction
that takes you on a tour
of the essential |project_markup| activities.
````

**When to break**

Break after:
- Complete independent clauses
- Introductory prepositional phrases
- Transitional phrases
- Items in a complex series

Keep together:
- Short phrases that form a single unit
- Inline markup and its target word
- Cross-reference markup

Example:

```restructuredtext
Interfaces are a mechanism for communication and resource sharing.
It is an integral part of environment isolation,
ensuring that each environment operates in its own isolated context,
while still allowing controlled interactions among the components and with the host.
```

---

## Headings and titles

**Capitalization**

Pattern: Sentence case for all headings (capitalize only first word and proper nouns).

Exception: Product names and proper nouns maintain their capitalization.

**Heading constraints**

- Headings must not end with a period
- Avoid links in headings
- Use `code` styling sparingly in headings (only when essential, such as command references)
- Headings must not be followed directly by a subheading (provide introductory content)
- Do not skip heading levels (e.g., h1 followed by h3)

**How-to title pattern**

How-to guides follow the pattern: "How to [action] [object]" and use imperatives, not gerunds:
- Good: "How to create an instance" (imperative)
- Avoid: "How to creating an instance" or "Creating an instance" (gerund)
- How to forward ports with tunneling
- How to fix connection conflicts
- How to debug issues in environments

Linking exception: In navigation and links, drop "How to" prefix and use infinitive:

```restructuredtext
How-to guides:

* Debug issues in environments
* Connect IDE to an environment
```

---

## Markup and formatting policies

Use the syntax references for markup details.
Apply the policies below for project-specific conventions.

**Admonition placement:** Place admonitions at the end of the subsection they relate to, rather than interrupting the flow of text in the middle of a section. This is especially relevant for multiple admonitions per section.

**Inline markup**

Semantic markup preference: Use semantic markup roles (for example, sample values, environment variables, file paths, commands, GUI labels, and programs) instead of generic emphasis. Choose the most specific role that suits the purpose and use it consistently.

Use italics sparingly to introduce new terms and for emphasis. Leave bold for product names and commands.

Commands in command roles should be presented in their complete form and should not be used as verbs or nouns in the text.
Use non-breaking spaces to prevent longer compound commands from wrapping.

End directory path names with a slash where possible and conventional to disambiguate directories from files.

Format placeholders in uppercase within angle brackets, without underscores.

**Non-breaking spaces:** Use non-breaking spaces for important proper names and compound commands where line breaks would be awkward.

**"See also" sections (optional)**

"See also" sections can appear on pages under any pillar and link to related content not immediately essential but potentially useful. Break link lists down by pillar, listing pillars and individual subsections in alphabetical order.

**Tab headings**

Pattern: Keep tab headings noun-based and consistent across related content. Avoid unintended "sticky toggling" (where tab state persists inappropriately across different contexts).

**Sphinx extensions and roles**

Preference: Use Sphinx-specific [roles](https://www.sphinx-doc.org/en/master/usage/restructuredtext/roles.html) and [directives](https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html) over `docutils` generic equivalents. Use all their options and capabilities, listing options in alphabetical order.

**Spacing and formatting**

Section gaps: Include a non-cumulative two-line gap (two blank lines) after code samples, lists, tables, and before headings for visual clarity.

---

**Simplified markup for GitHub**

Use simplified markup for files that have special meaning on GitHub and need to be rendered there (such as `README.md`, `CONTRIBUTING.rst`, `SECURITY.rst`). For example, don't use `$` prompts in command samples for these files because GitHub doesn't prevent their selection during copying, which can confuse users.

---

**Command prompts and code blocks**

**DO NOT** use `$` or `#` prompts in code samples except when using the `console` lexer, which makes them non-selectable. Prompts cause problems for users who copy-paste code.

It is ONLY acceptable to use the `$` prompt when it's non-selectable. The `console` lexer in `.. code-block::` automatically handles this, making the prompt non-selectable during copy operations.

**Avoid inline comments** in bash code blocks. Use prose before, after, or between code blocks instead:

Avoid:

```bash
juju deploy wordpress
juju deploy ntp-master --to 2  # colocates with wordpress
```

Preferred:

```
Deploy wordpress, then colocate ntp-master with it:
```

```bash
juju deploy wordpress
juju deploy ntp-master --to 2
```

**Code block length**: Limit code blocks to approximately 40 lines. Longer blocks are rarely read; consider breaking them up or offering as downloadable files.

**Separate input and output**: Don't combine commands and their output in one block. Separate them with explanatory text:

Avoid:

```
juju status
environment: gce3
machines:
...
```

Preferred:

```
Check the current state:
```

```bash
juju status
```

```
This returns the current state of each unit:
```

```
environment: gce3
machines:
...
```

**Placeholders**

Use uppercase within angle brackets for placeholders: `<INSTANCE_NAME>`, `<PORT>`

For longer code blocks, consider defining placeholders as environment variables:

```bash
CHANNEL=1.30/stable
```

Then use them in commands:

```bash
juju download easyrsa --channel=$CHANNEL
juju download kubernetes-worker --channel=$CHANNEL
```

This approach:
- Separates user-supplied data from commands
- Enables blocks to be copied without modification
- Reduces the chance of user errors

**UI interaction guidance**

Don't use UI elements as verbs or nouns in prose. Link them to actions:

- Good: "Click **Save** to save your settings"
- Avoid: "**Save** your settings" (using button text as verb)

**Interaction verbs**:
- Use `Click` for buttons (or `Tap` for primarily mobile products)
- Use `Select` for dropdowns, multiple options, or menu navigation
- Use `Press` for keyboard shortcuts and keys (NOT `Click`)

Examples:
- Click **Settings** to open user settings
- Select the machines you want to register, then click **Save**
- Press `Ctrl + C` to copy
- Press the `Enter` key to continue
- Select **Preferences > Languages > English** (using `>` for navigation)

**Formatting**:
- Bold UI elements: **Save**, **File**, **Settings**
- Use `>` for menu navigation: **File > New > Document**
- Italics for quoted UI text: Click the link in _"You can register new computers..."_

**Checkboxes**: Use `Select`/`Clear` or `Check`/`Uncheck` (consistent pairs):
- Select the **Enable firewall** checkbox
- Clear the **Add bookmark** checkbox

**Icons vs buttons**: Minimize use of "icon" and "button" terminology unless needed for clarity. When using images, provide alt text or write the name directly after.

**Configuration examples**

Always include caption when known.

Indentation: Use commonly recognized formatting:
- YAML files: 2-space indentation
- JSON files: 4-space indentation

**Multi-line shell commands**

Use backslash continuation or explicit line breaks:

---

## Cross-references and links

**Internal cross-references**

Prefer `:ref:`/`{ref}` with semantic anchor labels.
Use `:doc:`/`{doc}` only when no target exists and no target can be added,
such as pages intentionally designed without anchors (for example, index or release notes).

**First mention pattern**

Link important terms only at first mention on a page. Avoid excessive linking.

**Reference label convention**

Use the following pattern for anchor labels: `.. _{prefix}_{descriptive_name}:`.
Prefix indicates the section type (ref/how/exp/tut).

---

**Command names**

In example blocks, use exact subcommand syntax:

```
project-tool launch
project-tool connect
build-tool build
```

When inline, shorter references are acceptable: the `launch` command.

**Command line terminology**

Convention: Use [POSIX utility conventions](https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap12.html) when discussing command-line syntax, options, arguments, and other CLI elements.

---

## Documentation quality principles

Use the following in addition to general Diátaxis guidelines:

**Clarity**

- State assumptions explicitly
- Define prerequisites clearly
- Avoid jargon without explanation
- Use consistent terminology

**Usability**

- Focus on actionable information
- Use direct imperatives for instructions
- Break complex tasks into clear steps
- Provide working examples

**Precision**

- Avoid ambiguous language
- Use exact commands and syntax
- Specify versions when relevant
- Maintain consistent structure
