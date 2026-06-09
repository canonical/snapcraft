---
description: 'Core guidelines for documentation-related suggestions.'
applyTo: 'docs/**/*.md'
---

# Documentation instructions for GitHub Copilot

## Purpose

This file provides general guidance for reviewing documentation.

## Core Directives

This section outlines the absolute order of operations. These rules have the highest priority and must not be violated.

1. **Provide suggestions, not solutions**: Whenever possible, do not generate the documentation on behalf of the user. Focus on asking questions, providing suggestions, and reviewing pre-existing files. If you want to make changes, ask for validation before proceeding with any changes.
2. **Adherence to Philosophy**: In the absence of a direct user directive or the need for factual verification, all other rules below regarding interaction and modification must be followed.

## General Interaction & Philosophy

- **Text on Request Only**: Your default response should be a clear, natural language explanation. Do NOT provide new files or content unless directly requested, and ask for validation before making any changes to the code base.
- **Direct and Concise**: Answers must be precise, to the point, and free from unnecessary filler or verbose explanations. Get straight to the solution without "beating around the bush".
- **Explain the "Why"**: Don't just provide an answer; briefly explain the reasoning behind it. Why is this suggestion clearer, more concise, or easier to understand?

## General Structure 

In this repository, the documentation is placed in a dedicated `docs` directory. The purpose of the files in this directory is to provide information, instructions, and conceptual understanding of the code for users. 

The top-level `docs` directory should contain an overview or home page called `index.md`. This file should provide a brief description of the project, what needs it serves, and who the primary user base is.

The rest of the documentation follows the Diataxis framework consisting of four categories of documentation:

1. **Tutorials**: A practice lesson that walks a user through a learning experience. Tutorials should be placed in the `docs/tutorial` directory, but sometimes they're placed in the top-level `docs` directory.
2. **How-to guides**: Addresses real-world goals or problems by providing practical directions. How-to guides should be placed in the `docs/how-to` directory.
3. **Reference**: Contains technical descriptions of theoretical knowledge. Reference documents should be placed in the `docs/reference` directory.
4. **Explanation**: Provides context and background on topics for the user to understand the bigger picture. Explanation documents should be placed in the `docs/explanation` directory.

All files should neatly fit into one of these four categories. Do NOT mix content between multiple categories into a single document. Instead, suggest splitting up content into multiple documents to preserve the structure and Diataxis framework.

### Guidance on tutorials

A tutorial is a learning-based, end-to-end experience that provides a complete learning journey for the user. The majority of our tutorials walk the user through a basic deployment of the charm in the repository.

The tutorial should contain the following pieces:

- **A title**. This title should be more descriptive than "Getting started"
  or "Quick guide" which are both indicative of a how-to guide. In a single
  statement, what will the user accomplish in the tutorial?
- **An introduction**. In a couple of sentences, introduce the charm and
  what the tutorial aims to do. Avoid using a statement like "you will learn",
  as that makes an assumption about the user's prior knowledge. Focus
  instead on what the user will do or accomplish. 
- **What you'll do**. This should be the first section in the tutorial. It
  aims to quickly and succinctly establish everything the user will do in the
  tutorial. 
- **What you'll need**. This section should contain a list of resources, software
  or other tools that the user will need before starting the tutorial. In this section,
  provide information on the supported architecture(s) like AMD64 or ARM64.
  Also include additional information if the
  tutorial will require significant resources (CPU, RAM, or disk space). You can
  also suggest a Multipass VM to create an isolated testing environment for the tutorial.
- **Set up tutorial model**. The tutorial should begin at this point. Have
  the user create Juju model with a predetermined name so that you can reference
  the model name throughout the tutorial.
- **Deploy the charm + its dependencies**. If the charm requires any other
  charms to successfully deploy, either include in a single section or split into
  multiple sections. Use this part of the tutorial to enable any required configurations.
  Make sure to briefly explain what the configurations or additional charms are
  and why the steps are necessary. Split up important commands into separate
  command blocks, and provide explanations between each command.
- **Check the deployment was successful**. Instruct the user to run
  ``juju status`` and show example output. If appropriate, also instruct the
  user to run ``kubectl get pods -n <juju model name>`` and show the output.
  Offer a brief explanation so that the user knows that the deployment was
  successful.
- **Some final checkpoint**. Have the user visit a website, change a
  configuration, or run an action (maybe some combination of the three). Check
  that this step worked; this is more obvious for a website, but you may need
  to be creative if you have the user change a configuration or run an action.
- **Tear down the environment**. This should be the final section. Take a moment
  to congratulate the user for getting through the tutorial! Have the user
  destroy the Juju model that they set up, and if applicable, instruct them to
  delete the Multipass VM.

### Guidance on how-to guides

- How-to guides should contain a title with the format `# How to...`. The title should
concisely summarize the real-world problem addressed by the document.
- How-to guides should contain at least one CLI command that user can run to address the
problem or achieve the goal.
- Provide instructions or tasks that the user can take to achieve the goal. Avoid gerunds.


## Style guide pointers

- **Spelling**: Use US English for spelling suggestions.
- **Headings**: Use sentence case for headings. Avoid punctuation in headings. Do not skip levels in heading hierarchy. Do not use consecutive headings without intervening text, and suggest removing headings that don't enhance the understanding. Don't overuse `code` styling in headings.
- **Code examples**: Do not use prompt marks (for example, `$` or `#`) in code examples. Do not use comments in code examples; instead, put any comments directly into the text. Whenever possible, split up multiple commands into separate code blocks and provide explanations in between the code blocks. Use Git-flavoured Markdown with fenced code blocks (```) and command examples as shell blocks.
- **Lists**: Only use numbered lists when there is an inherent order to the items.
- **Latin words and phrases**: We can't assume the reader is familiar with Latin words and phrases. Words such as "etc.", "i.e.", "e.g.", "per", "versus", and "via" should be avoided. 

## Small-edit rules for AI agents

- Avoid describing tasks as "easy" or "simple".
- Preserve existing link targets and code samples formatting. If you change any heading filename or path, update all relative links in `docs/` accordingly.
- Demonstrative pronouns ("this", "these", "those", and "that") should be accompanied by the target noun when there's a risk of ambiguity in the paragraph. An isolated demonstrative pronoun can be used if there's no chance of ambiguity. In cases where a paragraph describes multiple ideas, avoid isolated demonstrative pronouns and be as specific as possible.

## Changelog guidance 

If you modify a procedure (commands, required versions, prerequisites) and a `docs/changelog.md` exists, add a short note in `docs/changelog.md` summarizing the user-facing change.

If you add a new file into `docs`, add a short note in `docs/changelog.md` summarizing the user-facing change.

If there are no user-relevant changes, then the changelog does not need to be updated.


