"""
These classes provide extended versions of the `literalinclude` and `code`
directives that apply substitutions to the input text. This makes it possible
to define substitutions for a particular document then include a template that
uses these extensions to generate output specific to that document.

For example, defining `project` as

  .. |project| replace:: my-project

then including a file containing

  .. code-par::

     mkdir |project|; cd |project|

will produce the equivalent output to

  .. code::

     mkdir my-project; cd my-project

The `litinclude-par` directive extends the `literalinclude` directive in a
different way, making substitutions in the arguments to the directive before
passing them to the underlying `literalinclude` implementation. This allows
file input to be parameterised so that a template document can quote and refer
to files defined by the different documents that include it.
"""

from docutils.parsers.rst import directives
import re
from sphinx.directives.code import CodeBlock, LiteralInclude


# Define a regular expression for locating placeholders.
placeholder_re = re.compile("\\|([^|]+)\\|")


class LiteralIncludeParam(LiteralInclude):
    """
    Like `literalinclude`, but expects a replacement/substitution to be
    used as the file name.
    """

    def run(self):
        # type: () -> List[nodes.Node]
        document = self.state.document

        # Replace placeholders in options.
        for option, value in self.options.items():
            # Find all the placeholders and produce a set of unique matches.
            matches = placeholder_re.findall(value)
            unique = set(matches)

            for match in unique:
                try:
                    # For each match, find the substitution(s) to use, starting from
                    # the definition and reading all the arguments as plain text.
                    node = document.substitution_defs[match].next_node()
                    subs = str(node).split()

                    # Substitute new text for the placeholders.
                    for text in subs:
                        value = value.replace("|" + match + "|", text)

                except KeyError:
                    ### We should probably warn about undefined placeholders.
                    pass

            self.options[option] = value

        subst_text = self.arguments[0]
        if subst_text[:1] != "|" or subst_text[-1:] != "|":
            return [
                document.reporter.warning(
                    "Argument should be a replacement/subsitution with the format "
                    "|<text>|",
                    line=self.lineno,
                )
            ]
        else:
            # Dereference the substitution text to produce a filename.
            subst_name = subst_text[1:-1]
            try:
                filename = str(document.substitution_defs[subst_name].next_node())
            except KeyError:
                return [
                    document.reporter.warning(
                        "Replacement/substitution {0} not defined".format(subst_name),
                        line=self.lineno,
                    )
                ]

            self.arguments[0] = filename
            return LiteralInclude.run(self)


class CodeParam(CodeBlock):
    """
    Like `code-block`, but replaces |<text>| with the corresponding text
    defined in a replacement/substitution.
    """

    required_arguments = 0
    optional_arguments = 1

    # Define a regular expression for locating placeholders.
    placeholder_re = re.compile("\\|([^|]+)\\|")

    def run(self):
        # type: () -> List[nodes.Node]
        document = self.state.document

        # Find placeholders in the text and replace them with replacements
        # defined for the document.

        for i, line in enumerate(self.content):
            # Find all the placeholders on the line and reduce it to a set of
            # unique matches.
            matches = placeholder_re.findall(line)
            unique = set(matches)

            # Start with the original line and accumulate more if multiple
            # substitutions occur.
            lines = [line]

            for match in unique:
                try:
                    # For each match, find any substitutions to use, starting from
                    # the definition and reading all the arguments as plain text.
                    node = document.substitution_defs[match].next_node()
                    subs = self.parse_args(str(node))

                    # Substitute new text for the placeholders on each line.
                    # If there are multiple substitutions for a placeholder then
                    # insert new lines for those.
                    new_lines = []

                    for old_line in lines:
                        for text in subs:
                            new_lines.append(old_line.replace("|" + match + "|", text))

                    # Update the list of lines with substitutions and any new
                    # lines created.
                    lines = new_lines

                except KeyError:
                    ### We should probably warn about undefined placeholders.
                    pass

            # Replace the original line with the processed list of new lines.
            self.content.data[i : i + 1] = lines
            self.content.items[i : i + 1] = [self.content.items[i]] * len(lines)

        # In Sphinx 2.0 the language argument to code-block is optional, but
        # in earlier versions we need to add one if no argument is given.
        if len(self.arguments) < 1:
            self.arguments = ["text"]

        return CodeBlock.run(self)

    def parse_args(self, text):
        # Parse the text, treating spaces as argument separators unless
        # the spaces are part of a quoted string.
        ### TODO: Add support for escaped quote characters if required.
        if not text:
            return []

        in_quote = False
        args = [""]

        for c in text:
            if in_quote:
                if c == '"':
                    args.append("")
                    in_quote = False
                else:
                    args[-1] += c
            elif c == '"':
                in_quote = True
            elif c == " ":
                args.append("")
            else:
                args[-1] += c

        if args[-1] == "":
            args.pop()

        return args


def setup(app):
    directives.register_directive("litinclude-par", LiteralIncludeParam)
    directives.register_directive("code-par", CodeParam)
    return {"version": "0.2"}
