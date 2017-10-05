# Snapcraft coding style guidelines

When writing code for snapcraft, we try to follow a set of rules that will lead
to consistency and readability.

This is a permanent work in progress, and sometimes being too strict with the
rules can end up making things actually less readable. So when you disagree
with one of the rules, please talk to us and help us making it better.

Some of the rules are enforced with static tests. You can read the [TESTING][1]
document for more information and details about how to run the static suite of
tests. Some other rules are only socially enforced during code reviews.

## PEP 8

We adhere to the Style Guide for Python Code documented in the [PEP 8][2].

## Multiline strings

For multiline strings, we prefer to use `textwrap.dedent`:

    ```
    # end first line with \ to avoid the empty line!
    s = textwrap.dedent("""\
        hello
          world
        """)
    print(repr(s))  # prints 'hello\n  world\n'
    ```

(from https://docs.python.org/3/library/textwrap.html#textwrap.dedent)

## Tests

* When asserting for equality, we prefer to use the `Equals` matcher from
  testtools:

    ```
    self.assertThat(actual, Equals(expected))
    ```

[1]: TESTING.md
[2]: https://www.python.org/dev/peps/pep-0008